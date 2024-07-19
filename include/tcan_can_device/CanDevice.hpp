#pragma once

#include <stdint.h>
#include <atomic>
#include <memory>
#include <string>

#include "tcan_can_device/CanDeviceOptions.hpp"
#include "tcan_can_device/CanDriver.hpp"
#include "tcan_can_device/CanDriverOptions.hpp"
#include "tcan_can_device/CanMsg.hpp"

#include "message_logger/message_logger.hpp"

namespace tcan_can_device {
class CanDriver;

//! A device that is connected via CAN.
class CanDevice {
 public:
  enum State { Initializing = 0, Active = 1, Error = -2, Missing = -1 };

  /*! Constructor
   * @param nodeId	ID of CAN node
   * @param name		human-readable name of the device
   */
  CanDevice() = delete;

  CanDevice(const uint32_t nodeId, const std::string& name)
      : CanDevice(std::unique_ptr<CanDeviceOptions>(new CanDeviceOptions(nodeId, name))) {}

  CanDevice(std::unique_ptr<CanDeviceOptions>&& options)
      : options_(std::move(options)), deviceTimeoutCounter_(0), state_(Initializing), driver_(nullptr) {}

  virtual ~CanDevice() = default;

  /*! Initialize the device. This function is automatically called by Bus::addDevice(..)
   *   (through initDeviceInternal(..))
   * This function is intended to do some initial device initialization (register messages to be received,
   *   restart remote node, ...)
   * @return true if successfully initialized
   */
  virtual bool initDevice() = 0;

  /*!
   * Configure the device
   * This function is automatically called after reception of a
   * bootup message. (or more general: After reception of any message if the device was in state Missing or Initializing)
   * @param msg   received message which caused the call of this function
   * @return      true if device is active
   */
  virtual bool configureDevice(const CanMsg& msg) = 0;

  /*! Do a sanity check of the device. This function is intended to be called with constant rate
   * and shall check heartbeats, SDO timeouts, ...
   * This function is automatically called if the Bus has asynchronous=true and sanityCheckInterval > 0
   * @return true if hasError() and isMissing() are false
   */
  virtual bool sanityCheck() {
    if (!isMissing()) {
      if (isTimedOut()) {
        state_ = Missing;
        MELO_WARN("Device %s timed out!", getName().c_str());
      }
    }

    return !(hasError() || isMissing());
  }

  inline uint32_t getNodeId() const { return options_->nodeId_; }
  inline const std::string& getName() const { return options_->name_; }

  inline bool isInitializing() const { return (state_ == Initializing); }
  inline bool isActive() const { return (state_ == Active); }
  inline bool hasError() const { return (state_ == Error); }
  inline bool isMissing() const { return (state_ == Missing); }

  virtual int getStatus() const { return static_cast<int>(state_.load()); }

  /*!
   * Resets the device to Initializing state
   */
  virtual void resetDevice() { state_ = Initializing; }

 public:  /// Internal functions
  inline void configureDeviceInternal(const CanMsg& msg) {
    if (state_ != Active && state_ != Error) {
      if (configureDevice(msg)) {
        state_ = Active;
        if (options_->printConfigInfo_) {
          MELO_INFO("Device %s configured successfully.", options_->name_.c_str());
        }
      }
    }
  }

  inline void resetDeviceTimeoutCounter() { deviceTimeoutCounter_ = 0; }

 protected:
  /*!
   * @return True if the device timed out
   */
  inline bool isTimedOut() {
    return (options_->maxDeviceTimeoutCounter_ != 0 && (deviceTimeoutCounter_++ > options_->maxDeviceTimeoutCounter_));
    // deviceTimeoutCounter_ is only increased if options_->maxDeviceTimeoutCounter != 0
  }

 protected:
  const std::unique_ptr<CanDeviceOptions> options_;

  std::atomic<unsigned int> deviceTimeoutCounter_;

  std::atomic<State> state_;

  //!  reference to the CAN bus the device is connected to
  CanDriver* driver_;

  std::thread sanityCheckThread_;
  std::atomic_bool isCheckingSanity_{false};
  void sanityCheckWorker() {
    auto nextLoop = std::chrono::steady_clock::now();
    while (isCheckingSanity_) {
      nextLoop += std::chrono::milliseconds(options_->sanityCheckInterval_);
      std::this_thread::sleep_until(nextLoop);
      sanityCheck();
    }
  }
};

} /* namespace tcan_can_device */
