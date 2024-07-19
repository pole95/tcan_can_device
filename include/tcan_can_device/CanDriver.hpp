#pragma once

#include <atomic>
#include <tcan_can_device/CanDevice.hpp>
#include <tcan_can_device/CanFrameIdentifier.hpp>
#include <tcan_can_device/CanMsg.hpp>

#include <linux/can.h>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>

#include "CanDriverOptions.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan_can_device {
class CanDevice;

class CanDriver {
 public:
  using MsgQueue = std::deque<CanMsg>;
  using CallbackPtr = std::function<bool(const CanMsg&)>;
  using CanFrameIdentifierToFunctionMap =
      std::unordered_map<CanFrameIdentifier, std::pair<CanDevice*, CallbackPtr>, CanFrameIdentifierHasher>;

  CanDriver() = delete;
  CanDriver(std::unique_ptr<CanDriverOptions>&& options);

  ~CanDriver() { stopThreads(); }

  void startThreads();
  void stopThreads(bool wait = true);

  /*! Copy a message to be sent to the output queue
   * @param msg	const reference to the message to be sent
   */
  inline bool sendMessage(const CanMsg& msg) {
    std::lock_guard<std::mutex> guard(outgoingMsgsMutex_);
    return sendMessageWithoutLock(msg);
  }

  /*! Adds a device and callback function for incoming messages identified by its CAN frame identifier. The timeout
   *  counter of the device is reset on reception of the message (treated as heartbeat).
   * @param canFrameId        29 or 11 bit frame ID of the message
   * @param device            pointer to the device
   * @param fp                pointer to the parse function
   * @return true if successful
   */
  template <typename T>
  inline bool addCanMessage(const uint32_t canFrameId, T* device, bool (std::common_type<T>::type::*fp)(const CanMsg&)) {
    //clang-format off
    return canFrameIdentifierToFunctionMap_
        .emplace(CanFrameIdentifier{canFrameId}, std::make_pair(device, std::bind(fp, device, std::placeholders::_1)))
        .second;
    //clang-format on
  }

  /*! Like addCanMessage with a specific CanId, but matches against a range of CanIds through a mask.
   * To match all messages, 0x..FA..33, one would pass CanFrameIdentifier { 0x00FA0033, 0x00FF00FF }, i.e. the ID and the mask.
   * Bits in the ID that correspond to zeros in the mask are ignored.
   * @param matcher           CanFrameIdentifier for the message
   * @param device            pointer to the device
   * @param fp                pointer to the parse function
   * @return true if successful
   */
  template <typename T>
  inline bool addCanMessage(const CanFrameIdentifier matcher, T* device, bool (std::common_type<T>::type::*fp)(const CanMsg&)) {
    return canFrameIdentifierToFunctionMap_.emplace(matcher, std::make_pair(device, std::bind(fp, device, std::placeholders::_1))).second;
  }

  void sendSync() { sendMessage(CanMsg(0x80, 0, nullptr)); }

 private:
  std::thread receiveThread_;
  std::thread transmitThread_;
  std::thread sanityCheckThread_;
  std::atomic_bool isRunning_;

  CanFrameIdentifierToFunctionMap canFrameIdentifierToFunctionMap_;

  CallbackPtr unmappedMessageCallbackFunction_;

  std::mutex outgoingMsgsMutex_;
  MsgQueue outgoingMsgs_;

  std::condition_variable condTransmitThread_;

  void transmitWorker();
  void receiveWorker();
  void sanityCheckWorker();

  void readMessage();
  void readData();

  void writeData(std::unique_lock<std::mutex>* lock);

  void handleErrorFrame(const can_frame& frame);
  void handleMessage(const CanMsg& msg);

  int socket_;
  int recvFlag_;
  int sendFlag_;

  const std::unique_ptr<CanDriverOptions> options_;

  void initializeInterface();

  inline bool isBufferFull() const {
    if (outgoingMsgs_.size() >= options_->maxQueueSize_) {
      MELO_WARN_THROTTLE(1000, "Exceeding max queue size! Dropping message!");
      return true;
    }
    return false;
  }

  inline bool sendMessageWithoutLock(const CanMsg& msg) {
    if (!isBufferFull()) {
      outgoingMsgs_.push_back(msg);
      condTransmitThread_.notify_all();
      return true;
    }

    return false;
  }
  bool defaultHandleUnmappedMessage(const CanMsg& /*msg*/) { return true; }
};
}  // namespace tcan_can_device