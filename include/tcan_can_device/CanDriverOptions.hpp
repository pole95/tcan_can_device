#pragma once

#include <linux/can.h>
#include <sys/time.h>
#include <sys/types.h>
#include <string>
#include <vector>

namespace tcan_can_device {
struct CanDriverOptions {
  CanDriverOptions() : CanDriverOptions(std::string()) {}

  CanDriverOptions(const std::string& interfaceName)
      : interfaceName_(interfaceName),
        priorityReceiveThread_(99),
        priorityTransmitThread(98),
        maxQueueSize_(1000),
        readTimeout_{1, 0},
        writeTimeout_{1, 0} {}

  ~CanDriverOptions() = default;

  std::string interfaceName_;
  int priorityReceiveThread_;
  int priorityTransmitThread;

  ssize_t maxQueueSize_;

  timeval readTimeout_;
  timeval writeTimeout_;

  bool loopback_;
  ssize_t sndBufLength_;
  unsigned int canErrorMask_;
  std::vector<can_filter> canFilters_;
};
}  // namespace tcan_can_device