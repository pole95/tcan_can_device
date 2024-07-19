#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cerrno>
#include <cstring>
#include <memory>
#include <message_logger/message_logger.hpp>
#include <mutex>
#include <stdexcept>
#include <tcan_can_device/CanDriver.hpp>
#include <tcan_can_device/CanDriverOptions.hpp>
#include <tcan_can_device/CanMsg.hpp>

using namespace tcan_can_device;

CanDriver::CanDriver(std::unique_ptr<CanDriverOptions>&& options)
    : socket_(-1),
      recvFlag_(0),
      sendFlag_(0),
      isRunning_(false),
      options_(std::move(options)),
      canFrameIdentifierToFunctionMap_(),
      unmappedMessageCallbackFunction_(std::bind(&CanDriver::defaultHandleUnmappedMessage, this, std::placeholders::_1)),
      outgoingMsgsMutex_(),
      outgoingMsgs_(),
      receiveThread_(),
      transmitThread_(),
      sanityCheckThread_(),
      condTransmitThread_() {
  initializeInterface();
  startThreads();
}

void CanDriver::startThreads() {
  isRunning_ = true;
  receiveThread_ = std::thread(&CanDriver::receiveWorker, this);
  transmitThread_ = std::thread(&CanDriver::transmitWorker, this);
}

void CanDriver::stopThreads(bool wait) {
  isRunning_ = false;
  condTransmitThread_.notify_all();
  if (wait) {
    if (receiveThread_.joinable()) {
      receiveThread_.join();
    }

    if (transmitThread_.joinable()) {
      transmitThread_.join();
    }
  }
}

void CanDriver::initializeInterface() {
  const char* interface = options_->interfaceName_.c_str();

  /* open socket */
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    MELO_FATAL("Opening CAN channel %s failed: %d", interface, socket_);
    throw std::runtime_error("Error in socket creation!");
  }

  /* configure socket */
  struct ifreq ifr;
  strcpy(ifr.ifr_name, interface);
  ioctl(socket_, SIOCGIFINDEX, &ifr);

  // loopback
  int loopback = options_->loopback_;
  if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) != 0) {
    MELO_WARN("Failed to set loopback mode");
    perror("setsockopt");
  }

  // receive own messages
  int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
  if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) != 0) {
    MELO_WARN("Failed to set reception of own messages option: (%d)\n  %s", errno, strerror(errno));
  }

  // CAN error handling
  can_err_mask_t err_mask = options_->canErrorMask_;
  if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0) {
    MELO_WARN("Failed to set error mask: (%d)\n  %s", errno, strerror(errno));
  }

  // On some CAN drivers, the txqueuelen of the netdevice cannot be changed (default=10).
  // The buffer size of the socket is larger than those 10 messages, so the write(..) call never blocks but raises a ENOBUFS error when
  // writing to the netdevice. This renders poll(..) useless. Not the writes in the sockets are the limiting pipe but the writes in the
  // underlying netdevice. The recommended way to fix this is to increase the txqueuelen of the netdevice (e.g. command line: ip link set
  // can0 txqueuelen 1000) If this is not possible, the sndbuf size of the socket can be decrease, such that the socket writes are the
  // limiting pipe, leading to blocking socket write(..) calls. (write becomes poll(..)-able)
  // https://www.mail-archive.com/socketcan-users@lists.berlios.de/msg00787.html
  // http://socket-can.996257.n3.nabble.com/Solving-ENOBUFS-returned-by-write-td2886.html
  if (options_->sndBufLength_ != 0) {
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &(options_->sndBufLength_), sizeof(options_->sndBufLength_)) != 0) {
      MELO_WARN("Failed to set sndBuf length: (%d)\n  %s", errno, strerror(errno));
    }
  }

  // set read timeout
  if (options_->readTimeout_.tv_sec != 0 || options_->readTimeout_.tv_usec != 0) {
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char*)&options_->readTimeout_, sizeof(options_->readTimeout_)) != 0) {
      MELO_WARN("Failed to set read timeout: (%d)\n  %s", errno, strerror(errno));
    }
  }

  // set write timeout
  if (options_->writeTimeout_.tv_sec != 0 || options_->writeTimeout_.tv_usec != 0) {
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, (char*)&options_->writeTimeout_, sizeof(options_->writeTimeout_)) != 0) {
      MELO_WARN("Failed to set write timeout: (%d)\n  %s", errno, strerror(errno));
    }
  }

  // set up filters
  if (options_->canFilters_.size() != 0) {
    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &(options_->canFilters_[0]), sizeof(can_filter) * options_->canFilters_.size()) !=
        0) {
      MELO_WARN("Failed to set CAN raw filters: (%d)\n  %s", errno, strerror(errno));
    }
  }

  /* bind socket */
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(struct sockaddr_can));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    MELO_FATAL("Error in socket %s bind: (%d)\n  %s", interface, errno, strerror(errno));
    throw std::runtime_error("Error in socket bind: " + std::string(strerror(errno)));
  }

  MELO_INFO("Opened socket %s.", interface);
}

void CanDriver::receiveWorker() {
  while (isRunning_) {
    readMessage();
  }
}

void CanDriver::transmitWorker() {
  std::unique_lock<std::mutex> lock{outgoingMsgsMutex_};
  while (isRunning_) {
    while (outgoingMsgs_.size() == 0 && isRunning_) {
      condTransmitThread_.wait(lock);
    }
    if (isRunning_) {
      writeData(&lock);
    }
  }
}

void CanDriver::writeData(std::unique_lock<std::mutex>* lock) {
  CanMsg msg = outgoingMsgs_.front();

  if (lock != nullptr) {
    lock->unlock();
  }

  can_frame frame;
  frame.can_id = msg.getCobId();
  frame.can_dlc = msg.getLength();
  std::copy(msg.getData(), &(msg.getData()[frame.can_dlc]), frame.data);
  ssize_t writtenBytes = send(socket_, &frame, sizeof(can_frame), sendFlag_);

  if (lock != nullptr) {
    lock->lock();
  }
  if (writtenBytes != sizeof(struct can_frame)) {
    MELO_ERROR("Failed to write data: (%d)\t%s", errno, strerror(errno));
  }

  outgoingMsgs_.pop_front();
}

void CanDriver::readMessage() {
  can_frame frame;
  ssize_t readBytes = recv(socket_, &frame, sizeof(can_frame), recvFlag_);
  if (readBytes <= 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      MELO_ERROR("Failed to read data: (%d)\t%s", errno, strerror(errno));
    }
  }

  if (frame.can_id > CAN_ERR_FLAG && frame.can_id < CAN_RTR_FLAG) {
    handleErrorFrame(frame);
  } else {
    handleMessage(CanMsg(frame.can_id, frame.can_dlc, frame.data));
  }
}

void CanDriver::handleMessage(const CanMsg& msg) {
  auto it = std::find_if(canFrameIdentifierToFunctionMap_.cbegin(), canFrameIdentifierToFunctionMap_.cend(),
                         [&msg](auto p) { return !((msg.getCobId() ^ p.first.identifier) & p.first.mask); });

  if (it != canFrameIdentifierToFunctionMap_.end()) {
    it->second.first->resetDeviceTimeoutCounter();
    it->second.first->configureDeviceInternal(msg);
    it->second.second(msg);  // call function pointer
  } else {
    unmappedMessageCallbackFunction_(msg);
  }
}

void CanDriver::handleErrorFrame(const can_frame& frame) {
  std::stringstream errorMsg;
  errorMsg << "received error frame (";
  for (int i = 0; i < 8; i++) errorMsg << " " << int(frame.data[i]);
  errorMsg << ")";
  // cob id
  if (frame.can_id & CAN_ERR_TX_TIMEOUT) {
    errorMsg << "TX timeout (by netdevice driver). ";
  }
  if (frame.can_id & CAN_ERR_LOSTARB) {
    errorMsg << "lost arbitration. ";
  }
  if (frame.can_id & CAN_ERR_CRTL) {
    errorMsg << "controller problems. ";
  }
  if (frame.can_id & CAN_ERR_PROT) {
    errorMsg << "protocol violations. ";
  }
  if (frame.can_id & CAN_ERR_TRX) {
    errorMsg << "transceiver status. ";
  }
  if (frame.can_id & CAN_ERR_ACK) {
    errorMsg << "received no ACK on transmission. ";
  }
  if (frame.can_id & CAN_ERR_BUSOFF) {
    errorMsg << "bus off. ";
  }
  if (frame.can_id & CAN_ERR_BUSOFF) {
    errorMsg << "bus error (may flood!). ";
  }
  if (frame.can_id & CAN_ERR_RESTARTED) {
    errorMsg << "controller restarted. ";
  }

  // bit 0
  errorMsg << "bit number in bitstream: 0x" << std::hex << static_cast<int>(frame.data[0]);

  // bit 1
  switch (frame.data[1]) {
    default:  // fall-through!
    case CAN_ERR_CRTL_UNSPEC:
      errorMsg << " / error status of CAN-controller: unspecified";
      break;

    case CAN_ERR_CRTL_RX_OVERFLOW:
      errorMsg << " / error status of CAN-controller: rx buffer overflow";
      break;

    case CAN_ERR_CRTL_TX_OVERFLOW:
      errorMsg << " / error status of CAN-controller: tx buffer overflow";
      break;

    case CAN_ERR_CRTL_RX_WARNING:
      errorMsg << " / error status of CAN-controller: reached warning level for RX errors";
      break;

    case CAN_ERR_CRTL_TX_WARNING:
      errorMsg << " / error status of CAN-controller: reached warning level for TX errors";
      break;

    case CAN_ERR_CRTL_RX_PASSIVE:
      errorMsg << " / error status of CAN-controller: reached error passive status RX";
      break;

    case CAN_ERR_CRTL_TX_PASSIVE:
      errorMsg << " / error status of CAN-controller: reached error passive status TX";
      break;
  }

  // bit 2
  switch (frame.data[2]) {
    default:  // fall-through!
    case CAN_ERR_PROT_UNSPEC:
      errorMsg << " / error in CAN protocol (type): unspecified";
      break;

    case CAN_ERR_PROT_BIT:
      errorMsg << " / error in CAN protocol (type): single bit error";
      break;

    case CAN_ERR_PROT_FORM:
      errorMsg << " / error in CAN protocol (type): frame format error";
      break;

    case CAN_ERR_PROT_STUFF:
      errorMsg << " / error in CAN protocol (type): bit stuffing error";
      break;

    case CAN_ERR_PROT_BIT0:
      errorMsg << " / error in CAN protocol (type): unable to send dominant bit";
      break;

    case CAN_ERR_PROT_BIT1:
      errorMsg << " / error in CAN protocol (type): unable to send recessive bit";
      break;

    case CAN_ERR_PROT_OVERLOAD:
      errorMsg << " / error in CAN protocol (type): bus overload";
      break;

    case CAN_ERR_PROT_ACTIVE:
      errorMsg << " / error in CAN protocol (type): active error announcement";
      break;

    case CAN_ERR_PROT_TX:
      errorMsg << " / error in CAN protocol (type): error occurred on transmission";
      break;
  }

  // bit 3
  switch (frame.data[3]) {
    default:
    case CAN_ERR_PROT_LOC_UNSPEC:
      errorMsg << " / error in CAN protocol (location): unspecified";
      break;

    case CAN_ERR_PROT_LOC_SOF:
      errorMsg << " / error in CAN protocol (location): start of frame";
      break;

    case CAN_ERR_PROT_LOC_ID28_21:
      errorMsg << " / error in CAN protocol (location): ID bits 28 - 21 (SFF: 10 - 3)";
      break;

    case CAN_ERR_PROT_LOC_ID20_18:
      errorMsg << " / error in CAN protocol (location): ID bits 20 - 18 (SFF: 2 - 0 )";
      break;

    case CAN_ERR_PROT_LOC_SRTR:
      errorMsg << " / error in CAN protocol (location): substitute RTR (SFF: RTR)";
      break;

    case CAN_ERR_PROT_LOC_IDE:
      errorMsg << " / error in CAN protocol (location): identifier extension";
      break;

    case CAN_ERR_PROT_LOC_ID17_13:
      errorMsg << " / error in CAN protocol (location): ID bits 17-13";
      break;

    case CAN_ERR_PROT_LOC_ID12_05:
      errorMsg << " / error in CAN protocol (location): ID bits 12-5";
      break;

    case CAN_ERR_PROT_LOC_ID04_00:
      errorMsg << " / error in CAN protocol (location): ID bits 4-0";
      break;

    case CAN_ERR_PROT_LOC_RTR:
      errorMsg << " / error in CAN protocol (location): RTR";
      break;

    case CAN_ERR_PROT_LOC_RES1:
      errorMsg << " / error in CAN protocol (location): reserved bit 1";
      break;

    case CAN_ERR_PROT_LOC_RES0:
      errorMsg << " / error in CAN protocol (location): reserved bit 0";
      break;

    case CAN_ERR_PROT_LOC_DLC:
      errorMsg << " / error in CAN protocol (location): data length code";
      break;

    case CAN_ERR_PROT_LOC_DATA:
      errorMsg << " / error in CAN protocol (location): data section";
      break;

    case CAN_ERR_PROT_LOC_CRC_SEQ:
      errorMsg << " / error in CAN protocol (location): CRC sequence";
      break;

    case CAN_ERR_PROT_LOC_CRC_DEL:
      errorMsg << " / error in CAN protocol (location): CRC delimiter";
      break;

    case CAN_ERR_PROT_LOC_ACK:
      errorMsg << " / error in CAN protocol (location): ACK slot";
      break;

    case CAN_ERR_PROT_LOC_ACK_DEL:
      errorMsg << " / error in CAN protocol (location): ACK delimiter";
      break;

    case CAN_ERR_PROT_LOC_EOF:
      errorMsg << " / error in CAN protocol (location): end of frame";
      break;

    case CAN_ERR_PROT_LOC_INTERM:
      errorMsg << " / error in CAN protocol (location): intermission";
      break;
  }

  // bit 4
  switch (frame.data[4]) {
    default:
    case CAN_ERR_TRX_UNSPEC:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_UNSPEC";
      break;

    case CAN_ERR_TRX_CANH_NO_WIRE:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_NO_WIRE";
      break;

    case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_BAT";
      break;

    case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_VCC";
      break;

    case CAN_ERR_TRX_CANH_SHORT_TO_GND:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_GND";
      break;

    case CAN_ERR_TRX_CANL_NO_WIRE:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_NO_WIRE";
      break;

    case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_BAT";
      break;

    case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_VCC";
      break;

    case CAN_ERR_TRX_CANL_SHORT_TO_GND:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_GND";
      break;

    case CAN_ERR_TRX_CANL_SHORT_TO_CANH:
      errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_CANH";
      break;
  }

  // bit 5-7
  errorMsg << " / controller specific additional information: 0x" << std::hex << static_cast<int>(frame.data[5]) << " 0x" << std::hex
           << static_cast<int>(frame.data[6]) << " 0x" << std::hex << static_cast<int>(frame.data[7]);

  MELO_ERROR_THROTTLE_STREAM(1000, errorMsg.str());
}