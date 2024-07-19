#pragma once

#include <stdint.h>
#include <algorithm>  // std::copy
#include <cassert>
#include <initializer_list>

namespace tcan_can_device {

//! General CANOpen message container

class CanMsg {
 public:
  static constexpr size_t Capacity = 8;

  /*! Constructor
   * @param  COBId  Communication Object Identifier
   */
  CanMsg() = delete;

  CanMsg(const uint32_t CobId) : CobId_(CobId), length_{0}, data_{0, 0, 0, 0, 0, 0, 0, 0} {}

  CanMsg(const uint32_t CobId, const uint8_t length) : CobId_(CobId), length_(length), data_{0, 0, 0, 0, 0, 0, 0, 0} {
    assert(length <= Capacity);
  }

  CanMsg(const uint32_t CobId, const uint8_t length, const uint8_t* data) : CobId_(CobId), length_(length), data_{0, 0, 0, 0, 0, 0, 0, 0} {
    assert(length <= Capacity);
    std::copy(&data[0], &data[length], data_);
  }

  CanMsg(const uint32_t CobId, const uint8_t length, const std::initializer_list<uint8_t> data)
      : CobId_(CobId), length_(length), data_{0, 0, 0, 0, 0, 0, 0, 0} {
    assert(length <= Capacity);
    assert(length == data.size());
    std::copy(data.begin(), data.end(), data_);
  }

  CanMsg(const uint32_t CobId, const std::initializer_list<uint8_t> data)
      : CobId_(CobId), length_(data.size()), data_{0, 0, 0, 0, 0, 0, 0, 0} {
    assert(data.size() <= Capacity);
    std::copy(data.begin(), data.end(), data_);
  }

  //! Destructor
  virtual ~CanMsg() = default;

  /*! Gets the Communication Object Identifier
   *
   * @return COBId
   */
  constexpr uint32_t getCobId() const { return CobId_; }

  /*! Gets the stack of values
   *
   * @return reference to data_[8]
   */
  inline const uint8_t* getData() const { return data_; }

  /*! Gets the lengths of the values in the stack
   * @return reference to length
   */
  constexpr uint8_t getLength() const { return length_; }

  /*!
   * Set length of the message. Note that the data is not set (but was initialized to 0)
   * @param length    length of the message in bytes [0,8]
   */
  inline void setLength(const uint8_t length) { length_ = length; }

  /*! Sets the stack of values
   * @param value   array of length 8
   */
  inline void setData(const uint8_t length, const uint8_t* data) {
    assert(length <= Capacity);
    length_ = length;
    std::copy(&data[0], &data[length], data_);
  }

  inline void write(const int32_t value) {
    assert(length_ + 4u <= Capacity);
    data_[3 + length_] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data_[2 + length_] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data_[1 + length_] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + length_] = static_cast<uint8_t>((value >> 0) & 0xFF);

    length_ += 4;
  }

  inline void write(const uint32_t value) {
    assert(length_ + 4u <= Capacity);
    data_[3 + length_] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data_[2 + length_] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data_[1 + length_] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + length_] = static_cast<uint8_t>((value >> 0) & 0xFF);

    length_ += 4;
  }

  inline void write(const int16_t value) {
    assert(length_ + 2u <= Capacity);
    data_[1 + length_] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + length_] = static_cast<uint8_t>((value >> 0) & 0xFF);

    length_ += 2;
  }

  inline void write(const uint16_t value) {
    assert(length_ + 2u <= Capacity);
    data_[1 + length_] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + length_] = static_cast<uint8_t>((value >> 0) & 0xFF);

    length_ += 2;
  }

  inline void write(const int8_t value) {
    assert(length_ + 1u <= Capacity);
    data_[0 + length_] = static_cast<uint8_t>(value);

    length_ += 1;
  }

  inline void write(const uint8_t value) {
    assert(length_ + 1u <= Capacity);
    data_[0 + length_] = value;

    length_ += 1;
  }

  inline void write(const int32_t value, const uint8_t pos) {
    assert(pos + 4u <= Capacity);
    data_[3 + pos] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data_[2 + pos] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

    if (pos + 4u > length_) {
      length_ = pos + 4;
    }
  }

  inline void write(const uint32_t value, const uint8_t pos) {
    assert(pos + 4u <= Capacity);
    data_[3 + pos] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data_[2 + pos] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

    if (pos + 4u > length_) {
      length_ = pos + 4;
    }
  }

  inline void write(const int16_t value, const uint8_t pos) {
    assert(pos + 2u <= Capacity);
    data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

    if (pos + 2u > length_) {
      length_ = pos + 2;
    }
  }

  inline void write(const uint16_t value, const uint8_t pos) {
    assert(pos + 2u <= Capacity);
    data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

    if (pos + 2u > length_) {
      length_ = pos + 2;
    }
  }

  inline void write(const int8_t value, const uint8_t pos) {
    assert(pos + 1u <= Capacity);
    data_[0 + pos] = static_cast<uint8_t>(value);

    if (pos + 1u > length_) {
      length_ = pos + 1;
    }
  }

  inline void write(const uint8_t value, const uint8_t pos) {
    assert(pos + 1u <= Capacity);
    data_[0 + pos] = value;

    if (pos + 1u > length_) {
      length_ = pos + 1;
    }
  }

  inline int32_t readint32(uint8_t pos) const {
    assert(pos + 4u <= length_);
    return (static_cast<int32_t>(data_[3 + pos]) << 24) | (static_cast<int32_t>(data_[2 + pos]) << 16) |
           (static_cast<int32_t>(data_[1 + pos]) << 8) | (static_cast<int32_t>(data_[0 + pos]));
  }

  inline uint32_t readuint32(uint8_t pos) const {
    assert(pos + 4u <= length_);
    return (static_cast<uint32_t>(data_[3 + pos]) << 24) | (static_cast<uint32_t>(data_[2 + pos]) << 16) |
           (static_cast<uint32_t>(data_[1 + pos]) << 8) | (static_cast<uint32_t>(data_[0 + pos]));
  }

  uint32_t readuint24(uint8_t pos) const {
    assert(pos + 3u <= length_);
    return (static_cast<uint32_t>(data_[2 + pos]) << 16) | (static_cast<uint32_t>(data_[1 + pos]) << 8) |
           (static_cast<uint32_t>(data_[0 + pos]));
  }

  inline int16_t readint16(uint8_t pos) const {
    assert(pos + 2u <= length_);
    return (static_cast<int16_t>(data_[1 + pos]) << 8) | (static_cast<int16_t>(data_[0 + pos]));
  }

  inline uint16_t readuint16(uint8_t pos) const {
    assert(pos + 2u <= length_);
    return (static_cast<uint16_t>(data_[1 + pos]) << 8) | (static_cast<uint16_t>(data_[0 + pos]));
  }

  inline int8_t readint8(uint8_t pos) const {
    assert(pos + 1u <= length_);
    return static_cast<int8_t>(data_[0 + pos]);
  }

  inline uint8_t readuint8(uint8_t pos) const {
    assert(pos + 1u <= length_);
    return data_[0 + pos];
  }

 private:
  //! Communication Object Identifier
  uint32_t CobId_;

  //! the message data length
  uint8_t length_;

  /*! Data of the CAN message
   */
  uint8_t data_[Capacity];
};

} /* namespace tcan_can_device */
