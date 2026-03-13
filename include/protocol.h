#pragma once

#include <Arduino.h>

#include <cstring>

constexpr uint8_t kMagicByte0 = 0xAA;
constexpr uint8_t kMagicByte1 = 0x55;
constexpr uint8_t kMaxPayloadSize = 96;
constexpr size_t kMaxFrameSize = 2 + 1 + 1 + kMaxPayloadSize + 1;
constexpr size_t kMaxTextPayload = 80;

enum class PacketType : uint8_t {
  SensorData = 0x01,
  TextMessage = 0x02,
  Command = 0x03,
};

struct __attribute__((packed)) PayloadSensor {
  uint32_t uptime_ms;
  uint8_t tank_full;
  uint16_t hw390_raw[5];
  uint16_t ldr_raw[2];
  float ds18_c[3];
};

struct FrameMessage {
  PacketType type;
  uint8_t length;
  uint8_t payload[kMaxPayloadSize];
};

static_assert(sizeof(PayloadSensor) == 31, "Unexpected PayloadSensor size");

inline uint8_t frameChecksum(const uint8_t *data, size_t len) {
  uint8_t csum = 0;
  for (size_t i = 0; i < len; ++i) {
    csum ^= data[i];
  }
  return csum;
}

inline size_t encodeFrame(PacketType type, const uint8_t *payload, uint8_t payloadLen, uint8_t *outBuffer, size_t outCapacity) {
  const size_t frameLen = static_cast<size_t>(payloadLen) + 5;
  if (outBuffer == nullptr || outCapacity < frameLen || payloadLen > kMaxPayloadSize) {
    return 0;
  }

  outBuffer[0] = kMagicByte0;
  outBuffer[1] = kMagicByte1;
  outBuffer[2] = static_cast<uint8_t>(type);
  outBuffer[3] = payloadLen;
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(&outBuffer[4], payload, payloadLen);
  }
  outBuffer[4 + payloadLen] = frameChecksum(outBuffer, 4 + payloadLen);
  return frameLen;
}

class FrameParser {
 public:
  bool parseByte(uint8_t byte, FrameMessage &out) {
    switch (state_) {
      case State::WaitMagic0:
        if (byte == kMagicByte0) {
          state_ = State::WaitMagic1;
          scratch_[0] = byte;
        }
        return false;
      case State::WaitMagic1:
        if (byte == kMagicByte1) {
          state_ = State::WaitType;
          scratch_[1] = byte;
        } else {
          state_ = State::WaitMagic0;
        }
        return false;
      case State::WaitType:
        type_ = static_cast<PacketType>(byte);
        scratch_[2] = byte;
        state_ = State::WaitLen;
        return false;
      case State::WaitLen:
        len_ = byte;
        scratch_[3] = byte;
        index_ = 0;
        if (len_ > kMaxPayloadSize) {
          reset();
          return false;
        }
        state_ = (len_ == 0) ? State::WaitChecksum : State::WaitPayload;
        return false;
      case State::WaitPayload:
        payload_[index_] = byte;
        scratch_[4 + index_] = byte;
        ++index_;
        if (index_ >= len_) {
          state_ = State::WaitChecksum;
        }
        return false;
      case State::WaitChecksum: {
        const uint8_t expected = frameChecksum(scratch_, static_cast<size_t>(4 + len_));
        if (expected == byte) {
          out.type = type_;
          out.length = len_;
          if (len_ > 0) {
            memcpy(out.payload, payload_, len_);
          }
          reset();
          return true;
        }
        reset();
        return false;
      }
    }
    reset();
    return false;
  }

  void reset() {
    state_ = State::WaitMagic0;
    index_ = 0;
    len_ = 0;
  }

 private:
  enum class State : uint8_t { WaitMagic0, WaitMagic1, WaitType, WaitLen, WaitPayload, WaitChecksum };

  State state_ = State::WaitMagic0;
  PacketType type_ = PacketType::TextMessage;
  uint8_t len_ = 0;
  uint8_t index_ = 0;
  uint8_t payload_[kMaxPayloadSize] = {0};
  uint8_t scratch_[kMaxFrameSize] = {0};
};
