#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "protocol_contract.h"

namespace trashbot::protocol {

inline uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0u ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline uint16_t readU16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}
inline int16_t readI16(const uint8_t *data) { return static_cast<int16_t>(readU16(data)); }
inline uint32_t readU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}
inline void writeU16(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}
inline void writeI16(uint8_t *data, int16_t value) { writeU16(data, static_cast<uint16_t>(value)); }
inline void writeU32(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

struct FrameView { uint8_t type; uint16_t sequence; const uint8_t *payload; uint16_t payloadLength; };

class FrameWriter {
 public:
  explicit FrameWriter(HardwareSerial &serial) : serial_(serial) {}
  void send(uint8_t type, uint16_t sequence, const uint8_t *payload, uint16_t payloadLength) {
    if (payloadLength > kMaximumPayload) return;
    uint8_t frame[kMaximumFrame] = {};
    frame[0] = kMarker0; frame[1] = kMarker1; frame[2] = kVersion; frame[3] = type; frame[4] = 0;
    writeU16(frame + 5, sequence); writeU16(frame + 7, payloadLength);
    if (payloadLength != 0 && payload != nullptr) memcpy(frame + 9, payload, payloadLength);
    writeU16(frame + 9 + payloadLength, crc16(frame + 2, 7 + payloadLength));
    serial_.write(frame, kFrameOverhead + payloadLength);
  }
  void acknowledge(uint8_t requestType, uint16_t sequence, uint8_t status = 0) {
    const uint8_t payload[kAcknowledgmentPayloadBytes] = {requestType, status};
    send(kAcknowledgment, sequence, payload, sizeof(payload));
  }
  void error(uint8_t requestType, uint16_t sequence, uint8_t code, uint16_t detail = 0) {
    uint8_t payload[kErrorPayloadBytes] = {requestType, code, 0, 0};
    writeU16(payload + 2, detail); send(kError, sequence, payload, sizeof(payload));
  }
 private:
  HardwareSerial &serial_;
};

class FrameParser {
 public:
  template <typename Handler> void service(HardwareSerial &serial, uint32_t nowMs, Handler &&handler) {
    while (serial.available() > 0) feed(static_cast<uint8_t>(serial.read()), nowMs, handler);
  }
  uint32_t rejectedFrames() const { return rejectedFrames_; }
 private:
  template <typename Handler> void feed(uint8_t byte, uint32_t nowMs, Handler &handler) {
    if (length_ == 0) { if (byte == kMarker0) buffer_[length_++] = byte; return; }
    if (length_ == 1) { if (byte == kMarker1) buffer_[length_++] = byte; else reset(byte); return; }
    if (length_ >= sizeof(buffer_)) { ++rejectedFrames_; reset(byte); return; }
    buffer_[length_++] = byte;
    if (length_ < 9) return;
    const uint16_t payloadLength = readU16(buffer_ + 7);
    if (payloadLength > kMaximumPayload) { ++rejectedFrames_; reset(byte); return; }
    const size_t expectedLength = kFrameOverhead + payloadLength;
    if (length_ < expectedLength) return;
    if (length_ != expectedLength || buffer_[2] != kVersion) { ++rejectedFrames_; reset(byte); return; }
    const uint16_t expectedCrc = readU16(buffer_ + 9 + payloadLength);
    const uint16_t actualCrc = crc16(buffer_ + 2, 7 + payloadLength);
    if (expectedCrc != actualCrc) { ++rejectedFrames_; reset(byte); return; }
    const FrameView frame{buffer_[3], readU16(buffer_ + 5), buffer_ + 9, payloadLength};
    handler(frame, nowMs); reset();
  }
  void reset(uint8_t possibleMarker = 0) {
    length_ = 0; if (possibleMarker == kMarker0) { buffer_[0] = kMarker0; length_ = 1; }
  }
  uint8_t buffer_[kMaximumFrame] = {};
  size_t length_ = 0;
  uint32_t rejectedFrames_ = 0;
};

}  // namespace trashbot::protocol
