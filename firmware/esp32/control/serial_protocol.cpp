/* SPDX-License-Identifier: GPL-3.0-only */
#include "serial_protocol.h"

namespace gs::balance {
namespace {

constexpr uint8_t kMarker0 = 0xa5u;
constexpr uint8_t kMarker1 = 0x5au;
constexpr size_t kHeaderSize = 9u;
constexpr size_t kCrcSize = 2u;

void writeU16(uint8_t *output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8u);
}

void writeU32(uint8_t *output, uint32_t value) {
  writeU16(output, static_cast<uint16_t>(value));
  writeU16(output + 2u, static_cast<uint16_t>(value >> 16u));
}

uint16_t readU16(const uint8_t *input) {
  return static_cast<uint16_t>(input[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(input[1]) << 8u);
}

uint32_t readU32(const uint8_t *input) {
  return static_cast<uint32_t>(readU16(input)) |
         static_cast<uint32_t>(readU16(input + 2u)) << 16u;
}

} // namespace

size_t SerialProtocol::encode(const SerialFrame &frame, uint8_t *output,
                              size_t capacity) {
  if (output == nullptr || frame.payload_length > kSerialMaximumPayloadSize) {
    return 0u;
  }
  const size_t frame_length = kHeaderSize + frame.payload_length + kCrcSize;
  if (capacity < frame_length) {
    return 0u;
  }
  output[0] = kMarker0;
  output[1] = kMarker1;
  output[2] = frame.version;
  output[3] = static_cast<uint8_t>(frame.type);
  output[4] = frame.flags;
  writeU16(&output[5], frame.sequence);
  writeU16(&output[7], frame.payload_length);
  for (size_t index = 0u; index < frame.payload_length; ++index) {
    output[kHeaderSize + index] = frame.payload[index];
  }
  const uint16_t crc = crc16(&output[2], 7u + frame.payload_length);
  writeU16(&output[kHeaderSize + frame.payload_length], crc);
  return frame_length;
}

uint16_t SerialProtocol::crc16(const uint8_t *bytes, size_t length) {
  uint16_t crc = 0xffffu;
  for (size_t index = 0u; index < length; ++index) {
    crc ^= static_cast<uint16_t>(bytes[index]) << 8u;
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
      crc = (crc & 0x8000u) != 0u ? static_cast<uint16_t>((crc << 1u) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1u);
    }
  }
  return crc;
}

SerialParser::SerialParser(uint32_t timeout_us) : timeout_us_(timeout_us) {}

SerialParseResult SerialParser::feed(uint8_t byte, uint64_t now_us,
                                     SerialFrame &frame) {
  if (length_ != 0u && now_us - last_byte_us_ > timeout_us_) {
    reset();
    return SerialParseResult::kTimeout;
  }
  last_byte_us_ = now_us;
  if (length_ == 0u) {
    if (byte == kMarker0) {
      bytes_[length_++] = byte;
    }
    return SerialParseResult::kIncomplete;
  }
  if (length_ == 1u) {
    if (byte != kMarker1) {
      length_ = byte == kMarker0 ? 1u : 0u;
      return SerialParseResult::kIncomplete;
    }
    bytes_[length_++] = byte;
    return SerialParseResult::kIncomplete;
  }
  bytes_[length_++] = byte;
  if (length_ == kHeaderSize) {
    const uint16_t payload_length = readU16(&bytes_[7]);
    if (payload_length > kSerialMaximumPayloadSize) {
      reset();
      return SerialParseResult::kMalformed;
    }
    expected_length_ = kHeaderSize + payload_length + kCrcSize;
  }
  if (expected_length_ != 0u && length_ == expected_length_) {
    return consumeComplete(frame);
  }
  return SerialParseResult::kIncomplete;
}

void SerialParser::reset() {
  length_ = 0u;
  expected_length_ = 0u;
  last_byte_us_ = 0u;
}

SerialParseResult SerialParser::consumeComplete(SerialFrame &frame) {
  const uint16_t payload_length = readU16(&bytes_[7]);
  const uint16_t expected_crc =
      SerialProtocol::crc16(&bytes_[2], 7u + payload_length);
  const uint16_t actual_crc = readU16(&bytes_[kHeaderSize + payload_length]);
  if (expected_crc != actual_crc) {
    reset();
    return SerialParseResult::kInvalidCrc;
  }
  if (bytes_[2] != kSerialProtocolVersion) {
    reset();
    return SerialParseResult::kMalformed;
  }
  frame.version = bytes_[2];
  frame.type = static_cast<SerialMessageType>(bytes_[3]);
  frame.flags = bytes_[4];
  frame.sequence = readU16(&bytes_[5]);
  frame.payload_length = payload_length;
  for (size_t index = 0u; index < payload_length; ++index) {
    frame.payload[index] = bytes_[kHeaderSize + index];
  }
  reset();
  return SerialParseResult::kFrame;
}

bool SerialSequence::accept(uint16_t sequence) {
  if (!initialized_) {
    last_ = sequence;
    initialized_ = true;
    return true;
  }
  const int16_t difference = static_cast<int16_t>(sequence - last_);
  if (difference <= 0) {
    return false;
  }
  last_ = sequence;
  return true;
}

void SerialSequence::reset() {
  last_ = 0u;
  initialized_ = false;
}

bool MovementCommand::expired(uint64_t received_us, uint64_t now_us) const {
  return now_us > received_us + static_cast<uint64_t>(lifetime_ms) * 1000u;
}

size_t MovementCommandCodec::encode(const MovementCommand &command,
                                    uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 10u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeU16(&payload[0], static_cast<uint16_t>(command.linear_velocity_milli));
  writeU16(&payload[2], static_cast<uint16_t>(command.yaw_rate_milli));
  writeU32(&payload[4], command.lease_id);
  writeU16(&payload[8], command.lifetime_ms);
  return kLength;
}

bool MovementCommandCodec::decode(const uint8_t *payload, size_t length,
                                  MovementCommand &command) {
  if (payload == nullptr || length != 10u) {
    return false;
  }
  command.linear_velocity_milli = static_cast<int16_t>(readU16(&payload[0]));
  command.yaw_rate_milli = static_cast<int16_t>(readU16(&payload[2]));
  command.lease_id = readU32(&payload[4]);
  command.lifetime_ms = readU16(&payload[8]);
  return command.lifetime_ms != 0u;
}

size_t DirectMotorCommandCodec::encode(const DirectMotorCommand &command,
                                       uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 10u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeU16(&payload[0], static_cast<uint16_t>(command.left));
  writeU16(&payload[2], static_cast<uint16_t>(command.right));
  writeU32(&payload[4], command.lease_id);
  writeU16(&payload[8], command.lifetime_ms);
  return kLength;
}

bool DirectMotorCommandCodec::decode(const uint8_t *payload, size_t length,
                                     DirectMotorCommand &command) {
  if (payload == nullptr || length != 10u) {
    return false;
  }
  command.left = static_cast<int16_t>(readU16(&payload[0]));
  command.right = static_cast<int16_t>(readU16(&payload[2]));
  command.lease_id = readU32(&payload[4]);
  command.lifetime_ms = readU16(&payload[8]);
  return command.lifetime_ms != 0u;
}

} // namespace gs::balance
