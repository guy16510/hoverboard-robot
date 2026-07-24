/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace gs::balance {

constexpr uint8_t kSerialProtocolVersion = 1u;
constexpr size_t kSerialMaximumPayloadSize = 48u;
constexpr size_t kSerialMaximumFrameSize = 11u + kSerialMaximumPayloadSize;

enum class SerialMessageType : uint8_t {
  kHello = 0x01,
  kCapabilities = 0x02,
  kArm = 0x10,
  kDisarm = 0x11,
  kStop = 0x12,
  kEmergencyStop = 0x13,
  kClearFault = 0x14,
  kSetOperatingMode = 0x15,
  kSetLinearVelocity = 0x20,
  kSetYawRate = 0x21,
  kSetVelocityAndYaw = 0x22,
  kHeartbeat = 0x23,
  kStatus = 0x30,
  kImuTelemetry = 0x31,
  kMotorTelemetry = 0x32,
  kOdometry = 0x33,
  kActiveFaults = 0x34,
  kConfigurationRead = 0x40,
  kConfigurationUpdate = 0x41,
  kAcknowledgment = 0x7e,
  kErrorResponse = 0x7f,
};

struct SerialFrame {
  uint8_t version = kSerialProtocolVersion;
  SerialMessageType type = SerialMessageType::kHello;
  uint8_t flags = 0u;
  uint16_t sequence = 0u;
  uint16_t payload_length = 0u;
  std::array<uint8_t, kSerialMaximumPayloadSize> payload{};
};

enum class SerialParseResult : uint8_t {
  kIncomplete,
  kFrame,
  kInvalidCrc,
  kMalformed,
  kTimeout,
};

enum class SerialErrorCode : uint8_t {
  kMalformed = 1u,
  kInvalidCrc = 2u,
  kStaleSequence = 3u,
  kUnsupported = 4u,
  kLeaseConflict = 5u,
  kUnsafeState = 6u,
  kInvalidConfiguration = 7u,
  kResponseQueueFull = 8u,
};

class SerialProtocol {
public:
  static size_t encode(const SerialFrame &frame, uint8_t *output,
                       size_t capacity);
  static uint16_t crc16(const uint8_t *bytes, size_t length);
};

class SerialParser {
public:
  explicit SerialParser(uint32_t timeout_us);

  SerialParseResult feed(uint8_t byte, uint64_t now_us, SerialFrame &frame);
  void reset();

private:
  SerialParseResult consumeComplete(SerialFrame &frame);

  uint32_t timeout_us_;
  std::array<uint8_t, kSerialMaximumFrameSize> bytes_{};
  size_t length_ = 0u;
  size_t expected_length_ = 0u;
  uint64_t last_byte_us_ = 0u;
};

class SerialSequence {
public:
  bool accept(uint16_t sequence);
  void reset();

private:
  uint16_t last_ = 0u;
  bool initialized_ = false;
};

struct MovementCommand {
  int16_t linear_velocity_milli = 0;
  int16_t yaw_rate_milli = 0;
  uint32_t lease_id = 0u;
  uint16_t lifetime_ms = 0u;

  bool expired(uint64_t received_us, uint64_t now_us) const;
};

class MovementCommandCodec {
public:
  static size_t encode(const MovementCommand &command, uint8_t *payload,
                       size_t capacity);
  static bool decode(const uint8_t *payload, size_t length,
                     MovementCommand &command);
};

} // namespace gs::balance
