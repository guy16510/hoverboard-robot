#pragma once

#include <cstddef>
#include <cstdint>

namespace trashbot::protocol {

constexpr uint8_t kMarker0 = 0xA5;
constexpr uint8_t kMarker1 = 0x5A;
constexpr uint8_t kVersion = 1;
constexpr uint16_t kMaximumPayload = 48;
constexpr size_t kFrameOverhead = 11;
constexpr size_t kMaximumFrame = kFrameOverhead + kMaximumPayload;

constexpr uint8_t kDriveMode = 2;
constexpr uint16_t kMotionPayloadBytes = 10;
constexpr uint16_t kCapabilitiesPayloadBytes = 12;
constexpr uint16_t kStatusPayloadBytes = 16;
constexpr uint16_t kUltrasonicPayloadBytes = 8;
constexpr uint16_t kAcknowledgmentPayloadBytes = 2;
constexpr uint16_t kErrorPayloadBytes = 4;

enum MessageType : uint8_t {
  kHello = 0x01,
  kCapabilities = 0x02,
  kArm = 0x10,
  kDisarm = 0x11,
  kStop = 0x12,
  kEmergencyStop = 0x13,
  kClearFault = 0x14,
  kSetOperatingMode = 0x15,
  kSetVelocityYaw = 0x22,
  kHeartbeat = 0x23,
  kStatus = 0x30,
  kUltrasonic = 0x35,
  kAcknowledgment = 0x7E,
  kError = 0x7F,
};

}  // namespace trashbot::protocol
