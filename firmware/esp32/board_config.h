#pragma once

#include <cstdint>

namespace trashbot::board {

constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kLeftThrottlePin = 25;
constexpr uint8_t kRightThrottlePin = 26;
constexpr uint8_t kLeftReversePin = 27;
constexpr uint8_t kRightReversePin = 14;
constexpr uint8_t kLeftBrakePin = 33;
constexpr uint8_t kRightBrakePin = 32;

// Right hoverboard motor Hall sensor inputs. These are normal GPIOs with
// internal pull-ups, so an open-collector Hall output can be wired directly
// when the Hall sensors are powered from the ESP32 3.3 V rail.
constexpr uint8_t kRightHallAPin = 19;
constexpr uint8_t kRightHallBPin = 21;
constexpr uint8_t kRightHallCPin = 22;

constexpr uint8_t kFrontTrigPin = 16;
constexpr uint8_t kFrontEchoPin = 34;
constexpr uint8_t kLeftTrigPin = 17;
constexpr uint8_t kLeftEchoPin = 35;
constexpr uint8_t kRightTrigPin = 18;
constexpr uint8_t kRightEchoPin = 39;

constexpr float kMaxLinearMilli = 350.0f;
constexpr float kMaxYawMilli = 800.0f;

constexpr float kDacReferenceVolts = 3.3f;
constexpr float kThrottleIdleVolts = 0.85f;
constexpr float kThrottleStartVolts = 1.15f;
constexpr float kThrottleMaxVolts = 3.15f;
constexpr float kCommandDeadband = 0.04f;
constexpr float kThrottleSlewPerSecond = 1.5f;
constexpr uint32_t kDirectionBrakeBeforeMs = 180;
constexpr uint32_t kDirectionBrakeAfterMs = 180;

constexpr uint32_t kHallRateWindowMs = 100;
constexpr uint32_t kHallMovingWindowMs = 200;
constexpr uint32_t kHallTelemetryMs = 50;

constexpr uint32_t kUltrasonicPingSpacingMs = 40;
constexpr uint32_t kUltrasonicTimeoutUs = 15000;
constexpr uint16_t kUltrasonicMinMm = 25;
constexpr uint16_t kUltrasonicMaxMm = 2500;
constexpr uint32_t kUltrasonicStaleMs = 500;
constexpr uint32_t kUltrasonicTelemetryMs = 100;

}  // namespace trashbot::board
