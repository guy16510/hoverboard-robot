#pragma once

#include <cstddef>
#include <cstdint>

namespace trashbot::board {

constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kLeftThrottlePin = 25;
constexpr uint8_t kRightThrottlePin = 26;
constexpr uint8_t kLeftReversePin = 27;
constexpr uint8_t kRightReversePin = 14;
constexpr uint8_t kLeftBrakePin = 33;
constexpr uint8_t kRightBrakePin = 32;

constexpr uint8_t kRightHallAPin = 19;
constexpr uint8_t kRightHallBPin = 21;
constexpr uint8_t kRightHallCPin = 22;
constexpr uint8_t kLeftHallAPin = 16;
constexpr uint8_t kLeftHallBPin = 17;
constexpr uint8_t kLeftHallCPin = 36;
constexpr bool kLeftHallEnabled = false;

constexpr uint8_t kFrontTrigPin = 5;
constexpr uint8_t kFrontEchoPin = 34;
constexpr uint8_t kLeftTrigPin = 15;
constexpr uint8_t kLeftEchoPin = 35;
constexpr uint8_t kRightTrigPin = 18;
constexpr uint8_t kRightEchoPin = 39;

constexpr uint8_t kMpu6050SdaPin = 4;
constexpr uint8_t kMpu6050SclPin = 23;
constexpr uint8_t kArmServoPin = 13;
constexpr bool kRgbEnabled = false;

constexpr uint32_t kImuSampleMs = 20;
constexpr uint32_t kImuTelemetryMs = 50;

template <std::size_t N>
constexpr bool pinsAreUnique(const uint8_t (&pins)[N]) {
  for (std::size_t left = 0; left < N; ++left) {
    for (std::size_t right = left + 1; right < N; ++right) {
      if (pins[left] == pins[right]) {
        return false;
      }
    }
  }
  return true;
}

constexpr uint8_t kAssignedPins[] = {
    kLeftThrottlePin, kRightThrottlePin, kLeftReversePin, kRightReversePin,
    kLeftBrakePin,    kRightBrakePin,    kRightHallAPin,  kRightHallBPin,
    kRightHallCPin,   kLeftHallAPin,     kLeftHallBPin,   kLeftHallCPin,
    kFrontTrigPin,    kFrontEchoPin,     kLeftTrigPin,    kLeftEchoPin,
    kRightTrigPin,    kRightEchoPin,     kMpu6050SdaPin,  kMpu6050SclPin,
    kArmServoPin,
};

static_assert(pinsAreUnique(kAssignedPins),
              "ESP32 board pin contract contains a duplicate GPIO");
static_assert(kLeftThrottlePin == 25 && kRightThrottlePin == 26,
              "ESP32 DAC throttle outputs must remain on GPIO25/GPIO26");
static_assert(kFrontEchoPin >= 34 && kLeftEchoPin >= 34 &&
                  kRightEchoPin >= 34,
              "ultrasonic echo pins are expected on input-only GPIOs");

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
