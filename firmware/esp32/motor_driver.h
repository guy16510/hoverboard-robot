#pragma once

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "board_config.h"

#ifndef WINXU_LEFT_MOTOR_SIGN
#define WINXU_LEFT_MOTOR_SIGN 1
#endif
#ifndef WINXU_RIGHT_MOTOR_SIGN
#define WINXU_RIGHT_MOTOR_SIGN -1
#endif
#ifndef WINXU_SWITCH_ACTIVE_HIGH
#define WINXU_SWITCH_ACTIVE_HIGH 1
#endif
#ifndef WINXU_RUNTIME_REVERSE
#define WINXU_RUNTIME_REVERSE 1
#endif

static_assert(WINXU_LEFT_MOTOR_SIGN == 1 || WINXU_LEFT_MOTOR_SIGN == -1,
              "WINXU_LEFT_MOTOR_SIGN must be +1 or -1");
static_assert(WINXU_RIGHT_MOTOR_SIGN == 1 || WINXU_RIGHT_MOTOR_SIGN == -1,
              "WINXU_RIGHT_MOTOR_SIGN must be +1 or -1");

namespace trashbot::drive {

inline uint8_t switchLevel(bool active) {
#if WINXU_SWITCH_ACTIVE_HIGH
  return active ? HIGH : LOW;
#else
  return active ? LOW : HIGH;
#endif
}

inline uint8_t voltsToDac(float volts) {
  const float bounded =
      std::max(0.0f, std::min(board::kDacReferenceVolts, volts));
  return static_cast<uint8_t>(
      lroundf((bounded / board::kDacReferenceVolts) * 255.0f));
}

class MotorChannel {
 public:
  MotorChannel(uint8_t throttlePin, uint8_t reversePin, uint8_t brakePin)
      : throttlePin_(throttlePin),
        reversePin_(reversePin),
        brakePin_(brakePin) {}

  void begin() {
    pinMode(reversePin_, OUTPUT);
    pinMode(brakePin_, OUTPUT);
    writeReverse(false);
    writeThrottle(0.0f);
    writeBrake(true);
  }

  void setTarget(float target) {
    target_ = std::max(-1.0f, std::min(1.0f, target));
  }

  void hardStop() {
    target_ = 0.0f;
    appliedMagnitude_ = 0.0f;
    phase_ = Phase::kRun;
    writeThrottle(0.0f);
    writeBrake(true);
  }

  void service(uint32_t nowMs) {
    const uint32_t elapsedMs =
        lastServiceMs_ == 0 ? 0 : nowMs - lastServiceMs_;
    lastServiceMs_ = nowMs;
    const float maximumDelta = board::kThrottleSlewPerSecond *
                               (static_cast<float>(elapsedMs) / 1000.0f);

    const bool wantsMotion = std::fabs(target_) >= board::kCommandDeadband;
    const bool desiredReverse = target_ < 0.0f;
    const float desiredMagnitude = wantsMotion ? std::fabs(target_) : 0.0f;

#if !WINXU_RUNTIME_REVERSE
    if (desiredReverse) {
      hardStop();
      return;
    }
#endif

    switch (phase_) {
      case Phase::kRun:
        serviceRun(nowMs, wantsMotion, desiredReverse, desiredMagnitude,
                   maximumDelta);
        return;
      case Phase::kBrakeBeforeDirection:
        writeThrottle(0.0f);
        writeBrake(true);
        if (nowMs - phaseStartedMs_ >= board::kDirectionBrakeBeforeMs) {
          reverse_ = pendingReverse_;
          writeReverse(reverse_);
          phase_ = Phase::kBrakeAfterDirection;
          phaseStartedMs_ = nowMs;
        }
        return;
      case Phase::kBrakeAfterDirection:
        writeThrottle(0.0f);
        writeBrake(true);
        if (nowMs - phaseStartedMs_ >= board::kDirectionBrakeAfterMs) {
          phase_ = Phase::kRun;
        }
        return;
    }
  }

 private:
  enum class Phase : uint8_t {
    kRun,
    kBrakeBeforeDirection,
    kBrakeAfterDirection,
  };

  static float moveToward(float current, float target, float maximumDelta) {
    if (target > current) {
      return std::min(target, current + maximumDelta);
    }
    return std::max(target, current - maximumDelta);
  }

  void serviceRun(uint32_t nowMs, bool wantsMotion, bool desiredReverse,
                  float desiredMagnitude, float maximumDelta) {
    if (!wantsMotion) {
      appliedMagnitude_ = moveToward(appliedMagnitude_, 0.0f, maximumDelta);
      writeThrottle(appliedMagnitude_);
      if (appliedMagnitude_ <= 0.001f) {
        appliedMagnitude_ = 0.0f;
        writeThrottle(0.0f);
        writeBrake(true);
      }
      return;
    }

    if (desiredReverse != reverse_) {
      appliedMagnitude_ = moveToward(appliedMagnitude_, 0.0f, maximumDelta);
      writeThrottle(appliedMagnitude_);
      if (appliedMagnitude_ <= 0.001f) {
        appliedMagnitude_ = 0.0f;
        pendingReverse_ = desiredReverse;
        writeThrottle(0.0f);
        writeBrake(true);
        phase_ = Phase::kBrakeBeforeDirection;
        phaseStartedMs_ = nowMs;
      }
      return;
    }

    writeBrake(false);
    appliedMagnitude_ =
        moveToward(appliedMagnitude_, desiredMagnitude, maximumDelta);
    writeThrottle(appliedMagnitude_);
  }

  void writeBrake(bool active) {
    digitalWrite(brakePin_, switchLevel(active));
  }

  void writeReverse(bool active) {
    digitalWrite(reversePin_, switchLevel(active));
  }

  void writeThrottle(float magnitude) {
    magnitude = std::max(0.0f, std::min(1.0f, magnitude));
    float volts = board::kThrottleIdleVolts;
    if (magnitude >= board::kCommandDeadband) {
      volts = board::kThrottleStartVolts +
              magnitude *
                  (board::kThrottleMaxVolts - board::kThrottleStartVolts);
    }
    dacWrite(throttlePin_, voltsToDac(volts));
  }

  uint8_t throttlePin_;
  uint8_t reversePin_;
  uint8_t brakePin_;
  float target_ = 0.0f;
  float appliedMagnitude_ = 0.0f;
  bool reverse_ = false;
  bool pendingReverse_ = false;
  Phase phase_ = Phase::kRun;
  uint32_t phaseStartedMs_ = 0;
  uint32_t lastServiceMs_ = 0;
};

class DifferentialDrive {
 public:
  DifferentialDrive()
      : left_(board::kLeftThrottlePin, board::kLeftReversePin,
              board::kLeftBrakePin),
        right_(board::kRightThrottlePin, board::kRightReversePin,
               board::kRightBrakePin) {}

  void begin() {
    left_.begin();
    right_.begin();
    stop();
  }

  void stop() {
    left_.hardStop();
    right_.hardStop();
  }

  void setNormalized(float linear, float yaw) {
    linear = std::max(-1.0f, std::min(1.0f, linear));
    yaw = std::max(-1.0f, std::min(1.0f, yaw));

    float left =
        (linear + yaw) * static_cast<float>(WINXU_LEFT_MOTOR_SIGN);
    float right =
        (linear - yaw) * static_cast<float>(WINXU_RIGHT_MOTOR_SIGN);
    const float peak = std::max(std::fabs(left), std::fabs(right));
    if (peak > 1.0f) {
      left /= peak;
      right /= peak;
    }
    left_.setTarget(left);
    right_.setTarget(right);
  }

  void service(uint32_t nowMs) {
    left_.service(nowMs);
    right_.service(nowMs);
  }

 private:
  MotorChannel left_;
  MotorChannel right_;
};

}  // namespace trashbot::drive
