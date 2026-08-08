#pragma once

#include <Arduino.h>
#include <cstdint>

#include "board_config.h"
#include "serial_protocol.h"

namespace trashbot::sensors {

struct UltrasonicReading {
  uint16_t frontMm;
  uint16_t leftMm;
  uint16_t rightMm;
  uint8_t validMask;
};

class UltrasonicArray {
 public:
  UltrasonicArray()
      : sensors_{{board::kFrontTrigPin, board::kFrontEchoPin},
                 {board::kLeftTrigPin, board::kLeftEchoPin},
                 {board::kRightTrigPin, board::kRightEchoPin}} {}

  void begin() {
    for (auto &sensor : sensors_) {
      pinMode(sensor.triggerPin, OUTPUT);
      pinMode(sensor.echoPin, INPUT);
      digitalWrite(sensor.triggerPin, LOW);
    }
  }

  void service(uint32_t nowMs) {
    if (nowMs - lastPingMs_ < board::kUltrasonicPingSpacingMs) {
      return;
    }
    lastPingMs_ = nowMs;

    Sensor &sensor = sensors_[nextSensor_];
    nextSensor_ = static_cast<uint8_t>((nextSensor_ + 1) % 3);

    digitalWrite(sensor.triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.triggerPin, LOW);

    const uint32_t durationUs =
        pulseIn(sensor.echoPin, HIGH, board::kUltrasonicTimeoutUs);
    if (durationUs == 0) {
      sensor.distanceMm = kInvalidDistance;
    } else {
      const uint32_t distanceMm = (durationUs * 343u) / 2000u;
      sensor.distanceMm = distanceMm >= board::kUltrasonicMinMm &&
                                  distanceMm <= board::kUltrasonicMaxMm
                              ? static_cast<uint16_t>(distanceMm)
                              : kInvalidDistance;
    }
    sensor.measuredAtMs = nowMs;
  }

  UltrasonicReading reading(uint32_t nowMs) const {
    UltrasonicReading result{kInvalidDistance, kInvalidDistance,
                             kInvalidDistance, 0};
    uint16_t *destinations[3] = {&result.frontMm, &result.leftMm,
                                 &result.rightMm};
    for (uint8_t index = 0; index < 3; ++index) {
      const uint16_t distance = freshDistance(sensors_[index], nowMs);
      *destinations[index] = distance;
      if (distance != kInvalidDistance) {
        result.validMask |= static_cast<uint8_t>(1u << index);
      }
    }
    return result;
  }

  void encode(uint8_t *payload, uint32_t nowMs) const {
    const UltrasonicReading current = reading(nowMs);
    protocol::writeU16(payload, current.frontMm);
    protocol::writeU16(payload + 2, current.leftMm);
    protocol::writeU16(payload + 4, current.rightMm);
    payload[6] = current.validMask;
    payload[7] = 0;
  }

 private:
  static constexpr uint16_t kInvalidDistance = 0xFFFFu;

  struct Sensor {
    uint8_t triggerPin;
    uint8_t echoPin;
    uint16_t distanceMm = kInvalidDistance;
    uint32_t measuredAtMs = 0;
  };

  static uint16_t freshDistance(const Sensor &sensor, uint32_t nowMs) {
    if (sensor.measuredAtMs == 0 ||
        nowMs - sensor.measuredAtMs > board::kUltrasonicStaleMs) {
      return kInvalidDistance;
    }
    return sensor.distanceMm;
  }

  Sensor sensors_[3];
  uint8_t nextSensor_ = 0;
  uint32_t lastPingMs_ = 0;
};

}  // namespace trashbot::sensors
