#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include <cstdint>

#include "board_config.h"

namespace trashbot::actuators {

class ServoActuator {
 public:
  void begin() {
    _servo.setPeriodHertz(50);
    _servo.attach(board::kArmServoPin, 500, 2500);
    setNormalized(0.0f);
  }

  void setNormalized(float value) {
    value = constrain(value, -1.0f, 1.0f);
    _normalized = value;
    const int degrees = static_cast<int>((value + 1.0f) * 90.0f);
    _servo.write(degrees);
  }

  float normalized() const { return _normalized; }

 private:
  Servo _servo;
  float _normalized = 0.0f;
};

}  // namespace trashbot::actuators
