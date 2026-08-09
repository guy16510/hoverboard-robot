#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

#include "board_config.h"

namespace trashbot::sensors {

struct ImuReading {
  int16_t accelXMilliG = 0;
  int16_t accelYMilliG = 0;
  int16_t accelZMilliG = 0;
  int16_t gyroXMilliDps = 0;
  int16_t gyroYMilliDps = 0;
  int16_t gyroZMilliDps = 0;
  int16_t temperatureCentiC = 0;
  bool valid = false;
};

class Mpu6050Sensor {
 public:
  void begin() {
    Wire.begin(board::kMpu6050SdaPin, board::kMpu6050SclPin);
    Wire.beginTransmission(kAddress);
    Wire.write(0x6B);
    Wire.write(0x00);
    _ready = Wire.endTransmission() == 0;
  }

  void service(uint32_t nowMs) {
    if (!_ready || nowMs - _lastReadMs < board::kImuSampleMs) {
      return;
    }
    _lastReadMs = nowMs;

    Wire.beginTransmission(kAddress);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
      _reading.valid = false;
      return;
    }
    const uint8_t expected = 14;
    if (Wire.requestFrom(kAddress, expected, true) != expected) {
      _reading.valid = false;
      return;
    }

    const int16_t ax = readI16();
    const int16_t ay = readI16();
    const int16_t az = readI16();
    const int16_t temp = readI16();
    const int16_t gx = readI16();
    const int16_t gy = readI16();
    const int16_t gz = readI16();

    _reading.accelXMilliG = static_cast<int16_t>((static_cast<int32_t>(ax) * 1000) / 16384);
    _reading.accelYMilliG = static_cast<int16_t>((static_cast<int32_t>(ay) * 1000) / 16384);
    _reading.accelZMilliG = static_cast<int16_t>((static_cast<int32_t>(az) * 1000) / 16384);
    _reading.gyroXMilliDps = static_cast<int16_t>((static_cast<int32_t>(gx) * 1000) / 131);
    _reading.gyroYMilliDps = static_cast<int16_t>((static_cast<int32_t>(gy) * 1000) / 131);
    _reading.gyroZMilliDps = static_cast<int16_t>((static_cast<int32_t>(gz) * 1000) / 131);
    _reading.temperatureCentiC = static_cast<int16_t>((static_cast<int32_t>(temp) * 100) / 340 + 3653);
    _reading.valid = true;
  }

  const ImuReading &reading() const { return _reading; }

 private:
  static constexpr uint8_t kAddress = 0x68;

  static int16_t readI16() {
    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  }

  bool _ready = false;
  uint32_t _lastReadMs = 0;
  ImuReading _reading{};
};

}  // namespace trashbot::sensors
