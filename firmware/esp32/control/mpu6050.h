/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "interfaces.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gs::balance {

struct Mpu6050Registers {
  static constexpr uint8_t kSampleRateDivider = 0x19;
  static constexpr uint8_t kConfiguration = 0x1a;
  static constexpr uint8_t kGyroscopeConfiguration = 0x1b;
  static constexpr uint8_t kAccelerometerConfiguration = 0x1c;
  static constexpr uint8_t kInterruptEnable = 0x38;
  static constexpr uint8_t kAccelXoutH = 0x3b;
  static constexpr uint8_t kPowerManagement1 = 0x6b;
  static constexpr uint8_t kWhoAmI = 0x75;
};

struct AxisMap {
  std::array<uint8_t, 3> index{0u, 1u, 2u};
  std::array<int8_t, 3> sign{1, 1, 1};
};

struct Mpu6050Config {
  AxisMap axis_map{};
  uint16_t accelerometer_range_g = 2u;
  uint16_t gyroscope_range_dps = 250u;
  uint8_t digital_low_pass_filter = 3u;
  uint16_t sample_rate_hz = 200u;
  uint16_t calibration_samples = 400u;
  float stationary_gyro_limit_dps = 5.0f;
  uint32_t timeout_us = 20000u;
};

struct Mpu6050Diagnostics {
  uint32_t i2c_errors = 0u;
  uint32_t missed_samples = 0u;
  uint32_t valid_samples = 0u;
  uint64_t first_sample_us = 0u;
  uint64_t last_sample_us = 0u;
  Vector3 raw_gyro_dps{};
  Vector3 gyro_bias_dps{};
  uint16_t calibration_samples = 0u;
  bool calibration_complete = false;
};

class IMpu6050Bus {
public:
  virtual ~IMpu6050Bus() = default;
  virtual bool begin(uint32_t frequency_hz) = 0;
  virtual bool read(uint8_t address, uint8_t reg, uint8_t *bytes,
                    size_t length) = 0;
  virtual bool write(uint8_t address, uint8_t reg, uint8_t value) = 0;
};

class Mpu6050Decoder {
public:
  static ImuSample decode(const uint8_t bytes[14], uint64_t timestamp_us,
                          const Mpu6050Config &config);
};

class GyroBiasCalibrator {
public:
  GyroBiasCalibrator(uint16_t required_samples, float stationary_limit_dps);

  bool add(const ImuSample &sample);
  bool complete() const;
  uint16_t acceptedSamples() const;
  Vector3 bias() const;
  void reset();

private:
  uint16_t required_samples_;
  float stationary_limit_dps_;
  Vector3 sum_{};
  uint16_t accepted_samples_ = 0u;
};

class Mpu6050Imu final : public IImu {
public:
  Mpu6050Imu(IMpu6050Bus &bus, IClock &clock, const Mpu6050Config &config);

  bool begin() override;
  bool sample(ImuSample &sample) override;
  uint8_t address() const;
  bool timedOut(uint64_t now_us) const;
  const Mpu6050Diagnostics &diagnostics() const;

private:
  bool detectAddress();
  bool configure();

  IMpu6050Bus &bus_;
  IClock &clock_;
  Mpu6050Config config_;
  GyroBiasCalibrator calibrator_;
  Mpu6050Diagnostics diagnostics_{};
  uint8_t address_ = 0u;
};

} // namespace gs::balance
