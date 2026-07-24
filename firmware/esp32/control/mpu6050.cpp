/* SPDX-License-Identifier: GPL-3.0-only */
#include "mpu6050.h"

#include <algorithm>
#include <cmath>

namespace gs::balance {
namespace {

int16_t readSigned(const uint8_t *bytes) {
  const uint16_t value =
      static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) << 8u) | bytes[1];
  return static_cast<int16_t>(value);
}

float accelerometerScale(uint16_t range_g) {
  switch (range_g) {
  case 4u:
    return 8192.0f;
  case 8u:
    return 4096.0f;
  case 16u:
    return 2048.0f;
  case 2u:
  default:
    return 16384.0f;
  }
}

float gyroscopeScale(uint16_t range_dps) {
  switch (range_dps) {
  case 500u:
    return 65.5f;
  case 1000u:
    return 32.8f;
  case 2000u:
    return 16.4f;
  case 250u:
  default:
    return 131.0f;
  }
}

uint8_t rangeBits(uint16_t range, uint16_t base) {
  uint8_t bits = 0u;
  while (range > base && bits < 3u) {
    range = static_cast<uint16_t>(range / 2u);
    ++bits;
  }
  return static_cast<uint8_t>(bits << 3u);
}

Vector3 mapAxes(const Vector3 &raw, const AxisMap &map) {
  const float values[] = {raw.x, raw.y, raw.z};
  return {
      values[map.index[0]] * static_cast<float>(map.sign[0]),
      values[map.index[1]] * static_cast<float>(map.sign[1]),
      values[map.index[2]] * static_cast<float>(map.sign[2]),
  };
}

bool plausibleAcceleration(const Vector3 &acceleration) {
  const float magnitude = std::sqrt(acceleration.x * acceleration.x +
                                    acceleration.y * acceleration.y +
                                    acceleration.z * acceleration.z);
  return magnitude >= 0.5f && magnitude <= 1.5f;
}

bool stationary(const ImuSample &sample, float gyro_limit) {
  return plausibleAcceleration(sample.accel_g) &&
         std::fabs(sample.gyro_dps.x) <= gyro_limit &&
         std::fabs(sample.gyro_dps.y) <= gyro_limit &&
         std::fabs(sample.gyro_dps.z) <= gyro_limit;
}

bool supportedAccelerometerRange(uint16_t range) {
  return range == 2u || range == 4u || range == 8u || range == 16u;
}

bool supportedGyroscopeRange(uint16_t range) {
  return range == 250u || range == 500u || range == 1000u || range == 2000u;
}

bool validAxisMap(const AxisMap &map) {
  for (size_t axis = 0u; axis < map.index.size(); ++axis) {
    if (map.index[axis] >= 3u ||
        (map.sign[axis] != 1 && map.sign[axis] != -1)) {
      return false;
    }
    for (size_t other = axis + 1u; other < map.index.size(); ++other) {
      if (map.index[axis] == map.index[other]) {
        return false;
      }
    }
  }
  return true;
}

bool validConfig(const Mpu6050Config &config) {
  if (!validAxisMap(config.axis_map) ||
      !supportedAccelerometerRange(config.accelerometer_range_g) ||
      !supportedGyroscopeRange(config.gyroscope_range_dps) ||
      config.digital_low_pass_filter > 6u || config.sample_rate_hz < 200u ||
      config.sample_rate_hz > 1000u ||
      !std::isfinite(config.stationary_gyro_limit_dps) ||
      config.stationary_gyro_limit_dps <= 0.0f) {
    return false;
  }
  const uint32_t period_us = 1000000u / config.sample_rate_hz;
  return config.timeout_us >= period_us * 2u;
}

} // namespace

ImuSample Mpu6050Decoder::decode(const uint8_t bytes[14], uint64_t timestamp_us,
                                 const Mpu6050Config &config) {
  const float accel_scale = accelerometerScale(config.accelerometer_range_g);
  const float gyro_scale = gyroscopeScale(config.gyroscope_range_dps);
  const Vector3 raw_accel = {
      readSigned(&bytes[0]) / accel_scale,
      readSigned(&bytes[2]) / accel_scale,
      readSigned(&bytes[4]) / accel_scale,
  };
  const Vector3 raw_gyro = {
      readSigned(&bytes[8]) / gyro_scale,
      readSigned(&bytes[10]) / gyro_scale,
      readSigned(&bytes[12]) / gyro_scale,
  };
  ImuSample sample;
  sample.accel_g = mapAxes(raw_accel, config.axis_map);
  sample.gyro_dps = mapAxes(raw_gyro, config.axis_map);
  sample.temperature_c = readSigned(&bytes[6]) / 340.0f + 36.53f;
  sample.timestamp_us = timestamp_us;
  sample.accelerometer_plausible = plausibleAcceleration(sample.accel_g);
  return sample;
}

GyroBiasCalibrator::GyroBiasCalibrator(uint16_t required_samples,
                                       float stationary_limit_dps)
    : required_samples_(required_samples),
      stationary_limit_dps_(stationary_limit_dps) {}

bool GyroBiasCalibrator::add(const ImuSample &sample) {
  if (complete()) {
    return true;
  }
  if (!stationary(sample, stationary_limit_dps_)) {
    reset();
    return false;
  }
  sum_.x += sample.gyro_dps.x;
  sum_.y += sample.gyro_dps.y;
  sum_.z += sample.gyro_dps.z;
  ++accepted_samples_;
  return complete();
}

bool GyroBiasCalibrator::complete() const {
  return required_samples_ == 0u || accepted_samples_ >= required_samples_;
}

uint16_t GyroBiasCalibrator::acceptedSamples() const {
  return accepted_samples_;
}

Vector3 GyroBiasCalibrator::bias() const {
  if (accepted_samples_ == 0u) {
    return {};
  }
  const float count = static_cast<float>(accepted_samples_);
  return {sum_.x / count, sum_.y / count, sum_.z / count};
}

void GyroBiasCalibrator::reset() {
  sum_ = {};
  accepted_samples_ = 0u;
}

Mpu6050Imu::Mpu6050Imu(IMpu6050Bus &bus, IClock &clock,
                       const Mpu6050Config &config)
    : bus_(bus), clock_(clock), config_(config),
      calibrator_(config.calibration_samples,
                  config.stationary_gyro_limit_dps) {}

bool Mpu6050Imu::begin() {
  address_ = 0u;
  diagnostics_ = {};
  calibrator_.reset();
  return validConfig(config_) && bus_.begin(400000u) && detectAddress() &&
         configure();
}

bool Mpu6050Imu::sample(ImuSample &sample) {
  uint8_t bytes[14] = {};
  if (address_ == 0u || !bus_.read(address_, Mpu6050Registers::kAccelXoutH,
                                   bytes, sizeof(bytes))) {
    ++diagnostics_.i2c_errors;
    ++diagnostics_.missed_samples;
    return false;
  }
  sample = Mpu6050Decoder::decode(bytes, clock_.nowMicros(), config_);
  if (!sample.accelerometer_plausible) {
    ++diagnostics_.missed_samples;
    return false;
  }
  diagnostics_.raw_gyro_dps = sample.gyro_dps;
  calibrator_.add(sample);
  diagnostics_.calibration_samples = calibrator_.acceptedSamples();
  diagnostics_.gyro_bias_dps = calibrator_.bias();
  diagnostics_.calibration_complete = calibrator_.complete();
  if (diagnostics_.calibration_complete) {
    const Vector3 offset = diagnostics_.gyro_bias_dps;
    sample.gyro_dps.x -= offset.x;
    sample.gyro_dps.y -= offset.y;
    sample.gyro_dps.z -= offset.z;
  }
  if (diagnostics_.valid_samples == 0u) {
    diagnostics_.first_sample_us = sample.timestamp_us;
  }
  ++diagnostics_.valid_samples;
  diagnostics_.last_sample_us = sample.timestamp_us;
  return true;
}

uint8_t Mpu6050Imu::address() const { return address_; }

bool Mpu6050Imu::timedOut(uint64_t now_us) const {
  if (diagnostics_.valid_samples == 0u) {
    return true;
  }
  if (now_us <= diagnostics_.last_sample_us) {
    return false;
  }
  return now_us - diagnostics_.last_sample_us > config_.timeout_us;
}

const Mpu6050Diagnostics &Mpu6050Imu::diagnostics() const {
  return diagnostics_;
}

bool Mpu6050Imu::detectAddress() {
  for (const uint8_t candidate : {uint8_t{0x68}, uint8_t{0x69}}) {
    uint8_t identity = 0u;
    if (bus_.read(candidate, Mpu6050Registers::kWhoAmI, &identity, 1u) &&
        ((identity & 0x7eu) == 0x68u || identity == 0x70u)) {
      address_ = candidate;
      return true;
    }
  }
  return false;
}

bool Mpu6050Imu::configure() {
  const uint8_t divider = static_cast<uint8_t>(
      std::max<uint16_t>(1u, 1000u / config_.sample_rate_hz) - 1u);
  const std::array<std::array<uint8_t, 2>, 6> writes{{
      {Mpu6050Registers::kPowerManagement1, 0x01u},
      {Mpu6050Registers::kConfiguration, config_.digital_low_pass_filter},
      {Mpu6050Registers::kSampleRateDivider, divider},
      {Mpu6050Registers::kGyroscopeConfiguration,
       rangeBits(config_.gyroscope_range_dps, 250u)},
      {Mpu6050Registers::kAccelerometerConfiguration,
       rangeBits(config_.accelerometer_range_g, 2u)},
      {Mpu6050Registers::kInterruptEnable, 0x00u},
  }};
  for (const auto &write : writes) {
    if (!bus_.write(address_, write[0], write[1])) {
      ++diagnostics_.i2c_errors;
      return false;
    }
  }
  return true;
}

} // namespace gs::balance
