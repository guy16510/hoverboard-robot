/* SPDX-License-Identifier: GPL-3.0-only */
#include "balance_configuration.h"

#include <cmath>
#include <limits>

namespace gs::balance {
namespace {

constexpr float kWireScale = 1000.0f;

void writeI32(uint8_t *output, int32_t value) {
  const uint32_t wire = static_cast<uint32_t>(value);
  output[0] = static_cast<uint8_t>(wire);
  output[1] = static_cast<uint8_t>(wire >> 8u);
  output[2] = static_cast<uint8_t>(wire >> 16u);
  output[3] = static_cast<uint8_t>(wire >> 24u);
}

int32_t readI32(const uint8_t *input) {
  const uint32_t wire = static_cast<uint32_t>(input[0]) |
                        static_cast<uint32_t>(input[1]) << 8u |
                        static_cast<uint32_t>(input[2]) << 16u |
                        static_cast<uint32_t>(input[3]) << 24u;
  return static_cast<int32_t>(wire);
}

bool inRange(float value, float minimum, float maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

bool validSign(float value) {
  return std::fabs(value - 1.0f) < 0.0001f || std::fabs(value + 1.0f) < 0.0001f;
}

bool assign(BalanceConfigKey key, float value, CascadedBalanceConfig &config) {
  switch (key) {
  case BalanceConfigKey::kInnerProportional:
    if (!inRange(value, 0.0f, 100.0f)) {
      return false;
    }
    config.inner.proportional = value;
    return true;
  case BalanceConfigKey::kInnerIntegral:
    if (!inRange(value, 0.0f, 50.0f)) {
      return false;
    }
    config.inner.integral = value;
    return true;
  case BalanceConfigKey::kInnerDerivative:
    if (!inRange(value, 0.0f, 20.0f)) {
      return false;
    }
    config.inner.derivative = value;
    return true;
  case BalanceConfigKey::kInnerIntegralLimit:
    if (!inRange(value, 0.0f, 100.0f)) {
      return false;
    }
    config.inner.integral_limit = value;
    return true;
  case BalanceConfigKey::kOuterProportional:
    if (!inRange(value, 0.0f, 20.0f)) {
      return false;
    }
    config.outer.proportional = value;
    return true;
  case BalanceConfigKey::kOuterIntegral:
    if (!inRange(value, 0.0f, 10.0f)) {
      return false;
    }
    config.outer.integral = value;
    return true;
  case BalanceConfigKey::kOuterDerivative:
    if (!inRange(value, 0.0f, 10.0f)) {
      return false;
    }
    config.outer.derivative = value;
    return true;
  case BalanceConfigKey::kOuterIntegralLimit:
    if (!inRange(value, 0.0f, 100.0f)) {
      return false;
    }
    config.outer.integral_limit = value;
    return true;
  case BalanceConfigKey::kDerivativeFilterHz:
    if (!inRange(value, 1.0f, 100.0f)) {
      return false;
    }
    config.derivative_filter_hz = value;
    return true;
  case BalanceConfigKey::kOutputLimit:
    if (!inRange(value, 0.0f, 250.0f)) {
      return false;
    }
    config.output_limit = value;
    return true;
  case BalanceConfigKey::kSlewPerSecond:
    if (!inRange(value, 1.0f, 5000.0f)) {
      return false;
    }
    config.slew_per_second = value;
    return true;
  case BalanceConfigKey::kMaximumPitchReference:
    if (!inRange(value, 0.0f, 15.0f)) {
      return false;
    }
    config.maximum_pitch_reference_deg = value;
    return true;
  case BalanceConfigKey::kYawGain:
    if (!inRange(value, -100.0f, 100.0f)) {
      return false;
    }
    config.yaw_gain = value;
    return true;
  case BalanceConfigKey::kYawLimit:
    if (!inRange(value, 0.0f, 250.0f)) {
      return false;
    }
    config.yaw_limit = value;
    return true;
  case BalanceConfigKey::kUprightOffset:
    if (!inRange(value, -15.0f, 15.0f)) {
      return false;
    }
    config.upright_offset_deg = value;
    return true;
  case BalanceConfigKey::kImuSign:
    if (!validSign(value)) {
      return false;
    }
    config.imu_sign = value;
    return true;
  case BalanceConfigKey::kWheelSign:
    if (!validSign(value)) {
      return false;
    }
    config.wheel_sign = value;
    return true;
  case BalanceConfigKey::kCount:
    return false;
  }
  return false;
}

} // namespace

size_t BalanceConfigurationCodec::encodeValue(BalanceConfigKey key, float value,
                                              uint8_t *payload,
                                              size_t capacity) {
  if (payload == nullptr || capacity < kPayloadSize || !std::isfinite(value)) {
    return 0u;
  }
  const float scaled = value * kWireScale;
  if (scaled < static_cast<float>(std::numeric_limits<int32_t>::min()) ||
      scaled > static_cast<float>(std::numeric_limits<int32_t>::max())) {
    return 0u;
  }
  payload[0] = static_cast<uint8_t>(key);
  writeI32(&payload[1], static_cast<int32_t>(std::lround(scaled)));
  return kPayloadSize;
}

size_t
BalanceConfigurationCodec::encodeCurrent(BalanceConfigKey key,
                                         const CascadedBalanceConfig &config,
                                         uint8_t *payload, size_t capacity) {
  float current = 0.0f;
  if (!value(key, config, current)) {
    return 0u;
  }
  return encodeValue(key, current, payload, capacity);
}

bool BalanceConfigurationCodec::applyUpdate(const uint8_t *payload,
                                            size_t length,
                                            CascadedBalanceConfig &config) {
  if (payload == nullptr || length != kPayloadSize ||
      payload[0] >= static_cast<uint8_t>(BalanceConfigKey::kCount)) {
    return false;
  }
  CascadedBalanceConfig updated = config;
  const auto key = static_cast<BalanceConfigKey>(payload[0]);
  const float candidate = static_cast<float>(readI32(&payload[1])) / kWireScale;
  if (!assign(key, candidate, updated)) {
    return false;
  }
  config = updated;
  return true;
}

bool BalanceConfigurationCodec::value(BalanceConfigKey key,
                                      const CascadedBalanceConfig &config,
                                      float &result) {
  switch (key) {
  case BalanceConfigKey::kInnerProportional:
    result = config.inner.proportional;
    return true;
  case BalanceConfigKey::kInnerIntegral:
    result = config.inner.integral;
    return true;
  case BalanceConfigKey::kInnerDerivative:
    result = config.inner.derivative;
    return true;
  case BalanceConfigKey::kInnerIntegralLimit:
    result = config.inner.integral_limit;
    return true;
  case BalanceConfigKey::kOuterProportional:
    result = config.outer.proportional;
    return true;
  case BalanceConfigKey::kOuterIntegral:
    result = config.outer.integral;
    return true;
  case BalanceConfigKey::kOuterDerivative:
    result = config.outer.derivative;
    return true;
  case BalanceConfigKey::kOuterIntegralLimit:
    result = config.outer.integral_limit;
    return true;
  case BalanceConfigKey::kDerivativeFilterHz:
    result = config.derivative_filter_hz;
    return true;
  case BalanceConfigKey::kOutputLimit:
    result = config.output_limit;
    return true;
  case BalanceConfigKey::kSlewPerSecond:
    result = config.slew_per_second;
    return true;
  case BalanceConfigKey::kMaximumPitchReference:
    result = config.maximum_pitch_reference_deg;
    return true;
  case BalanceConfigKey::kYawGain:
    result = config.yaw_gain;
    return true;
  case BalanceConfigKey::kYawLimit:
    result = config.yaw_limit;
    return true;
  case BalanceConfigKey::kUprightOffset:
    result = config.upright_offset_deg;
    return true;
  case BalanceConfigKey::kImuSign:
    result = config.imu_sign;
    return true;
  case BalanceConfigKey::kWheelSign:
    result = config.wheel_sign;
    return true;
  case BalanceConfigKey::kCount:
    return false;
  }
  return false;
}

} // namespace gs::balance
