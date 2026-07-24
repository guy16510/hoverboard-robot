/* SPDX-License-Identifier: GPL-3.0-only */
#include "complementary_pitch_estimator.h"

#include <cmath>

namespace gs::balance {
namespace {
constexpr float kRadiansToDegrees = 57.29577951308232f;
}

ComplementaryPitchEstimator::ComplementaryPitchEstimator(
    const ComplementaryPitchConfig &config)
    : config_(config) {}

void ComplementaryPitchEstimator::reset(const ImuSample &sample) {
  estimate_.raw_pitch_deg = accelerometerPitch(sample);
  estimate_.filtered_pitch_deg = estimate_.raw_pitch_deg;
  estimate_.pitch_rate_dps = sample.gyro_dps.y;
  estimate_.dt_seconds = 0.0f;
  estimate_.valid = sample.accelerometer_plausible;
  last_timestamp_us_ = sample.timestamp_us;
}

bool ComplementaryPitchEstimator::update(const ImuSample &sample) {
  if (last_timestamp_us_ == 0u) {
    reset(sample);
    return estimate_.valid;
  }
  const float dt_seconds =
      static_cast<float>(sample.timestamp_us - last_timestamp_us_) / 1000000.0f;
  if (dt_seconds < config_.minimum_dt_seconds ||
      dt_seconds > config_.maximum_dt_seconds) {
    reset(sample);
    return false;
  }
  const float raw_pitch = accelerometerPitch(sample);
  const float integrated =
      estimate_.filtered_pitch_deg + sample.gyro_dps.y * dt_seconds;
  estimate_.raw_pitch_deg = raw_pitch;
  estimate_.filtered_pitch_deg = config_.gyroscope_weight * integrated +
                                 (1.0f - config_.gyroscope_weight) * raw_pitch;
  estimate_.pitch_rate_dps = sample.gyro_dps.y;
  estimate_.dt_seconds = dt_seconds;
  estimate_.valid = sample.accelerometer_plausible;
  last_timestamp_us_ = sample.timestamp_us;
  return estimate_.valid;
}

const PitchEstimate &ComplementaryPitchEstimator::estimate() const {
  return estimate_;
}

float ComplementaryPitchEstimator::accelerometerPitch(
    const ImuSample &sample) const {
  return std::atan2(sample.accel_g.x, sample.accel_g.z) * kRadiansToDegrees +
         config_.upright_offset_deg;
}

} // namespace gs::balance
