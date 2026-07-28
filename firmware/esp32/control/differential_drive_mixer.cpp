/* SPDX-License-Identifier: GPL-3.0-only */
#include "differential_drive_mixer.h"

#include <algorithm>
#include <cmath>

namespace gs::drive {

DifferentialDriveMixer::DifferentialDriveMixer(
    const DifferentialDriveConfig &config)
    : config_(config) {}

float DifferentialDriveMixer::moveToward(float current, float target,
                                          float maximum_delta) {
  if (target > current) {
    return std::min(target, current + maximum_delta);
  }
  return std::max(target, current - maximum_delta);
}

DifferentialDriveOutput DifferentialDriveMixer::update(float linear_velocity,
                                                       float angular_velocity,
                                                       float dt_seconds) {
  DifferentialDriveOutput output;
  output.requested_linear = linear_velocity;
  output.requested_yaw = angular_velocity;
  if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity) ||
      !std::isfinite(dt_seconds) || dt_seconds <= 0.0f ||
      !std::isfinite(config_.linear_gain) ||
      !std::isfinite(config_.angular_gain) ||
      !std::isfinite(config_.maximum_command) ||
      !std::isfinite(config_.slew_per_second) ||
      config_.maximum_command <= 0.0f || config_.slew_per_second <= 0.0f ||
      config_.maximum_command > kDriveOutputLimit) {
    stop();
    output.valid = false;
    return output;
  }

  const float linear = linear_velocity * config_.linear_gain;
  const float angular = angular_velocity * config_.angular_gain;
  float target_left = linear + angular;
  float target_right = linear - angular;

  const float peak = std::max(std::fabs(target_left), std::fabs(target_right));
  if (peak > config_.maximum_command && peak > 0.0f) {
    const float scale = config_.maximum_command / peak;
    target_left *= scale;
    target_right *= scale;
  }
  target_left = std::clamp(target_left, -kDriveOutputLimit, kDriveOutputLimit);
  target_right = std::clamp(target_right, -kDriveOutputLimit, kDriveOutputLimit);

  const float maximum_delta = config_.slew_per_second * dt_seconds;
  left_ = std::clamp(moveToward(left_, target_left, maximum_delta),
                     -kDriveOutputLimit, kDriveOutputLimit);
  right_ = std::clamp(moveToward(right_, target_right, maximum_delta),
                      -kDriveOutputLimit, kDriveOutputLimit);
  output.target_left = target_left;
  output.target_right = target_right;
  output.left = left_;
  output.right = right_;
  return output;
}

void DifferentialDriveMixer::stop() {
  left_ = 0.0f;
  right_ = 0.0f;
}

} // namespace gs::drive
