/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

namespace gs::drive {

constexpr float kDriveOutputLimit = 250.0f;

struct DifferentialDriveConfig {
  float linear_gain = 650.0f;
  float angular_gain = 350.0f;
  float maximum_command = kDriveOutputLimit;
  float slew_per_second = 500.0f;
};

struct DifferentialDriveOutput {
  float requested_linear = 0.0f;
  float requested_yaw = 0.0f;
  float target_left = 0.0f;
  float target_right = 0.0f;
  float left = 0.0f;
  float right = 0.0f;
  bool valid = true;
};

class DifferentialDriveMixer {
public:
  explicit DifferentialDriveMixer(const DifferentialDriveConfig &config);
  DifferentialDriveOutput update(float linear_velocity, float angular_velocity,
                                 float dt_seconds);
  void stop();

private:
  static float moveToward(float current, float target, float maximum_delta);
  DifferentialDriveConfig config_;
  float left_ = 0.0f;
  float right_ = 0.0f;
};

} // namespace gs::drive
