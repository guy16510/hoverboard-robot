/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

namespace gs::drive {

struct DifferentialDriveConfig {
  float linear_gain = 650.0f;
  float angular_gain = 350.0f;
  float maximum_command = 700.0f;
  float slew_per_second = 900.0f;
};

struct DifferentialDriveOutput {
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
