/* SPDX-License-Identifier: GPL-3.0-only */
#include "differential_drive_mixer.h"

#include <cassert>
#include <cmath>
#include <limits>

using gs::drive::DifferentialDriveConfig;
using gs::drive::DifferentialDriveMixer;

int main() {
  DifferentialDriveConfig config;
  config.linear_gain = 100.0f;
  config.angular_gain = 50.0f;
  config.maximum_command = 80.0f;
  config.slew_per_second = 1000.0f;
  DifferentialDriveMixer mixer(config);

  auto output = mixer.update(0.5f, 0.0f, 0.1f);
  assert(output.valid);
  assert(std::fabs(output.left - 50.0f) < 0.001f);
  assert(std::fabs(output.right - 50.0f) < 0.001f);

  mixer.stop();
  output = mixer.update(0.5f, 0.5f, 0.1f);
  assert(output.left > output.right);
  assert(std::fabs(output.left) <= 80.0f);
  assert(std::fabs(output.right) <= 80.0f);

  mixer.stop();
  output = mixer.update(2.0f, 2.0f, 0.1f);
  assert(std::fabs(output.left) <= 80.0f);
  assert(std::fabs(output.right) <= 80.0f);

  output = mixer.update(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.1f);
  assert(!output.valid);
  assert(output.left == 0.0f);
  assert(output.right == 0.0f);

  return 0;
}
