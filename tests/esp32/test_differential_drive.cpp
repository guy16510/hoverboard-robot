/* SPDX-License-Identifier: GPL-3.0-only */
#include "differential_drive_mixer.h"

#include <cassert>
#include <cmath>
#include <limits>

using gs::drive::DifferentialDriveConfig;
using gs::drive::DifferentialDriveMixer;
using gs::drive::kDriveOutputLimit;

namespace {

bool close(float first, float second, float tolerance = 0.001f) {
  return std::fabs(first - second) <= tolerance;
}

void assertBounded(float value) {
  assert(std::isfinite(value));
  assert(std::fabs(value) <= kDriveOutputLimit);
}

} // namespace

int main() {
  DifferentialDriveConfig config;
  config.linear_gain = 650.0f;
  config.angular_gain = 350.0f;
  config.maximum_command = 250.0f;
  config.slew_per_second = 500.0f;
  DifferentialDriveMixer mixer(config);

  auto output = mixer.update(0.2f, 0.0f, 0.01f);
  assert(output.valid);
  assert(close(output.left, 5.0f));
  assert(close(output.right, 5.0f));
  assert(close(output.target_left, 130.0f));
  assert(close(output.target_right, 130.0f));

  for (int index = 0; index < 30; ++index) {
    output = mixer.update(0.2f, 0.0f, 0.01f);
  }
  assert(close(output.left, output.right));
  assert(close(output.left, 130.0f));

  mixer.stop();
  output = mixer.update(0.0f, 0.5f, 0.5f);
  assert(output.left > output.right);
  assert(close(output.left, -output.right));
  assertBounded(output.left);
  assertBounded(output.right);

  mixer.stop();
  output = mixer.update(1.0f, 1.0f, 1.0f);
  assertBounded(output.target_left);
  assertBounded(output.target_right);
  assertBounded(output.left);
  assertBounded(output.right);
  assert(close(std::fabs(output.target_left), 250.0f));

  mixer.stop();
  output = mixer.update(-1.0f, -1.0f, 1.0f);
  assertBounded(output.left);
  assertBounded(output.right);

  mixer.stop();
  output = mixer.update(0.35f, 0.8f, 0.01f);
  assertBounded(output.target_left);
  assertBounded(output.target_right);
  assert(close(std::fabs(output.left), 5.0f));
  assert(close(std::fabs(output.right), 5.0f));

  output = mixer.update(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.01f);
  assert(!output.valid);
  assert(output.left == 0.0f);
  assert(output.right == 0.0f);

  DifferentialDriveConfig unsafe = config;
  unsafe.maximum_command = 700.0f;
  DifferentialDriveMixer unsafe_mixer(unsafe);
  output = unsafe_mixer.update(0.1f, 0.0f, 0.01f);
  assert(!output.valid);
  assert(output.left == 0.0f);
  assert(output.right == 0.0f);

  return 0;
}
