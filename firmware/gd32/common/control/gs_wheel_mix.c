/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_wheel_mix.h"

#include <stdint.h>

#include "gs_safety.h"

enum { GS_WHEEL_LIMIT = 1000 };

static int32_t magnitude(int32_t value) { return value < 0 ? -value : value; }

static int16_t clamp_wheel(int32_t value) {
  if (value > GS_WHEEL_LIMIT) {
    return GS_WHEEL_LIMIT;
  }
  if (value < -GS_WHEEL_LIMIT) {
    return -GS_WHEEL_LIMIT;
  }
  return (int16_t)value;
}

int16_t gs_normalize_wheel_command(int16_t command) {
  const int16_t clamped = clamp_wheel(command);
  return magnitude(clamped) < GS_COMMAND_DEADBAND ? 0 : clamped;
}

gs_wheel_pair gs_normalize_wheel_pair(gs_wheel_pair pair) {
  pair.left = gs_normalize_wheel_command(pair.left);
  pair.right = gs_normalize_wheel_command(pair.right);
  return pair;
}

gs_wheel_pair gs_mix_wheels(int16_t speed, int16_t steer) {
  int32_t left = (int32_t)speed + steer;
  int32_t right = (int32_t)speed - steer;
  const int32_t largest =
      magnitude(left) > magnitude(right) ? magnitude(left) : magnitude(right);
  if (largest > GS_WHEEL_LIMIT) {
    left = left * GS_WHEEL_LIMIT / largest;
    right = right * GS_WHEEL_LIMIT / largest;
  }
  return gs_normalize_wheel_pair(
      (gs_wheel_pair){clamp_wheel(left), clamp_wheel(right)});
}

gs_wheel_pair gs_direct_wheels(int16_t left, int16_t right) {
  return gs_normalize_wheel_pair(
      (gs_wheel_pair){clamp_wheel(left), clamp_wheel(right)});
}

int16_t gs_slave_electrical_command(int16_t logical_right) {
  return (int16_t)-logical_right;
}

int32_t gs_slave_logical_odometer(int32_t electrical_odometer) {
  return -electrical_odometer;
}
