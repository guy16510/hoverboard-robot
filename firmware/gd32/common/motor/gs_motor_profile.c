/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_motor_profile.h"

#include "gs_types.h"

#if !defined(GS_POWER_PROFILE_ID)
#error "GS_POWER_PROFILE_ID must select an explicit motor power profile"
#endif

#if GS_POWER_PROFILE_ID == GS_POWER_PROFILE_CONSERVATIVE_250
static const gs_motor_power_profile current_profile = {
    .id = GS_POWER_PROFILE_CONSERVATIVE_250,
    .command_deadband = GS_COMMAND_DEADBAND,
    .command_full_scale = 250,
    .startup_compare = 40u,
    .maximum_compare = 80u,
    .acceleration_per_second = 400u,
    .deceleration_per_second = 800u,
};
#elif GS_POWER_PROFILE_ID == GS_POWER_PROFILE_STANDARD_1000
static const gs_motor_power_profile current_profile = {
    .id = GS_POWER_PROFILE_STANDARD_1000,
    .command_deadband = GS_COMMAND_DEADBAND,
    .command_full_scale = 1000,
    .startup_compare = 40u,
    .maximum_compare = 80u,
    .acceleration_per_second = 400u,
    .deceleration_per_second = 800u,
};
#else
#error "Unknown GS_POWER_PROFILE_ID"
#endif

_Static_assert(GS_COMMAND_DEADBAND > 0, "deadband must be positive");

static int16_t absolute_command(int16_t value) {
  return value < 0 ? (int16_t)-value : value;
}

const gs_motor_power_profile *gs_motor_power_profile_current(void) {
  return &current_profile;
}

uint16_t gs_motor_compare_for_command(int16_t command) {
  const int16_t magnitude = absolute_command(command);
  if (magnitude < current_profile.command_deadband) {
    return 0u;
  }
  if (magnitude >= current_profile.command_full_scale) {
    return current_profile.maximum_compare;
  }
  const int32_t compare_span =
      (int32_t)current_profile.maximum_compare - current_profile.startup_compare;
  const int32_t command_span =
      (int32_t)current_profile.command_full_scale - current_profile.command_deadband;
  const int32_t scaled =
      ((int32_t)magnitude - current_profile.command_deadband) * compare_span /
      command_span;
  return (uint16_t)((int32_t)current_profile.startup_compare + scaled);
}
