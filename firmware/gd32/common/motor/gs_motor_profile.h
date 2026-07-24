/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_MOTOR_PROFILE_H
#define GS_MOTOR_PROFILE_H

#include <stdint.h>

typedef enum {
  GS_POWER_PROFILE_CONSERVATIVE_250 = 1,
  GS_POWER_PROFILE_STANDARD_1000 = 2,
} gs_motor_power_profile_id;

typedef struct {
  gs_motor_power_profile_id id;
  int16_t command_deadband;
  int16_t command_full_scale;
  uint16_t startup_compare;
  uint16_t maximum_compare;
  uint16_t acceleration_per_second;
  uint16_t deceleration_per_second;
} gs_motor_power_profile;

const gs_motor_power_profile *gs_motor_power_profile_current(void);
uint16_t gs_motor_compare_for_command(int16_t command);

#endif
