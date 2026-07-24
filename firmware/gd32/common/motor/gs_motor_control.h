/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_MOTOR_CONTROL_H
#define GS_MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "gs_commutation.h"
#include "gs_motor_profile.h"
#include "gs_types.h"

typedef struct {
  void *context;
  void (*disable)(void *context);
  bool (*apply)(void *context, const gs_commutation_vector *vector,
                uint16_t compare_offset);
} gs_bridge_port;

typedef enum {
  GS_MOTOR_DISABLED = 0,
  GS_MOTOR_STARTING,
  GS_MOTOR_RUNNING,
  GS_MOTOR_REVERSAL_DWELL,
  GS_MOTOR_FAULT,
} gs_motor_state;

typedef struct {
  gs_motor_state state;
  int16_t applied_command;
  int16_t requested_command;
  int32_t odometer;
  uint32_t last_step_ms;
  uint32_t dwell_until_ms;
  uint16_t compare_offset;
  uint16_t ramp_remainder;
  uint8_t previous_hall;
  int8_t direction;
  bool hall_seen;
  bool bridge_enabled;
  gs_bridge_port bridge;
} gs_motor_controller;

typedef struct {
  uint8_t hall;
  bool hall_changed;
  uint32_t hall_interval_us;
  bool motion_permitted;
  int16_t requested_command;
  uint32_t now_ms;
} gs_motor_input;

typedef struct {
  gs_motor_demand demand;
  gs_hall_transition hall_result;
  bool faulted;
} gs_motor_output;

void gs_motor_init(gs_motor_controller *motor, gs_bridge_port bridge,
                   uint32_t now_ms);
gs_motor_output gs_motor_step(gs_motor_controller *motor,
                              const gs_motor_input *input);
bool gs_motor_bridge_active(const gs_motor_controller *motor);
void gs_motor_force_off(gs_motor_controller *motor);
void gs_motor_clear_fault(gs_motor_controller *motor, uint32_t now_ms);

#endif
