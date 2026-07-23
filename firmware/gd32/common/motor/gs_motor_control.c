/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_motor_control.h"

#include <stddef.h>

#include "gs_safety.h"
#include "gs_wheel_mix.h"

enum {
  GS_ACCELERATION_PER_SECOND = 400,
  GS_DECELERATION_PER_SECOND = 800,
};

#ifndef GS_COMMAND_FULL_SCALE
#define GS_COMMAND_FULL_SCALE 1000
#endif

_Static_assert(GS_COMMAND_FULL_SCALE > GS_COMMAND_DEADBAND,
               "command full scale must exceed deadband");
_Static_assert(GS_COMMAND_FULL_SCALE <= 1000,
               "command full scale cannot exceed protocol range");

static int16_t absolute_command(int16_t value) {
  return value < 0 ? (int16_t)-value : value;
}

static int8_t command_direction(int16_t value) {
  return value > 0 ? 1 : (value < 0 ? -1 : 0);
}

static int16_t clamp_command(int16_t value) {
  if (value > 1000) {
    return 1000;
  }
  if (value < -1000) {
    return -1000;
  }
  return value;
}

static int16_t advance_toward(int16_t current, int16_t target,
                              uint32_t elapsed_ms, uint16_t rate_per_second) {
  int32_t step = (int32_t)rate_per_second * elapsed_ms / 1000;
  const int32_t delta = (int32_t)target - current;
  if (step == 0 && delta != 0) {
    step = 1;
  }
  if (delta > step) {
    return (int16_t)(current + step);
  }
  if (delta < -step) {
    return (int16_t)(current - step);
  }
  return target;
}

static uint16_t compare_for_command(int16_t command) {
  const int16_t magnitude = absolute_command(command);
  if (magnitude < GS_COMMAND_DEADBAND) {
    return 0;
  }
  if (magnitude >= GS_COMMAND_FULL_SCALE) {
    return GS_PWM_OFFSET_MAX;
  }
  const int32_t span = GS_PWM_OFFSET_MAX - GS_PWM_OFFSET_START;
  const int32_t scaled = (int32_t)(magnitude - GS_COMMAND_DEADBAND) * span /
                         (GS_COMMAND_FULL_SCALE - GS_COMMAND_DEADBAND);
  const int32_t compare = GS_PWM_OFFSET_START + scaled;
  return (uint16_t)(compare > GS_PWM_OFFSET_MAX ? GS_PWM_OFFSET_MAX : compare);
}

static void disable_bridge(gs_motor_controller *motor) {
  if (motor->bridge.disable != NULL) {
    motor->bridge.disable(motor->bridge.context);
  }
}

void gs_motor_init(gs_motor_controller *motor, gs_bridge_port bridge,
                   uint32_t now_ms) {
  if (motor == NULL) {
    return;
  }
  *motor = (gs_motor_controller){0};
  motor->bridge = bridge;
  motor->state = GS_MOTOR_DISABLED;
  motor->last_step_ms = now_ms;
  disable_bridge(motor);
}

void gs_motor_force_off(gs_motor_controller *motor) {
  if (motor == NULL) {
    return;
  }
  motor->applied_command = 0;
  motor->requested_command = 0;
  motor->direction = 0;
  motor->state = GS_MOTOR_DISABLED;
  disable_bridge(motor);
}

void gs_motor_clear_fault(gs_motor_controller *motor, uint32_t now_ms) {
  if (motor == NULL) {
    return;
  }
  const gs_bridge_port bridge = motor->bridge;
  const int32_t odometer = motor->odometer;
  *motor = (gs_motor_controller){0};
  motor->bridge = bridge;
  motor->odometer = odometer;
  motor->state = GS_MOTOR_DISABLED;
  motor->last_step_ms = now_ms;
  disable_bridge(motor);
}

static gs_motor_output motor_output(const gs_motor_controller *motor,
                                    gs_hall_transition hall_result,
                                    bool bridge_enabled, bool faulted) {
  const gs_motor_output output = {
      {motor->applied_command, compare_for_command(motor->applied_command),
       bridge_enabled},
      hall_result,
      faulted,
  };
  return output;
}

static gs_motor_output fault_motor(gs_motor_controller *motor,
                                   gs_hall_transition hall_result) {
  motor->applied_command = 0;
  motor->state = GS_MOTOR_FAULT;
  disable_bridge(motor);
  return motor_output(motor, hall_result, false, true);
}

gs_motor_output gs_motor_step(gs_motor_controller *motor,
                              const gs_motor_input *input) {
  if (motor == NULL || input == NULL) {
    const gs_motor_output empty = {{0, 0, false}, GS_HALL_INVALID, true};
    return empty;
  }
  if (!input->motion_permitted || motor->state == GS_MOTOR_FAULT) {
    gs_motor_force_off(motor);
    motor->last_step_ms = input->now_ms;
    return motor_output(motor, GS_HALL_REPEATED, false, false);
  }

  motor->requested_command =
      gs_normalize_wheel_command(clamp_command(input->requested_command));
  if (motor->requested_command != 0 &&
      (input->hall == 0u || input->hall == 7u)) {
    return fault_motor(motor, GS_HALL_INVALID);
  }

  if (motor->state == GS_MOTOR_REVERSAL_DWELL) {
    disable_bridge(motor);
    if ((int32_t)(input->now_ms - motor->dwell_until_ms) < 0) {
      return motor_output(motor, GS_HALL_REPEATED, false, false);
    }
    motor->state = GS_MOTOR_DISABLED;
    motor->direction = command_direction(motor->requested_command);
  }

  const uint32_t elapsed_ms = input->now_ms - motor->last_step_ms;
  const int8_t requested_direction =
      command_direction(motor->requested_command);
  const int8_t applied_direction = command_direction(motor->applied_command);
  if (applied_direction != 0 && requested_direction != 0 &&
      applied_direction != requested_direction) {
    motor->applied_command = advance_toward(
        motor->applied_command, 0, elapsed_ms, GS_DECELERATION_PER_SECOND);
    motor->last_step_ms = input->now_ms;
    if (motor->applied_command == 0) {
      motor->state = GS_MOTOR_REVERSAL_DWELL;
      motor->dwell_until_ms = input->now_ms + GS_DIRECTION_DWELL_MS;
      disable_bridge(motor);
      return motor_output(motor, GS_HALL_REPEATED, false, false);
    }
  } else {
    const bool increasing = absolute_command(motor->requested_command) >
                            absolute_command(motor->applied_command);
    motor->applied_command = advance_toward(
        motor->applied_command, motor->requested_command, elapsed_ms,
        increasing ? GS_ACCELERATION_PER_SECOND : GS_DECELERATION_PER_SECOND);
    motor->last_step_ms = input->now_ms;
  }

  if (motor->applied_command == 0) {
    motor->state = GS_MOTOR_DISABLED;
    motor->direction = 0;
    disable_bridge(motor);
    return motor_output(motor, GS_HALL_REPEATED, false, false);
  }
  motor->direction = command_direction(motor->applied_command);
  gs_hall_transition hall_result = GS_HALL_REPEATED;
  if (!motor->hall_seen) {
    motor->previous_hall = input->hall;
    motor->hall_seen = true;
    motor->state = GS_MOTOR_STARTING;
  } else if (input->hall_changed) {
    hall_result =
        gs_validate_hall_transition(motor->previous_hall, input->hall,
                                    motor->direction, input->hall_interval_us);
    if (hall_result != GS_HALL_LEGAL) {
      return fault_motor(motor, hall_result);
    }
    motor->previous_hall = input->hall;
    motor->odometer += motor->direction;
    motor->state = GS_MOTOR_RUNNING;
  }

  const uint16_t compare = compare_for_command(motor->applied_command);
  if (compare == 0u) {
    disable_bridge(motor);
    return motor_output(motor, hall_result, false, false);
  }
  gs_commutation_vector vector;
  if (!gs_commutation_for_hall(input->hall, motor->direction, &vector) ||
      motor->bridge.apply == NULL ||
      !motor->bridge.apply(motor->bridge.context, &vector, compare)) {
    return fault_motor(motor, GS_HALL_INVALID);
  }
  return motor_output(motor, hall_result, true, false);
}
