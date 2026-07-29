/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_motor_control.h"

#include <stddef.h>

#include "gs_safety.h"
#include "gs_wheel_mix.h"

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
                              uint32_t elapsed_ms, uint16_t rate_per_second,
                              uint16_t *remainder) {
  const int32_t delta = (int32_t)target - current;
  if (delta == 0 || remainder == NULL) {
    if (remainder != NULL) {
      *remainder = 0u;
    }
    return target;
  }
  const uint64_t scaled = (uint64_t)rate_per_second * elapsed_ms + *remainder;
  const uint64_t step = scaled / 1000u;
  *remainder = (uint16_t)(scaled % 1000u);
  if (step == 0u) {
    return current;
  }
  const uint32_t magnitude = (uint32_t)(delta < 0 ? -delta : delta);
  if (step >= magnitude) {
    *remainder = 0u;
    return target;
  }
  return delta > 0 ? (int16_t)(current + (int32_t)step)
                   : (int16_t)(current - (int32_t)step);
}

static void disable_bridge(gs_motor_controller *motor) {
  motor->compare_offset = 0u;
  motor->bridge_enabled = false;
  if (motor->bridge.disable != NULL) {
    motor->bridge.disable(motor->bridge.context);
  }
}

static void reset_hall_tracking(gs_motor_controller *motor) {
  motor->previous_hall = 0u;
  motor->hall_seen = false;
}

void gs_motor_init(gs_motor_controller *motor, gs_bridge_port bridge,
                   uint32_t now_ms) {
  const gs_motor_configuration configuration = {
      .commutation_profile = GS_COMMUTATION_SYMMETRIC_REVERSE,
      .hall_cycle_mode = GS_HALL_CYCLE_AUTO,
  };
  gs_motor_init_config(motor, bridge, &configuration, now_ms);
}

void gs_motor_init_profile(gs_motor_controller *motor, gs_bridge_port bridge,
                           gs_commutation_profile commutation_profile,
                           uint32_t now_ms) {
  const gs_motor_configuration configuration = {
      .commutation_profile = commutation_profile,
      .hall_cycle_mode = GS_HALL_CYCLE_AUTO,
  };
  gs_motor_init_config(motor, bridge, &configuration, now_ms);
}

void gs_motor_init_config(gs_motor_controller *motor, gs_bridge_port bridge,
                          const gs_motor_configuration *configuration,
                          uint32_t now_ms) {
  if (motor == NULL) {
    return;
  }
  const gs_motor_configuration defaults = {
      .commutation_profile = GS_COMMUTATION_SYMMETRIC_REVERSE,
      .hall_cycle_mode = GS_HALL_CYCLE_AUTO,
  };
  const gs_motor_configuration *selected =
      configuration != NULL ? configuration : &defaults;
  *motor = (gs_motor_controller){0};
  motor->bridge = bridge;
  motor->commutation_profile = selected->commutation_profile;
  gs_hall_cycle_init(&motor->hall_cycle, selected->hall_cycle_mode);
  motor->state = GS_MOTOR_DISABLED;
  motor->last_step_ms = now_ms;
  disable_bridge(motor);
}

bool gs_motor_bridge_active(const gs_motor_controller *motor) {
  return motor != NULL && motor->bridge_enabled && motor->compare_offset != 0u;
}

void gs_motor_force_off(gs_motor_controller *motor) {
  if (motor == NULL) {
    return;
  }
  motor->applied_command = 0;
  motor->requested_command = 0;
  motor->direction = 0;
  motor->ramp_remainder = 0u;
  motor->state = GS_MOTOR_DISABLED;
  reset_hall_tracking(motor);
  disable_bridge(motor);
}

void gs_motor_clear_fault(gs_motor_controller *motor, uint32_t now_ms) {
  if (motor == NULL) {
    return;
  }
  const gs_bridge_port bridge = motor->bridge;
  const gs_motor_configuration configuration = {
      .commutation_profile = motor->commutation_profile,
      .hall_cycle_mode = motor->hall_cycle.configured_mode,
  };
  const int32_t odometer = motor->odometer;
  gs_motor_init_config(motor, bridge, &configuration, now_ms);
  motor->odometer = odometer;
}

static gs_motor_output motor_output(const gs_motor_controller *motor,
                                    gs_hall_transition hall_result,
                                    bool faulted) {
  const gs_motor_output output = {
      {motor->applied_command, motor->compare_offset, motor->bridge_enabled},
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
  return motor_output(motor, hall_result, true);
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
    return motor_output(motor, GS_HALL_REPEATED, false);
  }

  const gs_motor_power_profile *profile = gs_motor_power_profile_current();
  motor->requested_command =
      gs_normalize_wheel_command(clamp_command(input->requested_command));
  if (motor->requested_command != 0 &&
      (input->hall == 0u || input->hall == 7u)) {
    return fault_motor(motor, GS_HALL_INVALID);
  }

  if (motor->state == GS_MOTOR_REVERSAL_DWELL) {
    disable_bridge(motor);
    if ((int32_t)(input->now_ms - motor->dwell_until_ms) < 0) {
      return motor_output(motor, GS_HALL_REPEATED, false);
    }
    motor->state = GS_MOTOR_DISABLED;
    motor->direction = 0;
    motor->last_step_ms = input->now_ms;
    motor->ramp_remainder = 0u;
    return motor_output(motor, GS_HALL_REPEATED, false);
  }

  const uint32_t elapsed_ms = input->now_ms - motor->last_step_ms;
  const int8_t requested_direction =
      command_direction(motor->requested_command);
  const int8_t applied_direction = command_direction(motor->applied_command);
  if (applied_direction != 0 && requested_direction != 0 &&
      applied_direction != requested_direction) {
    motor->applied_command = advance_toward(
        motor->applied_command, 0, elapsed_ms, profile->deceleration_per_second,
        &motor->ramp_remainder);
    motor->last_step_ms = input->now_ms;
    if (motor->applied_command == 0) {
      motor->state = GS_MOTOR_REVERSAL_DWELL;
      motor->dwell_until_ms = input->now_ms + GS_DIRECTION_DWELL_MS;
      reset_hall_tracking(motor);
      disable_bridge(motor);
      return motor_output(motor, GS_HALL_REPEATED, false);
    }
  } else {
    const bool increasing = absolute_command(motor->requested_command) >
                            absolute_command(motor->applied_command);
    motor->applied_command = advance_toward(
        motor->applied_command, motor->requested_command, elapsed_ms,
        increasing ? profile->acceleration_per_second
                   : profile->deceleration_per_second,
        &motor->ramp_remainder);
    motor->last_step_ms = input->now_ms;
  }

  if (motor->applied_command == 0) {
    motor->state = GS_MOTOR_DISABLED;
    motor->direction = 0;
    reset_hall_tracking(motor);
    disable_bridge(motor);
    return motor_output(motor, GS_HALL_REPEATED, false);
  }
  motor->direction = command_direction(motor->applied_command);
  gs_hall_transition hall_result = GS_HALL_REPEATED;
  if (!motor->hall_seen) {
    motor->previous_hall = input->hall;
    motor->hall_seen = true;
    motor->state = GS_MOTOR_STARTING;
  } else if (input->hall_changed) {
    hall_result = gs_hall_cycle_validate(
        &motor->hall_cycle, motor->previous_hall, input->hall, motor->direction,
        input->hall_interval_us);
    if (hall_result != GS_HALL_LEGAL) {
      return fault_motor(motor, hall_result);
    }
    motor->previous_hall = input->hall;
    motor->odometer += motor->direction;
    motor->state = GS_MOTOR_RUNNING;
  }

  const uint16_t compare = gs_motor_compare_for_command(motor->applied_command);
  if (compare == 0u) {
    disable_bridge(motor);
    return motor_output(motor, hall_result, false);
  }
  gs_commutation_vector vector;
  if (!gs_commutation_for_hall_profile(input->hall, motor->direction,
                                       motor->commutation_profile, &vector) ||
      motor->bridge.apply == NULL ||
      !motor->bridge.apply(motor->bridge.context, &vector, compare)) {
    return fault_motor(motor, GS_HALL_INVALID);
  }
  motor->compare_offset = compare;
  motor->bridge_enabled = true;
  return motor_output(motor, hall_result, false);
}
