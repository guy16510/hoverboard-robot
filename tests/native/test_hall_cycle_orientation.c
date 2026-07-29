/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gs_hall_cycle.h"
#include "gs_motor_control.h"
#include "gs_wheel_mix.h"

typedef enum {
  TEST_MASTER = 0,
  TEST_SLAVE = 1,
} test_motor_role;

typedef struct {
  bool enabled;
  gs_commutation_vector vector;
} test_bridge;

static void bridge_disable(void *context) {
  test_bridge *bridge = context;
  bridge->enabled = false;
}

static bool bridge_apply(void *context, const gs_commutation_vector *vector,
                         uint16_t compare_offset) {
  test_bridge *bridge = context;
  bridge->enabled = compare_offset != 0u;
  bridge->vector = *vector;
  return true;
}

static gs_bridge_port bridge_port(test_bridge *bridge) {
  return (gs_bridge_port){
      .context = bridge,
      .disable = bridge_disable,
      .apply = bridge_apply,
  };
}

static int16_t electrical_command(test_motor_role role,
                                  int16_t logical_command) {
  return role == TEST_MASTER ? logical_command
                             : gs_slave_electrical_command(logical_command);
}

static int32_t logical_odometer(test_motor_role role, int32_t odometer) {
  return role == TEST_MASTER ? odometer : gs_slave_logical_odometer(odometer);
}

static uint8_t hardware_next_hall(uint8_t hall, int8_t electrical_direction) {
  static const uint8_t positive_command_cycle[8] = {
      0u, 3u, 6u, 2u, 5u, 1u, 4u, 0u,
  };
  static const uint8_t negative_command_cycle[8] = {
      0u, 5u, 3u, 1u, 6u, 4u, 2u, 0u,
  };
  return electrical_direction > 0 ? positive_command_cycle[hall]
                                  : negative_command_cycle[hall];
}

static gs_motor_output step_motor(gs_motor_controller *motor, uint8_t hall,
                                  bool hall_changed, uint32_t interval_us,
                                  int16_t command, uint32_t now_ms) {
  const gs_motor_input input = {
      .hall = hall,
      .hall_changed = hall_changed,
      .hall_interval_us = hall_changed ? interval_us : 0u,
      .motion_permitted = true,
      .requested_command = command,
      .now_ms = now_ms,
  };
  return gs_motor_step(motor, &input);
}

static uint32_t ramp_until_bridge_active(gs_motor_controller *motor,
                                         uint8_t hall, int16_t command,
                                         uint32_t now_ms) {
  for (uint32_t attempts = 0u; attempts < 1000u; ++attempts) {
    ++now_ms;
    const gs_motor_output output =
        step_motor(motor, hall, false, 0u, command, now_ms);
    GS_EXPECT_FALSE(output.faulted);
    if (gs_motor_bridge_active(motor)) {
      return now_ms;
    }
  }
  GS_EXPECT_TRUE(false);
  return now_ms;
}

static uint32_t drive_hardware_cycle(gs_motor_controller *motor, uint8_t *hall,
                                     int16_t electrical, uint32_t now_ms,
                                     uint32_t transitions) {
  const int8_t direction = electrical > 0 ? 1 : -1;
  for (uint32_t index = 0u; index < transitions; ++index) {
    *hall = hardware_next_hall(*hall, direction);
    now_ms += 100u;
    const gs_motor_output output =
        step_motor(motor, *hall, true, 100000u, electrical, now_ms);
    GS_EXPECT_FALSE(output.faulted);
    GS_EXPECT_EQ(GS_HALL_LEGAL, output.hall_result);
    GS_EXPECT_TRUE(output.demand.bridge_enabled);
  }
  return now_ms;
}

static void test_exact_hardware_cycle_runs_both_roles_both_directions(void) {
  static const uint8_t starting_halls[] = {1u, 3u, 2u, 6u, 4u, 5u};
  static const int16_t logical_commands[] = {250, -250};

  for (uint8_t role_value = TEST_MASTER; role_value <= TEST_SLAVE;
       ++role_value) {
    const test_motor_role role = (test_motor_role)role_value;
    for (size_t command_index = 0u;
         command_index < sizeof(logical_commands) / sizeof(logical_commands[0]);
         ++command_index) {
      for (size_t hall_index = 0u;
           hall_index < sizeof(starting_halls) / sizeof(starting_halls[0]);
           ++hall_index) {
        test_bridge bridge = {0};
        gs_motor_controller motor;
        uint8_t hall = starting_halls[hall_index];
        const int16_t logical = logical_commands[command_index];
        const int16_t electrical = electrical_command(role, logical);
        gs_motor_init(&motor, bridge_port(&bridge), 0u);

        uint32_t now_ms = ramp_until_bridge_active(&motor, hall, electrical, 0u);
        now_ms = drive_hardware_cycle(&motor, &hall, electrical, now_ms, 12u);
        (void)now_ms;

        GS_EXPECT_EQ(GS_HALL_CYCLE_COMMAND_INVERTED,
                     gs_hall_cycle_resolved_mode(&motor.hall_cycle));
        GS_EXPECT_EQ(GS_MOTOR_RUNNING, motor.state);
        GS_EXPECT_TRUE(electrical > 0 ? motor.odometer > 0
                                     : motor.odometer < 0);
        GS_EXPECT_TRUE(logical > 0
                           ? logical_odometer(role, motor.odometer) > 0
                           : logical_odometer(role, motor.odometer) < 0);
      }
    }
  }
}

static void test_learned_hardware_cycle_survives_direction_reversal(void) {
  for (uint8_t role_value = TEST_MASTER; role_value <= TEST_SLAVE;
       ++role_value) {
    const test_motor_role role = (test_motor_role)role_value;
    test_bridge bridge = {0};
    gs_motor_controller motor;
    uint8_t hall = 1u;
    int16_t electrical = electrical_command(role, 250);
    gs_motor_init(&motor, bridge_port(&bridge), 0u);

    uint32_t now_ms = ramp_until_bridge_active(&motor, hall, electrical, 0u);
    now_ms = drive_hardware_cycle(&motor, &hall, electrical, now_ms, 8u);
    GS_EXPECT_EQ(GS_HALL_CYCLE_COMMAND_INVERTED,
                 gs_hall_cycle_resolved_mode(&motor.hall_cycle));

    electrical = electrical_command(role, -250);
    for (uint32_t attempts = 0u; attempts < 2000u; ++attempts) {
      ++now_ms;
      const gs_motor_output output =
          step_motor(&motor, hall, false, 0u, electrical, now_ms);
      GS_EXPECT_FALSE(output.faulted);
      if (motor.direction == (electrical > 0 ? 1 : -1) &&
          gs_motor_bridge_active(&motor)) {
        break;
      }
    }
    GS_EXPECT_EQ(electrical > 0 ? 1 : -1, motor.direction);
    GS_EXPECT_TRUE(gs_motor_bridge_active(&motor));
    GS_EXPECT_EQ(GS_HALL_CYCLE_COMMAND_INVERTED,
                 gs_hall_cycle_resolved_mode(&motor.hall_cycle));

    const int32_t before_reverse = logical_odometer(role, motor.odometer);
    now_ms = drive_hardware_cycle(&motor, &hall, electrical, now_ms, 8u);
    (void)now_ms;
    GS_EXPECT_TRUE(logical_odometer(role, motor.odometer) < before_reverse);
  }
}

static void test_auto_orientation_still_rejects_bad_hall_edges(void) {
  test_bridge skipped_bridge = {0};
  gs_motor_controller skipped;
  gs_motor_init(&skipped, bridge_port(&skipped_bridge), 0u);
  uint32_t now_ms = ramp_until_bridge_active(&skipped, 1u, 250, 0u);
  const gs_motor_output skipped_output =
      step_motor(&skipped, 2u, true, 100000u, 250, now_ms + 100u);
  GS_EXPECT_TRUE(skipped_output.faulted);
  GS_EXPECT_EQ(GS_HALL_ILLEGAL, skipped_output.hall_result);
  GS_EXPECT_FALSE(gs_motor_bridge_active(&skipped));

  test_bridge fast_bridge = {0};
  gs_motor_controller fast;
  gs_motor_init(&fast, bridge_port(&fast_bridge), 0u);
  now_ms = ramp_until_bridge_active(&fast, 1u, 250, 0u);
  const gs_motor_output fast_output =
      step_motor(&fast, 3u, true, 499u, 250, now_ms + 1u);
  GS_EXPECT_TRUE(fast_output.faulted);
  GS_EXPECT_EQ(GS_HALL_TOO_FAST, fast_output.hall_result);
  GS_EXPECT_FALSE(gs_motor_bridge_active(&fast));
}

static void test_fixed_orientation_can_fail_closed(void) {
  const gs_motor_configuration configuration = {
      .commutation_profile = GS_COMMUTATION_SYMMETRIC_REVERSE,
      .hall_cycle_mode = GS_HALL_CYCLE_COMMAND_ALIGNED,
  };
  test_bridge bridge = {0};
  gs_motor_controller motor;
  gs_motor_init_config(&motor, bridge_port(&bridge), &configuration, 0u);
  const uint32_t now_ms = ramp_until_bridge_active(&motor, 1u, 250, 0u);
  const gs_motor_output output =
      step_motor(&motor, 3u, true, 100000u, 250, now_ms + 100u);
  GS_EXPECT_TRUE(output.faulted);
  GS_EXPECT_EQ(GS_HALL_ILLEGAL, output.hall_result);
  GS_EXPECT_FALSE(gs_motor_bridge_active(&motor));
}

void gs_test_hall_cycle_orientation(void) {
  test_exact_hardware_cycle_runs_both_roles_both_directions();
  test_learned_hardware_cycle_survives_direction_reversal();
  test_auto_orientation_still_rejects_bad_hall_edges();
  test_fixed_orientation_can_fail_closed();
}
