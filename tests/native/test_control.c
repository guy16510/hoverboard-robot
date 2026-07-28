/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "gausstop_bridge_profile.h"
#include "gs_commutation.h"
#include "gs_console.h"
#include "gs_motor_control.h"
#include "gs_safety.h"
#include "gs_wheel_mix.h"

static void test_wheel_coordination(void) {
  gs_wheel_pair pair;

  pair = gs_mix_wheels(0, 0);
  GS_EXPECT_EQ(0, pair.left);
  GS_EXPECT_EQ(0, pair.right);
  pair = gs_mix_wheels(1000, 0);
  GS_EXPECT_EQ(1000, pair.left);
  GS_EXPECT_EQ(1000, pair.right);
  pair = gs_mix_wheels(-1000, 0);
  GS_EXPECT_EQ(-1000, pair.left);
  GS_EXPECT_EQ(-1000, pair.right);
  pair = gs_mix_wheels(0, 1000);
  GS_EXPECT_EQ(1000, pair.left);
  GS_EXPECT_EQ(-1000, pair.right);
  pair = gs_mix_wheels(0, -1000);
  GS_EXPECT_EQ(-1000, pair.left);
  GS_EXPECT_EQ(1000, pair.right);
  pair = gs_mix_wheels(800, 400);
  GS_EXPECT_EQ(1000, pair.left);
  GS_EXPECT_EQ(333, pair.right);
  pair = gs_direct_wheels(-123, 456);
  GS_EXPECT_EQ(-123, pair.left);
  GS_EXPECT_EQ(456, pair.right);
  GS_EXPECT_EQ(-456, gs_slave_electrical_command(pair.right));
  GS_EXPECT_EQ(123456, gs_slave_logical_odometer(-123456));
}

static void expect_vector(uint8_t hall, int8_t direction, gs_phase source,
                          gs_phase sink, gs_phase floating) {
  gs_commutation_vector vector = {0};
  GS_EXPECT_TRUE(gs_commutation_for_hall(hall, direction, &vector));
  GS_EXPECT_EQ(source, vector.source);
  GS_EXPECT_EQ(sink, vector.sink);
  GS_EXPECT_EQ(floating, vector.floating);
  GS_EXPECT_TRUE(vector.source != vector.sink);
  GS_EXPECT_TRUE(vector.source != vector.floating);
  GS_EXPECT_TRUE(vector.sink != vector.floating);
}

static void test_commutation_and_hall_sequences(void) {
  static const uint8_t forward[] = {2, 3, 1, 5, 4, 6, 2};
  static const uint8_t reverse[] = {2, 6, 4, 5, 1, 3, 2};

  /* Exact legacy-final maps from the approved plan, not an arbitrary mapping.
   */
  expect_vector(1, 1, GS_PHASE_B, GS_PHASE_Y, GS_PHASE_G);
  expect_vector(2, 1, GS_PHASE_Y, GS_PHASE_G, GS_PHASE_B);
  expect_vector(3, 1, GS_PHASE_B, GS_PHASE_G, GS_PHASE_Y);
  expect_vector(4, 1, GS_PHASE_G, GS_PHASE_B, GS_PHASE_Y);
  expect_vector(5, 1, GS_PHASE_G, GS_PHASE_Y, GS_PHASE_B);
  expect_vector(6, 1, GS_PHASE_Y, GS_PHASE_B, GS_PHASE_G);
  expect_vector(2, -1, GS_PHASE_G, GS_PHASE_B, GS_PHASE_Y);
  GS_EXPECT_FALSE(gs_commutation_for_hall(0, 1, NULL));
  GS_EXPECT_FALSE(gs_commutation_for_hall(7, 1, NULL));

  for (size_t i = 1; i < sizeof(forward); ++i) {
    GS_EXPECT_EQ(GS_HALL_LEGAL, gs_validate_hall_transition(
                                    forward[i - 1], forward[i], 1, 500));
  }
  for (size_t i = 1; i < sizeof(reverse); ++i) {
    GS_EXPECT_EQ(GS_HALL_LEGAL, gs_validate_hall_transition(
                                    reverse[i - 1], reverse[i], -1, 500));
  }
  GS_EXPECT_EQ(GS_HALL_REPEATED, gs_validate_hall_transition(2, 2, 1, 0));
  GS_EXPECT_EQ(GS_HALL_TOO_FAST, gs_validate_hall_transition(2, 3, 1, 499));
  GS_EXPECT_EQ(GS_HALL_ILLEGAL, gs_validate_hall_transition(2, 1, 1, 500));
  GS_EXPECT_EQ(GS_HALL_INVALID, gs_validate_hall_transition(0, 2, 1, 500));
}

static void test_symmetric_reverse_profile_opposes_each_forward_vector(void) {
  for (uint8_t hall = 1u; hall <= 6u; ++hall) {
    gs_commutation_vector forward = {0};
    gs_commutation_vector reverse = {0};
    GS_EXPECT_TRUE(gs_commutation_for_hall_profile(
        hall, 1, GS_COMMUTATION_PHASE_ADVANCED_REVERSE, &forward));
    GS_EXPECT_TRUE(gs_commutation_for_hall_profile(
        hall, -1, GS_COMMUTATION_SYMMETRIC_REVERSE, &reverse));
    GS_EXPECT_EQ(forward.source, reverse.sink);
    GS_EXPECT_EQ(forward.sink, reverse.source);
    GS_EXPECT_EQ(forward.floating, reverse.floating);
  }
}

static void test_proven_bridge_and_power_profiles(void) {
  const gs_motor_power_profile *power = gs_motor_power_profile_current();

  GS_EXPECT_EQ(GS_POWER_PROFILE_CONSERVATIVE_250, power->id);
  GS_EXPECT_EQ(40, power->startup_compare);
  GS_EXPECT_EQ(100, power->maximum_compare);
  GS_EXPECT_EQ(100, gs_motor_compare_for_command(250));

  GS_EXPECT_EQ(GS_BRIDGE_PROFILE_PROVEN_THREE_LEG_MIDPOINT,
               gs_bridge_profile_current());
  for (uint8_t hall = 1u; hall <= 6u; ++hall) {
    for (int8_t direction = -1; direction <= 1; direction += 2) {
      gs_commutation_vector vector = {0};
      gs_bridge_drive_plan plan = {0};
      GS_EXPECT_TRUE(gs_commutation_for_hall(hall, direction, &vector));
      GS_EXPECT_TRUE(gs_bridge_make_drive_plan(&vector, 500u, 100u,
                                               power->maximum_compare, &plan));
      GS_EXPECT_EQ(600, plan.compare[vector.source]);
      GS_EXPECT_EQ(400, plan.compare[vector.sink]);
      GS_EXPECT_EQ(500, plan.compare[vector.floating]);
      GS_EXPECT_EQ(GS_BRIDGE_ALL_PHASES_MASK, plan.enabled_phase_mask);
    }
  }

  gs_bridge_drive_plan plan = {0};
  const gs_commutation_vector invalid = {GS_PHASE_G, GS_PHASE_G, GS_PHASE_B};
  GS_EXPECT_FALSE(gs_bridge_make_drive_plan(&invalid, 500u, 100u,
                                            power->maximum_compare, &plan));
  const gs_commutation_vector valid = {
      GS_PHASE_Y,
      GS_PHASE_B,
      GS_PHASE_G,
  };
  GS_EXPECT_FALSE(gs_bridge_make_drive_plan(&valid, 500u, 101u,
                                            power->maximum_compare, &plan));
}

static void test_console_strict_state_transitions(void) {
  gs_console_state state;
  gs_console_state before;
  gs_console_init(&state);

  GS_EXPECT_FALSE(state.enabled);
  GS_EXPECT_FALSE(state.shutdown);
  GS_EXPECT_EQ(0, state.target.first);
  GS_EXPECT_EQ(0, state.target.second);
  GS_EXPECT_EQ(10, state.ramp_per_tick);
  before = state;
  GS_EXPECT_EQ(GS_CONSOLE_REJECTED, gs_console_execute(&state, "drive 1 2", 9));
  GS_EXPECT_BYTES(&before, &state, sizeof(state));
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "enable", 6));
  GS_EXPECT_TRUE(state.enabled);
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED,
               gs_console_execute(&state, "drive 500 -100", 14));
  GS_EXPECT_EQ(GS_DRIVE_MIXED, state.target.mode);
  GS_EXPECT_EQ(500, state.target.first);
  GS_EXPECT_EQ(-100, state.target.second);
  gs_console_ramp_tick(&state);
  GS_EXPECT_EQ(10, state.current.left);
  GS_EXPECT_EQ(10, state.current.right);
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "ramp 25", 7));
  GS_EXPECT_EQ(25, state.ramp_per_tick);
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "lr -50 75", 9));
  GS_EXPECT_EQ(GS_DRIVE_DIRECT_LR, state.target.mode);
  GS_EXPECT_EQ(-50, state.target.first);
  GS_EXPECT_EQ(75, state.target.second);
  before = state;
  GS_EXPECT_EQ(GS_CONSOLE_REJECTED, gs_console_execute(&state, "lr 1 2x", 7));
  GS_EXPECT_BYTES(&before, &state, sizeof(state));
  GS_EXPECT_EQ(GS_CONSOLE_REJECTED,
               gs_console_execute(&state, "drive 999999999999 0", 20));
  GS_EXPECT_BYTES(&before, &state, sizeof(state));
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "stop", 4));
  GS_EXPECT_EQ(0, state.target.first);
  GS_EXPECT_EQ(0, state.target.second);
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "disable", 7));
  GS_EXPECT_FALSE(state.enabled);
  GS_EXPECT_EQ(0, state.current.left);
  GS_EXPECT_EQ(0, state.current.right);
  GS_EXPECT_EQ(GS_CONSOLE_CLEAR_FAULT,
               gs_console_execute(&state, "clearfault", 10));
  GS_EXPECT_EQ(GS_CONSOLE_STATUS, gs_console_execute(&state, "status", 6));
  GS_EXPECT_EQ(GS_CONSOLE_HELP, gs_console_execute(&state, "help", 4));
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "shutdown", 8));
  GS_EXPECT_TRUE(state.shutdown);
  GS_EXPECT_FALSE(state.enabled);
  before = state;
  GS_EXPECT_EQ(GS_CONSOLE_REJECTED, gs_console_execute(&state, "enable", 6));
  GS_EXPECT_BYTES(&before, &state, sizeof(state));
  GS_EXPECT_EQ(
      GS_CONSOLE_REJECTED,
      gs_console_execute(
          &state,
          "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
          64));
}

static void test_console_ramp_waits_for_motion_ready(void) {
  gs_console_state state;
  gs_console_init(&state);
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED, gs_console_execute(&state, "enable", 6));
  GS_EXPECT_EQ(GS_CONSOLE_APPLIED,
               gs_console_execute(&state, "lr 100 100", 10));

  gs_console_ramp_tick_when_ready(&state, false);
  GS_EXPECT_EQ(0, state.current.left);
  GS_EXPECT_EQ(0, state.current.right);

  gs_console_ramp_tick_when_ready(&state, true);
  GS_EXPECT_EQ(10, state.current.left);
  GS_EXPECT_EQ(10, state.current.right);

  gs_console_ramp_tick_when_ready(&state, false);
  GS_EXPECT_EQ(0, state.current.left);
  GS_EXPECT_EQ(0, state.current.right);
}

static gs_safety_sample safe_sample(uint16_t adc) {
  const gs_safety_sample sample = {true, true, adc, true};
  return sample;
}

static void calibrate_adc(gs_safety_supervisor *s, uint16_t base) {
  for (unsigned i = 0; i < GS_ADC_CALIBRATION_SAMPLES; ++i) {
    gs_safety_sample_adc_off(s, (uint16_t)(base + (i & 1u)));
  }
}

static void test_safety_zero_startup_timeouts_and_inputs(void) {
  gs_safety_supervisor s;
  gs_safety_sample sample = safe_sample(2000);

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  GS_EXPECT_FALSE(gs_safety_bridge_allowed(&s));
  gs_safety_set_enabled(&s, true);
  gs_safety_note_command(&s, 1);
  gs_safety_note_demand(&s, true, 1);
  gs_safety_evaluate(&s, &sample, 2);
  GS_EXPECT_TRUE(gs_safety_bridge_allowed(&s));
  gs_safety_evaluate(&s, &sample, 402);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_COMMAND_TIMEOUT) != 0u);
  GS_EXPECT_FALSE(gs_safety_bridge_allowed(&s));

  gs_safety_init(&s, GS_SAFETY_SLAVE, false, 0);
  calibrate_adc(&s, 2000);
  gs_safety_set_enabled(&s, true);
  gs_safety_note_command(&s, 1);
  gs_safety_note_demand(&s, true, 1);
  gs_safety_evaluate(&s, &sample, 102);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  gs_safety_set_enabled(&s, true);
  gs_safety_note_command(&s, 500);
  gs_safety_note_demand(&s, true, 1);
  gs_safety_evaluate(&s, &sample, 702);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_STARTUP_TIMEOUT) != 0u);

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  gs_safety_set_enabled(&s, true);
  gs_safety_note_command(&s, 100);
  gs_safety_note_demand(&s, true, 1);
  gs_safety_note_hall(&s, 10);
  gs_safety_evaluate(&s, &sample, 311);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_STALL) != 0u);

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  sample.pa4_high = false;
  gs_safety_evaluate(&s, &sample, 1);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_SHUTDOWN) != 0u);

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  sample = safe_sample(4010);
  gs_safety_evaluate(&s, &sample, 1);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_PROTECTION) != 0u);
}

static void test_safety_adc_watchdog_latching_and_clear(void) {
  gs_safety_supervisor s;
  gs_safety_sample sample = safe_sample(2000);

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  for (unsigned i = 0; i < GS_ADC_CALIBRATION_SAMPLES; ++i) {
    gs_safety_sample_adc_off(&s, (uint16_t)(1000 + i * 10));
  }
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_ADC_CALIBRATION) != 0u);

  gs_safety_init(&s, GS_SAFETY_MASTER, true, 0);
  GS_EXPECT_TRUE((s.faults.bits & GS_FAULT_WATCHDOG_LOCKOUT) != 0u);
  GS_EXPECT_FALSE(gs_safety_clear(&s, &sample, 1000));

  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  gs_safety_latch(&s, GS_FAULT_PROTOCOL);
  GS_EXPECT_FALSE(gs_safety_clear(&s, &sample, 249));
  GS_EXPECT_TRUE(gs_safety_clear(&s, &sample, 250));
  GS_EXPECT_EQ(0, s.faults.bits);
  GS_EXPECT_FALSE(s.enabled);

  gs_safety_latch(&s, GS_FAULT_HALL_INVALID);
  sample.hall_valid = false;
  GS_EXPECT_FALSE(gs_safety_clear(&s, &sample, 1000));

  sample = safe_sample(2000);
  gs_safety_init(&s, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&s, 2000);
  gs_safety_set_enabled(&s, true);
  gs_safety_note_demand(&s, true, 100);
  gs_safety_latch(&s, GS_FAULT_PROTOCOL);
  gs_safety_set_enabled(&s, false);
  gs_safety_note_demand(&s, false, 500);
  GS_EXPECT_EQ(500, s.bridge_off_since_ms);
  GS_EXPECT_FALSE(gs_safety_clear(&s, &sample, 749));
  GS_EXPECT_TRUE(gs_safety_clear(&s, &sample, 750));
}

typedef struct {
  unsigned disable_count;
  unsigned apply_count;
  uint16_t last_offset;
  gs_commutation_vector last_vector;
} fake_bridge;

static void fake_disable(void *context) {
  fake_bridge *bridge = context;
  ++bridge->disable_count;
}

static bool fake_apply(void *context, const gs_commutation_vector *vector,
                       uint16_t compare_offset) {
  fake_bridge *bridge = context;
  ++bridge->apply_count;
  bridge->last_vector = *vector;
  bridge->last_offset = compare_offset;
  return true;
}

static void test_motor_uses_configured_symmetric_reverse_profile(void) {
  fake_bridge fake = {0};
  const gs_bridge_port port = {&fake, fake_disable, fake_apply};
  gs_motor_controller motor;

  gs_motor_init_profile(&motor, port, GS_COMMUTATION_SYMMETRIC_REVERSE, 0);
  gs_motor_clear_fault(&motor, 10);
  GS_EXPECT_EQ(GS_COMMUTATION_SYMMETRIC_REVERSE,
               motor.commutation_profile);
  const gs_motor_output out = gs_motor_step(
      &motor, &(gs_motor_input){2, false, 0, true, -250, 210});

  GS_EXPECT_TRUE(out.demand.bridge_enabled);
  GS_EXPECT_EQ(GS_PHASE_G, fake.last_vector.source);
  GS_EXPECT_EQ(GS_PHASE_Y, fake.last_vector.sink);
  GS_EXPECT_EQ(GS_PHASE_B, fake.last_vector.floating);
}

static void test_motor_ramps_limits_stops_and_reverses(void) {
  fake_bridge fake = {0};
  const gs_bridge_port port = {&fake, fake_disable, fake_apply};
  gs_motor_controller motor;
  gs_motor_output out;

  gs_motor_init(&motor, port, 0);
  GS_EXPECT_EQ(1, fake.disable_count);
  GS_EXPECT_EQ(GS_MOTOR_DISABLED, motor.state);
  out =
      gs_motor_step(&motor, &(gs_motor_input){2, false, 1000, true, 1000, 50});
  GS_EXPECT_EQ(20, out.demand.logical_command);
  GS_EXPECT_FALSE(out.demand.bridge_enabled);
  out =
      gs_motor_step(&motor, &(gs_motor_input){2, false, 1000, true, 1000, 150});
  GS_EXPECT_EQ(60, out.demand.logical_command);
  GS_EXPECT_TRUE(out.demand.bridge_enabled);
  GS_EXPECT_EQ(gs_motor_compare_for_command(60), out.demand.compare_offset);
  GS_EXPECT_TRUE(fake.last_offset <=
                 gs_motor_power_profile_current()->maximum_compare);

  out = gs_motor_step(&motor, &(gs_motor_input){3, true, 500, true, 1000, 200});
  GS_EXPECT_EQ(GS_HALL_LEGAL, out.hall_result);
  GS_EXPECT_EQ(1, motor.odometer);
  out = gs_motor_step(&motor, &(gs_motor_input){3, false, 0, true, 0, 250});
  GS_EXPECT_EQ(40, out.demand.logical_command);
  GS_EXPECT_FALSE(out.demand.bridge_enabled);
  out = gs_motor_step(&motor, &(gs_motor_input){3, false, 0, true, 0, 300});
  GS_EXPECT_EQ(0, out.demand.logical_command);
  GS_EXPECT_EQ(GS_MOTOR_DISABLED, motor.state);

  motor.applied_command = 100;
  motor.direction = 1;
  motor.last_step_ms = 300;
  out = gs_motor_step(&motor, &(gs_motor_input){3, false, 0, true, -1000, 350});
  GS_EXPECT_EQ(60, out.demand.logical_command);
  out = gs_motor_step(&motor, &(gs_motor_input){3, false, 0, true, -1000, 450});
  GS_EXPECT_EQ(0, out.demand.logical_command);
  GS_EXPECT_EQ(GS_MOTOR_REVERSAL_DWELL, motor.state);
  out = gs_motor_step(&motor, &(gs_motor_input){5, false, 0, true, -1000, 699});
  GS_EXPECT_EQ(0, out.demand.logical_command);
  GS_EXPECT_FALSE(out.demand.bridge_enabled);
  out = gs_motor_step(&motor, &(gs_motor_input){5, false, 0, true, -1000, 700});
  GS_EXPECT_EQ(0, out.demand.logical_command);
  GS_EXPECT_FALSE(out.demand.bridge_enabled);
  out = gs_motor_step(&motor, &(gs_motor_input){5, false, 0, true, -1000, 703});
  GS_EXPECT_TRUE(out.demand.logical_command < 0);
  out = gs_motor_step(&motor,
                      &(gs_motor_input){1, true, 100000, true, -1000, 750});
  GS_EXPECT_EQ(GS_HALL_LEGAL, out.hall_result);
  GS_EXPECT_FALSE(out.faulted);
}

static void test_motor_hall_faults_and_immediate_off(void) {
  fake_bridge fake = {0};
  const gs_bridge_port port = {&fake, fake_disable, fake_apply};
  gs_motor_controller motor;
  gs_motor_output out;

  gs_motor_init(&motor, port, 0);
  out =
      gs_motor_step(&motor, &(gs_motor_input){0, false, 1000, true, 1000, 200});
  GS_EXPECT_TRUE(out.faulted);
  GS_EXPECT_EQ(GS_HALL_INVALID, out.hall_result);
  GS_EXPECT_EQ(GS_MOTOR_FAULT, motor.state);
  GS_EXPECT_FALSE(out.demand.bridge_enabled);

  gs_motor_init(&motor, port, 0);
  (void)gs_motor_step(&motor,
                      &(gs_motor_input){2, false, 1000, true, 1000, 200});
  out = gs_motor_step(&motor, &(gs_motor_input){3, true, 499, true, 1000, 250});
  GS_EXPECT_TRUE(out.faulted);
  GS_EXPECT_EQ(GS_HALL_TOO_FAST, out.hall_result);

  gs_motor_init(&motor, port, 0);
  motor.ramp_remainder = 999u;
  out = gs_motor_step(&motor,
                      &(gs_motor_input){2, false, 1000, false, 1000, 200});
  GS_EXPECT_FALSE(out.demand.bridge_enabled);
  GS_EXPECT_EQ(0, out.demand.logical_command);
  GS_EXPECT_EQ(0, motor.ramp_remainder);
}

static void test_motor_reanchors_hall_after_coast_stop(void) {
  fake_bridge fake = {0};
  const gs_bridge_port port = {&fake, fake_disable, fake_apply};
  gs_motor_controller motor;

  gs_motor_init(&motor, port, 0);
  (void)gs_motor_step(&motor, &(gs_motor_input){2, false, 0, true, 1000, 200});
  gs_motor_output out = gs_motor_step(
      &motor, &(gs_motor_input){3, true, 100000, true, 1000, 250});
  GS_EXPECT_EQ(GS_HALL_LEGAL, out.hall_result);
  GS_EXPECT_EQ(1, motor.odometer);

  out = gs_motor_step(&motor, &(gs_motor_input){3, false, 0, true, 0, 400});
  GS_EXPECT_EQ(GS_MOTOR_DISABLED, motor.state);
  GS_EXPECT_FALSE(motor.hall_seen);
  (void)gs_motor_step(&motor, &(gs_motor_input){5, true, 100000, true, 0, 450});

  (void)gs_motor_step(&motor, &(gs_motor_input){5, false, 0, true, 1000, 550});
  out = gs_motor_step(&motor,
                      &(gs_motor_input){4, true, 100000, true, 1000, 600});
  GS_EXPECT_EQ(GS_HALL_LEGAL, out.hall_result);
  GS_EXPECT_FALSE(out.faulted);
  GS_EXPECT_EQ(2, motor.odometer);
}

static void test_motor_respects_rates_at_one_millisecond_service(void) {
  fake_bridge fake = {0};
  const gs_bridge_port port = {&fake, fake_disable, fake_apply};
  gs_motor_controller motor;
  gs_motor_output out = {0};

  gs_motor_init(&motor, port, 0);
  for (uint32_t now = 1u; now <= 100u; ++now) {
    out = gs_motor_step(&motor, &(gs_motor_input){2, false, 0, true, 250, now});
  }
  GS_EXPECT_EQ(40, out.demand.logical_command);

  for (uint32_t now = 101u; now <= 625u; ++now) {
    out = gs_motor_step(&motor, &(gs_motor_input){2, false, 0, true, 250, now});
  }
  GS_EXPECT_EQ(250, out.demand.logical_command);

  for (uint32_t now = 626u; now <= 725u; ++now) {
    out = gs_motor_step(&motor, &(gs_motor_input){2, false, 0, true, 0, now});
  }
  GS_EXPECT_EQ(170, out.demand.logical_command);
}

static void test_startup_timeout_begins_after_bridge_activation(void) {
  fake_bridge fake = {0};
  const gs_bridge_port port = {&fake, fake_disable, fake_apply};
  gs_motor_controller motor;
  gs_safety_supervisor safety;
  const gs_safety_sample sample = safe_sample(2000);

  gs_motor_init(&motor, port, 0);
  gs_safety_init(&safety, GS_SAFETY_MASTER, false, 0);
  calibrate_adc(&safety, 2000);
  gs_safety_set_enabled(&safety, true);
  for (uint32_t now = 1u; now <= 800u; ++now) {
    gs_safety_note_command(&safety, now);
    gs_safety_note_demand(&safety, gs_motor_bridge_active(&motor), now);
    gs_safety_evaluate(&safety, &sample, now);
    (void)gs_motor_step(&motor, &(gs_motor_input){2, false, 0, true, 250, now});
  }
  GS_EXPECT_TRUE(gs_motor_bridge_active(&motor));
  GS_EXPECT_EQ(126, safety.motion_start_ms);
  GS_EXPECT_EQ(GS_FAULT_NONE, safety.faults.bits);

  for (uint32_t now = 801u; now <= 827u; ++now) {
    gs_safety_note_command(&safety, now);
    gs_safety_note_demand(&safety, gs_motor_bridge_active(&motor), now);
    gs_safety_evaluate(&safety, &sample, now);
  }
  GS_EXPECT_TRUE((safety.faults.bits & GS_FAULT_STARTUP_TIMEOUT) != 0u);
}

void gs_test_control(void) {
  test_wheel_coordination();
  test_commutation_and_hall_sequences();
  test_symmetric_reverse_profile_opposes_each_forward_vector();
  test_proven_bridge_and_power_profiles();
  test_console_strict_state_transitions();
  test_console_ramp_waits_for_motion_ready();
  test_safety_zero_startup_timeouts_and_inputs();
  test_safety_adc_watchdog_latching_and_clear();
  test_motor_uses_configured_symmetric_reverse_profile();
  test_motor_ramps_limits_stops_and_reverses();
  test_motor_hall_faults_and_immediate_off();
  test_motor_reanchors_hall_after_coast_stop();
  test_motor_respects_rates_at_one_millisecond_service();
  test_startup_timeout_begins_after_bridge_activation();
}
