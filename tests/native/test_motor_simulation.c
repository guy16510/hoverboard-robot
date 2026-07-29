/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include <stdbool.h>
#include <stdint.h>

#include "gs_frame_parser.h"
#include "gs_hall_qualifier.h"
#include "gs_motor_control.h"
#include "gs_safety.h"
#include "gs_wheel_mix.h"

typedef enum {
  GS_SIM_MASTER = 0,
  GS_SIM_SLAVE,
} gs_motor_sim_role;

typedef struct {
  gs_commutation_vector vector;
  bool enabled;
} gs_motor_sim_bridge;

typedef struct {
  gs_motor_controller motor;
  gs_safety_supervisor safety;
  gs_motor_sim_bridge bridge;
  gs_motor_sim_role role;
  uint8_t hall;
  uint32_t next_transition_ms;
  uint32_t legal_transitions;
  gs_motor_output last_output;
} gs_motor_sim;

static void sim_bridge_disable(void *context) {
  gs_motor_sim_bridge *bridge = context;
  bridge->enabled = false;
}

static bool sim_bridge_apply(void *context, const gs_commutation_vector *vector,
                             uint16_t compare_offset) {
  gs_motor_sim_bridge *bridge = context;
  bridge->vector = *vector;
  bridge->enabled = compare_offset != 0u;
  return true;
}

static gs_bridge_port sim_bridge_port(gs_motor_sim_bridge *bridge) {
  const gs_bridge_port port = {
      .context = bridge,
      .disable = sim_bridge_disable,
      .apply = sim_bridge_apply,
  };
  return port;
}

static void sim_init(gs_motor_sim *sim, gs_motor_sim_role role,
                     uint8_t starting_hall) {
  *sim = (gs_motor_sim){0};
  sim->role = role;
  sim->hall = starting_hall;
  if (role == GS_SIM_MASTER) {
    gs_motor_init(&sim->motor, sim_bridge_port(&sim->bridge), 0u);
  } else {
    gs_motor_init_profile(&sim->motor, sim_bridge_port(&sim->bridge),
                          GS_COMMUTATION_SYMMETRIC_REVERSE, 0u);
  }
  gs_safety_init(&sim->safety,
                 role == GS_SIM_MASTER ? GS_SAFETY_MASTER : GS_SAFETY_SLAVE,
                 false, 0u);
  for (uint8_t sample = 0u; sample < GS_ADC_CALIBRATION_SAMPLES; ++sample) {
    gs_safety_sample_adc_off(&sim->safety, 2048u);
  }
  gs_safety_set_enabled(&sim->safety, true);
}

static int16_t electrical_command(gs_motor_sim_role role,
                                  int16_t logical_command) {
  return role == GS_SIM_MASTER ? logical_command
                               : gs_slave_electrical_command(logical_command);
}

static int32_t logical_odometer(const gs_motor_sim *sim) {
  return sim->role == GS_SIM_MASTER
             ? sim->motor.odometer
             : gs_slave_logical_odometer(sim->motor.odometer);
}

static uint8_t next_hall(uint8_t hall, int8_t direction) {
  static const uint8_t forward[8] = {0u, 5u, 3u, 1u, 6u, 4u, 2u, 0u};
  static const uint8_t reverse[8] = {0u, 3u, 6u, 2u, 5u, 1u, 4u, 0u};
  return direction > 0 ? forward[hall] : reverse[hall];
}

static bool same_vector(const gs_commutation_vector *first,
                        const gs_commutation_vector *second) {
  return first->source == second->source && first->sink == second->sink &&
         first->floating == second->floating;
}

static bool sim_bridge_produces_torque(const gs_motor_sim *sim,
                                       int8_t direction) {
  gs_commutation_vector forward = {0};
  if (!sim->bridge.enabled ||
      !gs_commutation_for_hall_profile(
          sim->hall, 1, GS_COMMUTATION_PHASE_ADVANCED_REVERSE, &forward)) {
    return false;
  }
  const gs_commutation_vector expected = direction > 0
                                             ? forward
                                             : (gs_commutation_vector){
                                                   .source = forward.sink,
                                                   .sink = forward.source,
                                                   .floating = forward.floating,
                                               };
  return same_vector(&sim->bridge.vector, &expected);
}

static gs_motor_output sim_step_with_interval(gs_motor_sim *sim,
                                              int16_t logical_command,
                                              uint32_t now, bool hall_changed,
                                              uint32_t hall_interval_us) {
  const gs_safety_sample safe = {
      .pa4_high = true,
      .adc_valid = true,
      .adc_value = 2048u,
      .hall_valid = true,
  };
  gs_safety_note_command(&sim->safety, now);
  gs_safety_note_demand(&sim->safety, gs_motor_bridge_active(&sim->motor), now);
  gs_safety_evaluate(&sim->safety, &safe, now);

  const bool permitted = sim->safety.enabled && sim->safety.adc_ready &&
                         sim->safety.faults.bits == GS_FAULT_NONE;
  const gs_motor_input input = {
      .hall = sim->hall,
      .hall_changed = hall_changed,
      .hall_interval_us = hall_changed ? hall_interval_us : 0u,
      .motion_permitted = permitted,
      .requested_command = electrical_command(sim->role, logical_command),
      .now_ms = now,
  };
  sim->last_output = gs_motor_step(&sim->motor, &input);
  if (sim->last_output.hall_result == GS_HALL_LEGAL) {
    gs_safety_note_hall(&sim->safety, now);
    ++sim->legal_transitions;
  }
  return sim->last_output;
}

static gs_motor_output sim_step(gs_motor_sim *sim, int16_t logical_command,
                                uint32_t now, bool hall_changed) {
  return sim_step_with_interval(sim, logical_command, now, hall_changed,
                                100000u);
}

static uint32_t sim_run(gs_motor_sim *sim, int16_t logical_command,
                        uint32_t first_ms, uint32_t last_ms,
                        bool allow_rotation) {
  const int16_t command = electrical_command(sim->role, logical_command);
  const int8_t direction = command > 0 ? 1 : -1;

  for (uint32_t now = first_ms; now <= last_ms; ++now) {
    bool hall_changed = false;
    if (allow_rotation && sim_bridge_produces_torque(sim, direction)) {
      if (sim->next_transition_ms == 0u) {
        sim->next_transition_ms = now + 50u;
      }
      if (now >= sim->next_transition_ms) {
        sim->hall = next_hall(sim->hall, direction);
        sim->next_transition_ms = now + 100u;
        hall_changed = true;
      }
    } else {
      sim->next_transition_ms = 0u;
    }
    (void)sim_step(sim, logical_command, now, hall_changed);
  }
  return sim->safety.faults.bits;
}

static void test_master_reverse_first_transition_exits_startup(void) {
  gs_motor_sim sim;
  sim_init(&sim, GS_SIM_MASTER, 1u);

  const uint32_t faults = sim_run(&sim, -250, 1u, 300u, true);

  GS_EXPECT_EQ(GS_FAULT_NONE, faults);
  GS_EXPECT_TRUE(sim.safety.hall_seen);
  GS_EXPECT_TRUE(sim.legal_transitions > 0u);
  GS_EXPECT_EQ(GS_MOTOR_RUNNING, sim.motor.state);
}

static void
test_every_role_direction_and_starting_hall_sustains_rotation(void) {
  static const uint8_t valid_halls[] = {1u, 3u, 2u, 6u, 4u, 5u};
  static const int16_t commands[] = {250, -250};

  for (gs_motor_sim_role role = GS_SIM_MASTER; role <= GS_SIM_SLAVE; ++role) {
    for (size_t direction = 0u;
         direction < sizeof(commands) / sizeof(commands[0]); ++direction) {
      for (size_t hall = 0u;
           hall < sizeof(valid_halls) / sizeof(valid_halls[0]); ++hall) {
        gs_motor_sim sim;
        sim_init(&sim, role, valid_halls[hall]);

        const uint32_t faults =
            sim_run(&sim, commands[direction], 1u, 1200u, true);

        GS_EXPECT_EQ(GS_FAULT_NONE, faults);
        GS_EXPECT_EQ(GS_MOTOR_RUNNING, sim.motor.state);
        GS_EXPECT_TRUE(sim.legal_transitions >= 10u);
        GS_EXPECT_TRUE(commands[direction] > 0 ? logical_odometer(&sim) > 0
                                               : logical_odometer(&sim) < 0);
      }
    }
  }
}

static void
test_stationary_startup_waits_for_bridge_before_first_transition(void) {
  gs_motor_sim sim;
  sim_init(&sim, GS_SIM_MASTER, 6u);

  GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&sim, 250, 1u, 100u, true));
  GS_EXPECT_FALSE(gs_motor_bridge_active(&sim.motor));
  GS_EXPECT_EQ(0, sim.legal_transitions);
  GS_EXPECT_EQ(6, sim.hall);

  GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&sim, 250, 101u, 300u, true));
  GS_EXPECT_TRUE(gs_motor_bridge_active(&sim.motor));
  GS_EXPECT_TRUE(sim.legal_transitions > 0u);
}

static void test_first_valid_reverse_transition_prevents_startup_timeout(void) {
  gs_motor_sim sim;
  sim_init(&sim, GS_SIM_MASTER, 4u);

  GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&sim, -250, 1u, 200u, true));
  GS_EXPECT_TRUE(sim.safety.hall_seen);
  GS_EXPECT_TRUE(sim.safety.last_hall_ms > sim.safety.motion_start_ms);

  const uint32_t faults = sim_run(&sim, -250, 201u, 900u, false);
  GS_EXPECT_EQ(0u, faults & GS_FAULT_STARTUP_TIMEOUT);
  GS_EXPECT_TRUE((faults & GS_FAULT_STALL) != 0u);
}

static void test_invalid_and_skipped_hall_transitions_fault_and_disable(void) {
  gs_motor_sim invalid;
  sim_init(&invalid, GS_SIM_MASTER, 2u);
  (void)sim_run(&invalid, 250, 1u, 200u, false);
  invalid.hall = 0u;
  const gs_motor_output invalid_output = sim_step(&invalid, 250, 201u, true);
  GS_EXPECT_TRUE(invalid_output.faulted);
  GS_EXPECT_EQ(GS_HALL_INVALID, invalid_output.hall_result);
  GS_EXPECT_FALSE(invalid_output.demand.bridge_enabled);

  gs_motor_sim skipped;
  sim_init(&skipped, GS_SIM_MASTER, 2u);
  (void)sim_run(&skipped, 250, 1u, 200u, false);
  skipped.hall = 1u;
  const gs_motor_output skipped_output = sim_step(&skipped, 250, 201u, true);
  GS_EXPECT_TRUE(skipped_output.faulted);
  GS_EXPECT_EQ(GS_HALL_ILLEGAL, skipped_output.hall_result);
  GS_EXPECT_FALSE(skipped_output.demand.bridge_enabled);
}

static void test_genuinely_stalled_motor_reports_startup_timeout(void) {
  gs_motor_sim sim;
  sim_init(&sim, GS_SIM_MASTER, 5u);

  const uint32_t faults = sim_run(&sim, -250, 1u, 900u, false);

  GS_EXPECT_EQ(GS_FAULT_STARTUP_TIMEOUT, faults);
  GS_EXPECT_FALSE(gs_motor_bridge_active(&sim.motor));
  GS_EXPECT_FALSE(sim.safety.hall_seen);
}

static void test_slave_direction_inversion_is_applied_exactly_once(void) {
  GS_EXPECT_EQ(-250, electrical_command(GS_SIM_SLAVE, 250));
  GS_EXPECT_EQ(250, electrical_command(GS_SIM_SLAVE, -250));
  GS_EXPECT_EQ(250, electrical_command(GS_SIM_MASTER, 250));
  GS_EXPECT_EQ(-250, electrical_command(GS_SIM_MASTER, -250));

  gs_motor_sim forward;
  sim_init(&forward, GS_SIM_SLAVE, 1u);
  GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&forward, 250, 1u, 400u, true));
  GS_EXPECT_EQ(-1, forward.motor.direction);
  GS_EXPECT_TRUE(forward.motor.odometer < 0);
  GS_EXPECT_TRUE(logical_odometer(&forward) > 0);

  gs_motor_sim reverse;
  sim_init(&reverse, GS_SIM_SLAVE, 1u);
  GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&reverse, -250, 1u, 400u, true));
  GS_EXPECT_EQ(1, reverse.motor.direction);
  GS_EXPECT_TRUE(reverse.motor.odometer > 0);
  GS_EXPECT_TRUE(logical_odometer(&reverse) < 0);
}

static uint32_t fuzz_random(uint32_t *state) {
  *state ^= *state << 13u;
  *state ^= *state >> 17u;
  *state ^= *state << 5u;
  return *state;
}

static void test_seeded_motor_and_transport_noise_fails_closed(void) {
  static const uint8_t valid_halls[] = {1u, 2u, 3u, 4u, 5u, 6u};
  uint32_t random = 0x91e10da5u;

  for (uint32_t iteration = 0u; iteration < 120u; ++iteration) {
    const uint32_t choice = fuzz_random(&random);
    const gs_motor_sim_role role =
        (choice & 1u) != 0u ? GS_SIM_MASTER : GS_SIM_SLAVE;
    const int16_t command = (choice & 2u) != 0u ? 250 : -250;
    gs_motor_sim sim;
    sim_init(&sim, role,
             valid_halls[fuzz_random(&random) %
                         (sizeof(valid_halls) / sizeof(valid_halls[0]))]);
    GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&sim, command, 1u, 600u, true));
    GS_EXPECT_TRUE(gs_motor_bridge_active(&sim.motor));

    const int8_t direction = sim.motor.direction;
    const int32_t odometer = sim.motor.odometer;
    const int16_t applied = sim.motor.applied_command;
    const uint8_t qualified_hall = sim.hall;
    const uint32_t scenario = choice % 10u;

    if (scenario == 0u) {
      gs_hall_qualifier qualifier;
      gs_qualified_hall_event event = {0};
      const uint8_t noisy_hall = (uint8_t)(qualified_hall % 6u + 1u);
      const uint32_t width_us = fuzz_random(&random) % GS_HALL_STABILITY_US;
      gs_hall_qualifier_init(&qualifier, qualified_hall, 1000u);
      GS_EXPECT_FALSE(
          gs_hall_qualifier_update(&qualifier, noisy_hall, 1500u, &event));
      GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, qualified_hall,
                                               1500u + width_us, &event));
      GS_EXPECT_EQ(1u, gs_hall_qualifier_glitches(&qualifier));
      GS_EXPECT_EQ(qualified_hall, gs_hall_qualifier_value(&qualifier));
      GS_EXPECT_EQ(direction, sim.motor.direction);
      GS_EXPECT_EQ(odometer, sim.motor.odometer);
      GS_EXPECT_EQ(applied, sim.motor.applied_command);
      GS_EXPECT_TRUE(gs_motor_bridge_active(&sim.motor));
    } else if (scenario >= 1u && scenario <= 4u) {
      gs_hall_qualifier qualifier;
      gs_qualified_hall_event event = {0};
      uint8_t candidate = 0u;
      uint32_t edge_us = 1600u;
      if (scenario == 2u) {
        candidate = next_hall(next_hall(qualified_hall, direction), direction);
      } else if (scenario == 3u) {
        candidate = next_hall(qualified_hall, (int8_t)-direction);
      } else if (scenario == 4u) {
        candidate = next_hall(qualified_hall, direction);
        edge_us = 1499u;
      }
      gs_hall_qualifier_init(&qualifier, qualified_hall, 1000u);
      GS_EXPECT_FALSE(
          gs_hall_qualifier_update(&qualifier, candidate, edge_us, &event));
      GS_EXPECT_TRUE(gs_hall_qualifier_update(
          &qualifier, candidate, edge_us + GS_HALL_STABILITY_US, &event));
      sim.hall = event.hall;
      const gs_motor_output output =
          sim_step_with_interval(&sim, command, 601u, true, event.interval_us);
      GS_EXPECT_TRUE(output.faulted);
      GS_EXPECT_FALSE(output.demand.bridge_enabled);
      GS_EXPECT_FALSE(gs_motor_bridge_active(&sim.motor));
    } else if (scenario == 5u || scenario == 6u) {
      const uint8_t marker[] = {GS_COMMAND_MARKER};
      uint8_t frame[GS_ESP_COMMAND_SIZE];
      uint8_t decoded[GS_MAX_FRAME_SIZE] = {0};
      gs_frame_parser parser;
      GS_EXPECT_TRUE(gs_encode_esp_command(
          frame, &(gs_esp_command){.speed = command, .sequence = 7u}));
      gs_frame_parser_init(&parser, marker, sizeof(marker), sizeof(frame));

      if (scenario == 5u) {
        frame[5] ^= 0x80u;
        gs_parse_result result = GS_PARSE_NONE;
        for (size_t index = 0u; index < sizeof(frame); ++index) {
          result = gs_frame_parser_feed(&parser, frame[index], index, decoded);
        }
        GS_EXPECT_EQ(GS_PARSE_BAD_CRC, result);
      } else {
        const size_t truncated = sizeof(frame) / 2u;
        for (size_t index = 0u; index < truncated; ++index) {
          (void)gs_frame_parser_feed(&parser, frame[index], index, decoded);
        }
        GS_EXPECT_EQ(
            GS_PARSE_TIMEOUT,
            gs_frame_parser_poll(&parser,
                                 truncated + GS_PARTIAL_FRAME_TIMEOUT_MS + 1u));
        for (size_t index = 0u; index < sizeof(frame); ++index) {
          uint8_t noise = (uint8_t)fuzz_random(&random);
          if (noise == GS_COMMAND_MARKER) {
            noise ^= 0x80u;
          }
          GS_EXPECT_EQ(GS_PARSE_NONE,
                       gs_frame_parser_feed(&parser, noise,
                                            1000u + (uint32_t)index, decoded));
        }
      }
      GS_EXPECT_EQ(direction, sim.motor.direction);
      GS_EXPECT_EQ(odometer, sim.motor.odometer);
      GS_EXPECT_EQ(applied, sim.motor.applied_command);
      GS_EXPECT_TRUE(gs_motor_bridge_active(&sim.motor));
    } else if (scenario == 7u) {
      const uint32_t faults = sim_run(&sim, command, 601u, 1000u, false);
      GS_EXPECT_TRUE((faults & GS_FAULT_STALL) != 0u);
      GS_EXPECT_FALSE(gs_motor_bridge_active(&sim.motor));
    } else if (scenario == 8u) {
      GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&sim, command, 601u, 1000u, true));
      GS_EXPECT_EQ(direction, sim.motor.direction);
      GS_EXPECT_TRUE(sim.motor.odometer != odometer);
      GS_EXPECT_TRUE(gs_motor_bridge_active(&sim.motor));
    } else {
      GS_EXPECT_EQ(GS_FAULT_NONE, sim_run(&sim, 0, 601u, 900u, false));
      GS_EXPECT_EQ(GS_FAULT_NONE,
                   sim_run(&sim, (int16_t)-command, 901u, 1700u, true));
      GS_EXPECT_EQ(-direction, sim.motor.direction);
      GS_EXPECT_TRUE(gs_motor_bridge_active(&sim.motor));
    }
  }
}

void gs_test_motor_simulation(void) {
  test_master_reverse_first_transition_exits_startup();
  test_every_role_direction_and_starting_hall_sustains_rotation();
  test_stationary_startup_waits_for_bridge_before_first_transition();
  test_first_valid_reverse_transition_prevents_startup_timeout();
  test_invalid_and_skipped_hall_transitions_fault_and_disable();
  test_genuinely_stalled_motor_reports_startup_timeout();
  test_slave_direction_inversion_is_applied_exactly_once();
  test_seeded_motor_and_transport_noise_fails_closed();
}
