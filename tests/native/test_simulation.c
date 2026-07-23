/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include <string.h>

#include "gausstop_bridge_profile.h"
#include "gs_master.h"
#include "gs_motor_control.h"
#include "gs_safety.h"
#include "gs_slave.h"
#include "gs_wheel_mix.h"

typedef struct {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  uint8_t slave_feedback[GS_SLAVE_FEEDBACK_SIZE];
} gs_robot_sim;

typedef struct {
  gs_bridge_drive_plan plan;
  uint32_t apply_count;
  uint32_t disable_count;
  bool enabled;
} gs_sim_bridge;

static void sim_bridge_disable(void *context) {
  gs_sim_bridge *bridge = context;
  bridge->enabled = false;
  ++bridge->disable_count;
}

static bool sim_bridge_apply(void *context, const gs_commutation_vector *vector,
                             uint16_t compare_offset) {
  gs_sim_bridge *bridge = context;
  const gs_motor_power_profile *profile = gs_motor_power_profile_current();
  if (!gs_bridge_make_drive_plan(vector, 500u, compare_offset,
                                 profile->maximum_compare, &bridge->plan)) {
    return false;
  }
  bridge->enabled = true;
  ++bridge->apply_count;
  return true;
}

static gs_bridge_port sim_bridge_port(gs_sim_bridge *bridge) {
  const gs_bridge_port port = {
      .context = bridge,
      .disable = sim_bridge_disable,
      .apply = sim_bridge_apply,
  };
  return port;
}

static void sim_safety_prepare(gs_safety_supervisor *safety,
                               gs_safety_role role) {
  gs_safety_init(safety, role, false, 0u);
  for (uint8_t sample = 0u; sample < GS_ADC_CALIBRATION_SAMPLES; ++sample) {
    gs_safety_sample_adc_off(safety, 2048u);
  }
  gs_safety_set_enabled(safety, true);
}

static void sim_init(gs_robot_sim *sim) {
  memset(sim, 0, sizeof(*sim));
  gs_master_init(&sim->master, 0u);
  gs_slave_init(&sim->slave, 0u);
}

static bool sim_send_esp(gs_robot_sim *sim, const gs_esp_command *command,
                         uint32_t now_ms) {
  return gs_encode_esp_command(sim->esp_frame, command) &&
         gs_master_accept_esp_frame(&sim->master, sim->esp_frame, now_ms);
}

static bool sim_forward_to_slave(gs_robot_sim *sim, uint32_t now_ms) {
  return gs_master_make_slave_frame(&sim->master, sim->slave_frame, now_ms) &&
         gs_slave_accept_master_frame(&sim->slave, sim->slave_frame, now_ms);
}

static bool sim_feedback_to_master(gs_robot_sim *sim, uint32_t now_ms) {
  return gs_slave_make_feedback(&sim->slave, sim->slave_feedback, now_ms) &&
         gs_master_accept_slave_feedback(&sim->master, sim->slave_feedback,
                                         now_ms);
}

static bool sim_exact_ack(gs_robot_sim *sim, uint32_t now_ms,
                          uint16_t expected_sequence) {
  uint8_t frame[GS_MASTER_FEEDBACK_SIZE];
  gs_master_feedback feedback = {0};
  return gs_master_make_feedback(&sim->master, frame, now_ms) &&
         gs_decode_master_feedback(&feedback, frame) &&
         feedback.accepted_esp_sequence == expected_sequence &&
         feedback.forwarded_slave_sequence == expected_sequence &&
         feedback.accepted_slave_sequence == expected_sequence;
}

static void sim_ready(gs_robot_sim *sim, uint16_t sequence) {
  const gs_esp_command ready = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = sequence,
  };
  GS_EXPECT_TRUE(sim_send_esp(sim, &ready, 1u));
  GS_EXPECT_TRUE(sim_forward_to_slave(sim, 2u));
  GS_EXPECT_TRUE(sim_feedback_to_master(sim, 3u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, sim->master.state);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, sim->slave.state);
}

static void sim_move(gs_robot_sim *sim, uint16_t sequence, int16_t left,
                     int16_t right) {
  const gs_esp_command move = {
      .speed = left,
      .steer = right,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = sequence,
  };
  GS_EXPECT_TRUE(sim_send_esp(sim, &move, 4u));
  GS_EXPECT_TRUE(sim_forward_to_slave(sim, 5u));
  sim->slave.applied_electrical = sim->slave.demanded_electrical;
  GS_EXPECT_TRUE(sim_feedback_to_master(sim, 6u));
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, sim->master.state);
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, sim->slave.state);
}

static void test_feedback_cannot_fake_command_acceptance(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 1u);

  for (uint32_t now = 20u; now <= 80u; now += 20u) {
    GS_EXPECT_TRUE(sim_feedback_to_master(&sim, now));
  }

  uint8_t combined_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_master_feedback combined = {0};
  GS_EXPECT_TRUE(gs_master_make_feedback(&sim.master, combined_frame, 81u));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&combined, combined_frame));
  GS_EXPECT_EQ(1, combined.accepted_esp_sequence);
  GS_EXPECT_EQ(1, combined.forwarded_slave_sequence);
  GS_EXPECT_EQ(1, combined.accepted_slave_sequence);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, combined.master_state);
  GS_EXPECT_EQ(0, combined.left_applied);
  GS_EXPECT_EQ(0, combined.right_applied);
}

static void test_slave_power_loss_stops_master(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 1u);
  sim_move(&sim, 2u, 200, 200);

  gs_master_tick(&sim.master, 107u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_EQ(0, sim.master.demanded.left);
  GS_EXPECT_EQ(0, sim.master.demanded.right);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);

  GS_EXPECT_TRUE(
      gs_master_make_slave_frame(&sim.master, sim.slave_frame, 108u));
  gs_slave_command stop = {0};
  GS_EXPECT_TRUE(gs_decode_slave_command(&stop, sim.slave_frame));
  GS_EXPECT_EQ(0, stop.electrical_command);
  GS_EXPECT_TRUE((stop.flags & GS_COMMAND_DISABLE) != 0u);
}

static void test_corruption_does_not_refresh_peer_age(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 10u);
  sim_move(&sim, 11u, 150, 150);

  GS_EXPECT_TRUE(gs_slave_make_feedback(&sim.slave, sim.slave_feedback, 50u));
  sim.slave_feedback[4] ^= 0x5Au;
  GS_EXPECT_FALSE(
      gs_master_accept_slave_feedback(&sim.master, sim.slave_feedback, 50u));
  GS_EXPECT_EQ(1, sim.master.invalid_slave_feedback_frames);
  GS_EXPECT_EQ(6, sim.master.last_slave_feedback_ms);

  gs_master_tick(&sim.master, 107u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
}

static void test_slave_timeout_propagates_to_master(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 20u);
  sim_move(&sim, 21u, 180, 180);

  gs_slave_tick(&sim.slave, 106u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.slave.state);
  GS_EXPECT_TRUE((sim.slave.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
  GS_EXPECT_TRUE(sim_feedback_to_master(&sim, 107u));
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
}

static void test_unexpected_slave_reset_stops_master(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 30u);
  sim_move(&sim, 31u, 220, 220);

  gs_slave_init(&sim.slave, 7u);
  GS_EXPECT_TRUE(sim_feedback_to_master(&sim, 8u));
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_EQ(0, sim.master.demanded.left);
  GS_EXPECT_EQ(0, sim.master.demanded.right);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
}

static void test_delayed_commands_and_sequence_wraparound(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 0xFFFFu);

  const gs_esp_command wrapped = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 1u,
  };
  GS_EXPECT_TRUE(sim_send_esp(&sim, &wrapped, 4u));
  GS_EXPECT_EQ(1, sim.master.last_esp_sequence);
  GS_EXPECT_EQ(1, sim.master.missing_esp_sequences);
  GS_EXPECT_TRUE(sim_forward_to_slave(&sim, 5u));
  GS_EXPECT_EQ(1, sim.slave.last_master_sequence);
  GS_EXPECT_EQ(1, sim.slave.missing_master_sequences);

  const gs_esp_command delayed = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 0xFFFFu,
  };
  GS_EXPECT_FALSE(sim_send_esp(&sim, &delayed, 6u));
  GS_EXPECT_EQ(1, sim.master.last_esp_sequence);

  gs_slave_command delayed_slave = {
      .sequence = 0xFFFFu,
  };
  GS_EXPECT_TRUE(gs_encode_slave_command(sim.slave_frame, &delayed_slave));
  GS_EXPECT_FALSE(
      gs_slave_accept_master_frame(&sim.slave, sim.slave_frame, 7u));
  GS_EXPECT_EQ(1, sim.slave.last_master_sequence);
}

static void test_corrupt_esp_command_never_acknowledged(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 40u);

  const gs_esp_command move = {
      .speed = 250,
      .steer = 250,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 41u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(sim.esp_frame, &move));
  sim.esp_frame[5] ^= 0x80u;
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&sim.master, sim.esp_frame, 4u));
  GS_EXPECT_EQ(40, sim.master.last_esp_sequence);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, sim.master.state);
  GS_EXPECT_EQ(0, sim.master.demanded.left);
  GS_EXPECT_EQ(0, sim.master.demanded.right);
}

static void test_scheduled_stop_and_wait_transport_converges(void) {
  gs_robot_sim sim;
  gs_command_sequencer sequencer;
  gs_esp_command desired = {
      .master_flags = GS_COMMAND_DIRECT_LR,
  };
  bool exact_ack = false;
  uint16_t feedback_ack_sequence = 0u;
  sim_init(&sim);
  gs_command_sequencer_init(&sequencer);

  for (uint32_t now = 0u; now <= 1000u; now += 20u) {
    if (now >= 100u && desired.speed < 250) {
      desired.speed = (int16_t)(desired.speed + 10);
      desired.steer = desired.speed;
    }
    if (now % 60u == 0u && sequencer.sent) {
      if (sim_exact_ack(&sim, now, sequencer.in_flight.sequence)) {
        feedback_ack_sequence = sequencer.in_flight.sequence;
      }
    }
    exact_ack =
        sequencer.sent && feedback_ack_sequence == sequencer.in_flight.sequence;
    const gs_esp_command *selected =
        gs_command_sequencer_select(&sequencer, &desired, exact_ack, now);
    GS_EXPECT_TRUE(selected != NULL);
    GS_EXPECT_TRUE(sim_send_esp(&sim, selected, now));
    GS_EXPECT_TRUE(sim_forward_to_slave(&sim, now));
    sim.slave.applied_electrical = sim.slave.demanded_electrical;
    GS_EXPECT_TRUE(sim_feedback_to_master(&sim, now));
    gs_master_tick(&sim.master, now);
    gs_slave_tick(&sim.slave, now);
    GS_EXPECT_TRUE(sim.master.state != GS_CONTROLLER_FAULTED);
    GS_EXPECT_TRUE(sim.slave.state != GS_CONTROLLER_FAULTED);
    GS_EXPECT_FALSE(
        gs_command_sequencer_ack_expired(&sequencer, exact_ack, now, 200u));
  }

  GS_EXPECT_EQ(250, sequencer.in_flight.speed);
  GS_EXPECT_EQ(250, sim.master.demanded.left);
  GS_EXPECT_EQ(250, sim.master.demanded.right);
  GS_EXPECT_EQ(-250, sim.slave.demanded_electrical);
  GS_EXPECT_TRUE(sim_exact_ack(&sim, 1020u, sequencer.in_flight.sequence));
  GS_EXPECT_TRUE(sim.master.valid_esp_frames < 20u);
  GS_EXPECT_EQ(0, sim.master.missing_esp_sequences);
  GS_EXPECT_EQ(0, sim.slave.missing_master_sequences);

  desired = (gs_esp_command){
      .master_flags = GS_COMMAND_DISABLE,
      .slave_flags = GS_COMMAND_DISABLE,
  };
  const uint16_t moving_sequence = sequencer.in_flight.sequence;
  const gs_esp_command *disabled =
      gs_command_sequencer_select(&sequencer, &desired, false, 1021u);
  GS_EXPECT_TRUE(disabled != NULL);
  GS_EXPECT_TRUE(disabled->sequence != moving_sequence);
  GS_EXPECT_TRUE(sim_send_esp(&sim, disabled, 1021u));
  GS_EXPECT_TRUE(sim_forward_to_slave(&sim, 1021u));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, sim.master.state);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, sim.slave.state);
  GS_EXPECT_EQ(0, sim.master.demanded.left);
  GS_EXPECT_EQ(0, sim.slave.demanded_electrical);
}

static void
test_full_scale_command_drives_both_logical_odometers_forward(void) {
  static const uint8_t master_halls[] = {2u, 3u, 1u, 5u, 4u, 6u, 2u};
  static const uint8_t slave_halls[] = {2u, 6u, 4u, 5u, 1u, 3u, 2u};
  gs_robot_sim sim;
  gs_sim_bridge master_bridge = {0};
  gs_sim_bridge slave_bridge = {0};
  gs_motor_controller master_motor;
  gs_motor_controller slave_motor;
  gs_safety_supervisor master_safety;
  gs_safety_supervisor slave_safety;
  uint8_t master_hall_index = 0u;
  uint8_t slave_hall_index = 0u;
  sim_init(&sim);
  sim_ready(&sim, 1u);
  sim_move(&sim, 2u, 250, 250);
  gs_motor_init(&master_motor, sim_bridge_port(&master_bridge), 0u);
  gs_motor_init(&slave_motor, sim_bridge_port(&slave_bridge), 0u);
  sim_safety_prepare(&master_safety, GS_SAFETY_MASTER);
  sim_safety_prepare(&slave_safety, GS_SAFETY_SLAVE);

  for (uint32_t now = 0u; now <= 800u; now += 25u) {
    const bool hall_changed = now >= 200u && now <= 700u && now % 100u == 0u;
    if (hall_changed) {
      ++master_hall_index;
      ++slave_hall_index;
    }
    GS_EXPECT_TRUE(sim_send_esp(&sim, &sim.master.last_esp_command, now));
    GS_EXPECT_TRUE(sim_forward_to_slave(&sim, now));
    gs_safety_note_command(&master_safety, now);
    gs_safety_note_command(&slave_safety, now);
    gs_safety_note_demand(
        &master_safety,
        gs_normalize_wheel_command(sim.master.demanded.left) != 0, now);
    gs_safety_note_demand(
        &slave_safety,
        gs_normalize_wheel_command(sim.slave.demanded_electrical) != 0, now);

    const gs_motor_input master_input = {
        .hall = master_halls[master_hall_index],
        .hall_changed = hall_changed,
        .hall_interval_us = 100000u,
        .motion_permitted = gs_safety_bridge_allowed(&master_safety),
        .requested_command = sim.master.demanded.left,
        .now_ms = now,
    };
    const gs_motor_input slave_input = {
        .hall = slave_halls[slave_hall_index],
        .hall_changed = hall_changed,
        .hall_interval_us = 100000u,
        .motion_permitted = gs_safety_bridge_allowed(&slave_safety),
        .requested_command = sim.slave.demanded_electrical,
        .now_ms = now,
    };
    const gs_motor_output master_output =
        gs_motor_step(&master_motor, &master_input);
    const gs_motor_output slave_output =
        gs_motor_step(&slave_motor, &slave_input);
    GS_EXPECT_FALSE(master_output.faulted);
    GS_EXPECT_FALSE(slave_output.faulted);
    if (hall_changed) {
      GS_EXPECT_EQ(GS_HALL_LEGAL, master_output.hall_result);
      GS_EXPECT_EQ(GS_HALL_LEGAL, slave_output.hall_result);
      gs_safety_note_hall(&master_safety, now);
      gs_safety_note_hall(&slave_safety, now);
    }

    const gs_safety_sample safe_sample = {
        .pa4_high = true,
        .adc_valid = true,
        .adc_value = 2048u,
        .hall_valid = true,
    };
    gs_safety_evaluate(&master_safety, &safe_sample, now);
    gs_safety_evaluate(&slave_safety, &safe_sample, now);
    sim.master.applied.left = master_output.demand.logical_command;
    sim.master.local_odometer = master_motor.odometer;
    gs_master_set_motor_status(&sim.master, master_halls[master_hall_index],
                               master_output.demand.compare_offset,
                               master_output.demand.bridge_enabled);
    sim.slave.applied_electrical = slave_output.demand.logical_command;
    sim.slave.odometer = slave_motor.odometer;
    gs_slave_set_motor_status(&sim.slave, slave_halls[slave_hall_index],
                              slave_output.demand.compare_offset,
                              slave_output.demand.bridge_enabled, true);
    GS_EXPECT_TRUE(sim_feedback_to_master(&sim, now));
    gs_master_tick(&sim.master, now);
    gs_slave_tick(&sim.slave, now);
    GS_EXPECT_EQ(GS_FAULT_NONE, master_safety.faults.bits);
    GS_EXPECT_EQ(GS_FAULT_NONE, slave_safety.faults.bits);
    GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, sim.master.state);
    GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, sim.slave.state);
  }

  uint8_t feedback_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_master_feedback feedback = {0};
  GS_EXPECT_TRUE(gs_master_make_feedback(&sim.master, feedback_frame, 800u));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&feedback, feedback_frame));
  GS_EXPECT_EQ(6, feedback.left_odometer);
  GS_EXPECT_EQ(6, feedback.right_odometer);
  GS_EXPECT_EQ(100, feedback.left_compare_offset);
  GS_EXPECT_EQ(100, feedback.right_compare_offset);
  GS_EXPECT_TRUE((feedback.motor_status_flags &
                  GS_MASTER_MOTOR_LEFT_BRIDGE_ENABLED) != 0u);
  GS_EXPECT_TRUE((feedback.motor_status_flags &
                  GS_MASTER_MOTOR_RIGHT_BRIDGE_ENABLED) != 0u);
  GS_EXPECT_EQ(GS_BRIDGE_ALL_PHASES_MASK,
               master_bridge.plan.enabled_phase_mask);
  GS_EXPECT_EQ(GS_BRIDGE_ALL_PHASES_MASK, slave_bridge.plan.enabled_phase_mask);
  GS_EXPECT_TRUE(master_bridge.apply_count > 0u);
  GS_EXPECT_TRUE(slave_bridge.apply_count > 0u);
}

void gs_test_simulation(void) {
  test_feedback_cannot_fake_command_acceptance();
  test_slave_power_loss_stops_master();
  test_corruption_does_not_refresh_peer_age();
  test_slave_timeout_propagates_to_master();
  test_unexpected_slave_reset_stops_master();
  test_delayed_commands_and_sequence_wraparound();
  test_corrupt_esp_command_never_acknowledged();
  test_scheduled_stop_and_wait_transport_converges();
  test_full_scale_command_drives_both_logical_odometers_forward();
}
