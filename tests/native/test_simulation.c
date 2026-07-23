/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include <string.h>

#include "gs_master.h"
#include "gs_safety.h"
#include "gs_slave.h"

typedef struct {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  uint8_t slave_feedback[GS_SLAVE_FEEDBACK_SIZE];
} gs_robot_sim;

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
  return gs_master_make_slave_frame(&sim->master, sim->slave_frame) &&
         gs_slave_accept_master_frame(&sim->slave, sim->slave_frame, now_ms);
}

static bool sim_feedback_to_master(gs_robot_sim *sim, uint32_t now_ms) {
  return gs_slave_make_feedback(&sim->slave, sim->slave_feedback, now_ms) &&
         gs_master_accept_slave_feedback(&sim->master, sim->slave_feedback,
                                         now_ms);
}

static void sim_ready(gs_robot_sim *sim, uint16_t sequence,
                      uint16_t enable_epoch) {
  const gs_esp_command ready = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = sequence,
      .enable_epoch = enable_epoch,
  };
  GS_EXPECT_TRUE(sim_send_esp(sim, &ready, 1u));
  GS_EXPECT_TRUE(sim_forward_to_slave(sim, 2u));
  GS_EXPECT_TRUE(sim_feedback_to_master(sim, 3u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, sim->master.state);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, sim->slave.state);
  GS_EXPECT_EQ(enable_epoch, sim->master.enable_epoch);
  GS_EXPECT_EQ(enable_epoch, sim->slave.enable_epoch);
}

static void sim_move(gs_robot_sim *sim, uint16_t sequence,
                     uint16_t enable_epoch, int16_t left, int16_t right) {
  const gs_esp_command move = {
      .speed = left,
      .steer = right,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = sequence,
      .enable_epoch = enable_epoch,
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
  sim_ready(&sim, 1u, 1u);

  for (uint32_t now = 20u; now <= 80u; now += 20u) {
    GS_EXPECT_TRUE(sim_feedback_to_master(&sim, now));
  }

  uint8_t combined_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_master_feedback combined = {0};
  GS_EXPECT_TRUE(
      gs_master_make_feedback(&sim.master, combined_frame, 81u));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&combined, combined_frame));
  GS_EXPECT_EQ(1, combined.accepted_esp_sequence);
  GS_EXPECT_EQ(1, combined.forwarded_slave_sequence);
  GS_EXPECT_EQ(1, combined.accepted_slave_sequence);
  GS_EXPECT_EQ(1, combined.master_enable_epoch);
  GS_EXPECT_EQ(1, combined.slave_enable_epoch);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, combined.master_state);
  GS_EXPECT_EQ(0, combined.left_applied);
  GS_EXPECT_EQ(0, combined.right_applied);
}

static void test_slave_power_loss_stops_master(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 1u, 2u);
  sim_move(&sim, 2u, 2u, 200, 200);

  gs_master_tick(&sim.master, 107u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_EQ(0, sim.master.demanded.left);
  GS_EXPECT_EQ(0, sim.master.demanded.right);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
  GS_EXPECT_EQ(1, sim.master.fault_epoch);

  GS_EXPECT_TRUE(gs_master_make_slave_frame(&sim.master, sim.slave_frame));
  gs_slave_command stop = {0};
  GS_EXPECT_TRUE(gs_decode_slave_command(&stop, sim.slave_frame));
  GS_EXPECT_EQ(0, stop.electrical_command);
  GS_EXPECT_TRUE((stop.flags & GS_COMMAND_DISABLE) != 0u);
}

static void test_corruption_does_not_refresh_peer_age(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 10u, 3u);
  sim_move(&sim, 11u, 3u, 150, 150);

  GS_EXPECT_TRUE(gs_slave_make_feedback(&sim.slave, sim.slave_feedback, 50u));
  sim.slave_feedback[8] ^= 0x5Au;
  GS_EXPECT_FALSE(gs_master_accept_slave_feedback(
      &sim.master, sim.slave_feedback, 50u));
  GS_EXPECT_EQ(1, sim.master.invalid_slave_feedback_frames);
  GS_EXPECT_EQ(6, sim.master.last_slave_feedback_ms);

  gs_master_tick(&sim.master, 107u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
}

static void test_slave_timeout_propagates_to_master(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 20u, 4u);
  sim_move(&sim, 21u, 4u, 180, 180);

  gs_slave_tick(&sim.slave, 106u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.slave.state);
  GS_EXPECT_TRUE((sim.slave.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
  GS_EXPECT_EQ(1, sim.slave.fault_epoch);
  GS_EXPECT_TRUE(sim_feedback_to_master(&sim, 107u));
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, sim.master.state);
  GS_EXPECT_TRUE((sim.master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
  GS_EXPECT_EQ(1, sim.master.fault_epoch);
}

static void test_unexpected_slave_reset_stops_master(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 30u, 5u);
  sim_move(&sim, 31u, 5u, 220, 220);

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
  sim_ready(&sim, 0xFFFFu, 6u);

  const gs_esp_command wrapped = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 1u,
      .enable_epoch = 6u,
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
      .enable_epoch = 6u,
  };
  GS_EXPECT_FALSE(sim_send_esp(&sim, &delayed, 6u));
  GS_EXPECT_EQ(1, sim.master.last_esp_sequence);

  gs_slave_command delayed_slave = {
      .sequence = 0xFFFFu,
      .enable_epoch = 6u,
  };
  GS_EXPECT_TRUE(gs_encode_slave_command(sim.slave_frame, &delayed_slave));
  GS_EXPECT_FALSE(
      gs_slave_accept_master_frame(&sim.slave, sim.slave_frame, 7u));
  GS_EXPECT_EQ(1, sim.slave.last_master_sequence);
}

static void test_corrupt_esp_command_never_acknowledged(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 40u, 7u);

  const gs_esp_command move = {
      .speed = 250,
      .steer = 250,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 41u,
      .enable_epoch = 7u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(sim.esp_frame, &move));
  sim.esp_frame[9] ^= 0x80u;
  GS_EXPECT_FALSE(
      gs_master_accept_esp_frame(&sim.master, sim.esp_frame, 4u));
  GS_EXPECT_EQ(40, sim.master.last_esp_sequence);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, sim.master.state);
  GS_EXPECT_EQ(0, sim.master.demanded.left);
  GS_EXPECT_EQ(0, sim.master.demanded.right);
}

static void test_stale_clear_and_old_enable_epoch_rejected(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  sim_ready(&sim, 50u, 8u);
  gs_master_latch_fault(&sim.master, GS_FAULT_STALL);
  gs_slave_latch_fault(&sim.slave, GS_FAULT_STALL);

  const gs_esp_command stale_clear = {
      .master_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .sequence = 51u,
      .master_clear_fault_epoch = 0u,
      .slave_clear_fault_epoch = 0u,
  };
  GS_EXPECT_FALSE(sim_send_esp(&sim, &stale_clear, 4u));
  GS_EXPECT_EQ(GS_CLEAR_REJECT_STALE_EPOCH,
               sim.master.last_clear_result);

  const gs_esp_command clear = {
      .master_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .sequence = 52u,
      .master_clear_fault_epoch = 1u,
      .slave_clear_fault_epoch = 1u,
  };
  GS_EXPECT_TRUE(sim_send_esp(&sim, &clear, 5u));
  GS_EXPECT_TRUE(sim_forward_to_slave(&sim, 6u));
  gs_master_finish_fault_clear(&sim.master, GS_CLEAR_OK);
  gs_slave_finish_fault_clear(&sim.slave, GS_CLEAR_OK);

  const gs_esp_command old_motion = {
      .speed = 100,
      .steer = 100,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 53u,
      .enable_epoch = 8u,
  };
  GS_EXPECT_FALSE(sim_send_esp(&sim, &old_motion, 7u));

  sim_ready(&sim, 54u, 9u);
  sim_move(&sim, 55u, 9u, 100, 100);
}

static void test_first_fault_is_stable_across_additional_faults(void) {
  gs_robot_sim sim;
  sim_init(&sim);
  gs_master_latch_fault(&sim.master, GS_FAULT_STALL);
  GS_EXPECT_EQ(GS_FAULT_STALL, sim.master.first_fault);
  GS_EXPECT_EQ(1, sim.master.fault_epoch);
  gs_master_latch_fault(&sim.master, GS_FAULT_PROTOCOL);
  GS_EXPECT_EQ(GS_FAULT_STALL, sim.master.first_fault);
  GS_EXPECT_EQ(2, sim.master.fault_epoch);
  gs_master_latch_fault(&sim.master, GS_FAULT_PROTOCOL);
  GS_EXPECT_EQ(2, sim.master.fault_epoch);
}

void gs_test_simulation(void) {
  test_feedback_cannot_fake_command_acceptance();
  test_slave_power_loss_stops_master();
  test_corruption_does_not_refresh_peer_age();
  test_slave_timeout_propagates_to_master();
  test_unexpected_slave_reset_stops_master();
  test_delayed_commands_and_sequence_wraparound();
  test_corrupt_esp_command_never_acknowledged();
  test_stale_clear_and_old_enable_epoch_rejected();
  test_first_fault_is_stable_across_additional_faults();
}
