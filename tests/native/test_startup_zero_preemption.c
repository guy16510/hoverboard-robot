/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "gs_master.h"
#include "gs_protocol.h"
#include "gs_slave.h"

static gs_master_feedback
decode_master_feedback(const gs_master_coordinator *master, uint32_t now_ms) {
  uint8_t frame[GS_MASTER_FEEDBACK_SIZE] = {0};
  gs_master_feedback feedback = {0};
  GS_EXPECT_TRUE(gs_master_make_feedback(master, frame, now_ms));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&feedback, frame));
  return feedback;
}

static void deliver_to_master(gs_master_coordinator *master,
                              const gs_esp_command *command, uint32_t now_ms) {
  uint8_t frame[GS_ESP_COMMAND_SIZE] = {0};
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, command));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(master, frame, now_ms));
}

static void exchange_with_slave(gs_master_coordinator *master,
                                gs_slave_coordinator *slave, uint32_t now_ms) {
  uint8_t command_frame[GS_SLAVE_COMMAND_SIZE] = {0};
  uint8_t feedback_frame[GS_SLAVE_FEEDBACK_SIZE] = {0};
  GS_EXPECT_TRUE(gs_master_make_slave_frame(master, command_frame, now_ms));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(slave, command_frame, now_ms));
  GS_EXPECT_TRUE(gs_slave_make_feedback(slave, feedback_frame, now_ms + 1u));
  GS_EXPECT_TRUE(
      gs_master_accept_slave_feedback(master, feedback_frame, now_ms + 1u));
}

static void test_partial_disable_ack_recovers_to_ready_zero(void) {
  gs_command_sequencer sequencer;
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  gs_command_sequencer_init(&sequencer);
  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);

  gs_master_set_runtime_status(&master, true, true);
  gs_master_set_motor_status(&master, 6u, 0u, false);
  gs_slave_set_motor_status(&slave, 1u, 0u, false, true);

  const gs_esp_command disable = {
      .master_flags = GS_COMMAND_DISABLE,
      .slave_flags = GS_COMMAND_DISABLE,
  };
  const gs_esp_command *selected =
      gs_command_sequencer_select(&sequencer, &disable, false, 1u);
  GS_EXPECT_TRUE(selected != NULL);
  GS_EXPECT_EQ(1u, selected->sequence);
  deliver_to_master(&master, selected, 1u);

  uint8_t dropped_slave_frame[GS_SLAVE_COMMAND_SIZE] = {0};
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, dropped_slave_frame, 2u));

  const gs_master_feedback blocked = decode_master_feedback(&master, 3u);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, blocked.master_state);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, blocked.slave_state);
  GS_EXPECT_EQ(1u, blocked.accepted_esp_sequence);
  GS_EXPECT_EQ(1u, blocked.forwarded_slave_sequence);
  GS_EXPECT_EQ(0u, blocked.accepted_slave_sequence);
  GS_EXPECT_FALSE(gs_master_feedback_exact_ack(&blocked, 1u, true));
  GS_EXPECT_FALSE(gs_master_feedback_runtime_healthy(&blocked));
  GS_EXPECT_FALSE(gs_master_feedback_motion_ready(&blocked, 1u, true));
  GS_EXPECT_EQ(0u, blocked.status_flags & GS_FEEDBACK_PEER_HEALTHY);
  GS_EXPECT_EQ(0, blocked.left_applied);
  GS_EXPECT_EQ(0, blocked.right_applied);

  const gs_esp_command ready_zero = {
      .master_flags = GS_COMMAND_DIRECT_LR,
  };
  selected = gs_command_sequencer_select(&sequencer, &ready_zero, false, 4u);
  GS_EXPECT_TRUE(selected != NULL);
  GS_EXPECT_EQ(2u, selected->sequence);
  GS_EXPECT_EQ(0, selected->speed);
  GS_EXPECT_EQ(0, selected->steer);
  GS_EXPECT_EQ(GS_COMMAND_DIRECT_LR, selected->master_flags);
  GS_EXPECT_EQ(0u, selected->slave_flags);

  const gs_esp_command move = {
      .speed = 100,
      .steer = 100,
      .master_flags = GS_COMMAND_DIRECT_LR,
  };
  const gs_esp_command *blocked_move =
      gs_command_sequencer_select(&sequencer, &move, false, 5u);
  GS_EXPECT_TRUE(blocked_move != NULL);
  GS_EXPECT_EQ(2u, blocked_move->sequence);
  GS_EXPECT_EQ(0, blocked_move->speed);
  GS_EXPECT_EQ(0, blocked_move->steer);

  deliver_to_master(&master, selected, 6u);
  exchange_with_slave(&master, &slave, 7u);
  const gs_master_feedback recovered = decode_master_feedback(&master, 9u);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, recovered.master_state);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, recovered.slave_state);
  GS_EXPECT_TRUE((recovered.status_flags & GS_FEEDBACK_PEER_HEALTHY) != 0u);
  GS_EXPECT_TRUE(gs_master_feedback_exact_ack(&recovered, 2u, true));
  GS_EXPECT_TRUE(gs_master_feedback_runtime_healthy(&recovered));
  GS_EXPECT_TRUE(gs_master_feedback_motion_ready(&recovered, 2u, true));
  GS_EXPECT_EQ(0, recovered.left_applied);
  GS_EXPECT_EQ(0, recovered.right_applied);
  GS_EXPECT_EQ(0u, recovered.motor_status_flags &
                       (GS_MASTER_MOTOR_LEFT_BRIDGE_ENABLED |
                        GS_MASTER_MOTOR_RIGHT_BRIDGE_ENABLED));
}

static void test_fault_clear_disable_recovers_to_safe_ready_zero(void) {
  gs_command_sequencer sequencer;
  gs_command_sequencer_init(&sequencer);

  const gs_esp_command clear_disable = {
      .master_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
  };
  const gs_esp_command ready_zero = {
      .master_flags = GS_COMMAND_DIRECT_LR,
  };

  const gs_esp_command *selected =
      gs_command_sequencer_select(&sequencer, &clear_disable, false, 1u);
  GS_EXPECT_TRUE(selected != NULL);
  GS_EXPECT_EQ(1u, selected->sequence);

  selected = gs_command_sequencer_select(&sequencer, &ready_zero, false, 2u);
  GS_EXPECT_TRUE(selected != NULL);
  GS_EXPECT_EQ(2u, selected->sequence);
  GS_EXPECT_EQ(0, selected->speed);
  GS_EXPECT_EQ(0, selected->steer);
  GS_EXPECT_EQ(GS_COMMAND_DIRECT_LR, selected->master_flags);
  GS_EXPECT_EQ(0u, selected->slave_flags);
}

void gs_test_startup_zero_preemption(void) {
  test_partial_disable_ack_recovers_to_ready_zero();
  test_fault_clear_disable_recovers_to_safe_ready_zero();
}
