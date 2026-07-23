/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "gs_master.h"
#include "gs_safety.h"
#include "gs_slave.h"
#include "gs_wheel_mix.h"

static void exchange_slave_feedback(gs_master_coordinator *master,
                                    gs_slave_coordinator *slave,
                                    uint32_t now_ms) {
  uint8_t feedback[GS_SLAVE_FEEDBACK_SIZE];
  GS_EXPECT_TRUE(gs_slave_make_feedback(slave, feedback, now_ms));
  GS_EXPECT_TRUE(gs_master_accept_slave_feedback(master, feedback, now_ms));
}

static void test_zero_ready_then_sequence_acknowledged_motion(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  uint8_t master_feedback_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_master_feedback combined;

  gs_master_init(&master, 0);
  gs_slave_init(&slave, 0);

  const gs_esp_command ready = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 1u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(esp_frame, &ready));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master.state);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 2));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, slave.state);
  exchange_slave_feedback(&master, &slave, 3);
  GS_EXPECT_TRUE(gs_master_peer_healthy(&master, 3));

  const gs_esp_command move = {
      .speed = 500,
      .steer = 100,
      .sequence = 2u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(esp_frame, &move));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4));
  GS_EXPECT_EQ(600, master.demanded.left);
  GS_EXPECT_EQ(400, master.demanded.right);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 5));
  GS_EXPECT_EQ(-400, slave.demanded_electrical);
  slave.applied_electrical = slave.demanded_electrical;
  exchange_slave_feedback(&master, &slave, 6);

  GS_EXPECT_TRUE(gs_master_make_feedback(&master, master_feedback_frame, 7));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&combined, master_feedback_frame));
  GS_EXPECT_EQ(GS_PROTOCOL_VERSION, combined.protocol_version);
  GS_EXPECT_EQ(2, combined.accepted_esp_sequence);
  GS_EXPECT_EQ(2, combined.forwarded_slave_sequence);
  GS_EXPECT_EQ(2, combined.accepted_slave_sequence);
  GS_EXPECT_EQ(400, combined.right_applied);
  GS_EXPECT_TRUE((combined.status_flags & GS_FEEDBACK_PEER_HEALTHY) != 0u);
}

static void test_motion_rejected_until_zero_ready_ack(void) {
  gs_master_coordinator master;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  const gs_esp_command move = {
      .speed = 100,
      .steer = 100,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 1u,
  };

  gs_master_init(&master, 0);
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &move));
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 1));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, master.state);
  GS_EXPECT_EQ(0, master.demanded.left);
  GS_EXPECT_EQ(0, master.demanded.right);
  GS_EXPECT_EQ(1, master.invalid_esp_frames);
}

static void test_slave_feedback_loss_and_fault_stop_master(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];

  gs_master_init(&master, 0);
  gs_slave_init(&slave, 0);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 1u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 2));
  exchange_slave_feedback(&master, &slave, 3);

  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.speed = 100,
                                  .steer = 100,
                                  .master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 2u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4));
  gs_master_tick(&master, 104);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, master.state);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);

  gs_master_init(&master, 0);
  gs_slave_init(&slave, 0);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 10u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 2));
  exchange_slave_feedback(&master, &slave, 3);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.speed = 100,
                                  .steer = 100,
                                  .master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 11u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4));
  slave.faults = GS_FAULT_STALL;
  slave.state = GS_CONTROLLER_FAULTED;
  exchange_slave_feedback(&master, &slave, 5);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, master.state);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_STALL) != 0u);
}

static void test_duplicate_sequences_are_idempotent(void) {
  gs_master_coordinator master;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  gs_master_init(&master, 0);
  gs_esp_command command = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 25u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 1));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 2));
  GS_EXPECT_EQ(1, master.valid_esp_frames);
  GS_EXPECT_EQ(2, master.last_esp_command_ms);

  command.speed = 50;
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 3));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master.state);
  GS_EXPECT_EQ(1, master.invalid_esp_frames);
}

static void test_deadband_normalized_before_state_and_transport(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  gs_slave_command command;

  GS_EXPECT_EQ(0, gs_normalize_wheel_command(49));
  GS_EXPECT_EQ(0, gs_normalize_wheel_command(-49));
  GS_EXPECT_EQ(50, gs_normalize_wheel_command(50));
  GS_EXPECT_EQ(-50, gs_normalize_wheel_command(-50));

  gs_master_init(&master, 0);
  gs_slave_init(&slave, 0);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.speed = 49,
                                  .steer = -49,
                                  .master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 1u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master.state);
  GS_EXPECT_EQ(0, master.demanded.left);
  GS_EXPECT_EQ(0, master.demanded.right);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_decode_slave_command(&command, slave_frame));
  GS_EXPECT_EQ(0, command.electrical_command);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 2));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, slave.state);
}

void gs_test_architecture(void) {
  test_zero_ready_then_sequence_acknowledged_motion();
  test_motion_rejected_until_zero_ready_ack();
  test_slave_feedback_loss_and_fault_stop_master();
  test_duplicate_sequences_are_idempotent();
  test_deadband_normalized_before_state_and_transport();
}
