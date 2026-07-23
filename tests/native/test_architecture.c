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

static void establish_ready(gs_master_coordinator *master,
                            gs_slave_coordinator *slave, uint16_t sequence,
                            uint16_t enable_epoch, uint32_t now_ms) {
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  const gs_esp_command ready = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = sequence,
      .enable_epoch = enable_epoch,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(esp_frame, &ready));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(master, esp_frame, now_ms));
  GS_EXPECT_TRUE(gs_master_make_slave_frame(master, slave_frame));
  GS_EXPECT_TRUE(
      gs_slave_accept_master_frame(slave, slave_frame, now_ms + 1u));
  exchange_slave_feedback(master, slave, now_ms + 2u);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master->state);
  GS_EXPECT_EQ(GS_CONTROLLER_READY, slave->state);
  GS_EXPECT_EQ(enable_epoch, master->enable_epoch);
  GS_EXPECT_EQ(enable_epoch, slave->enable_epoch);
  GS_EXPECT_TRUE(gs_master_peer_healthy(master, now_ms + 2u));
}

static void test_zero_ready_then_sequence_acknowledged_motion(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  uint8_t master_feedback_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_master_feedback combined = {0};

  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);
  establish_ready(&master, &slave, 1u, 7u, 1u);

  const gs_esp_command move = {
      .speed = 500,
      .steer = 100,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 2u,
      .enable_epoch = 7u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(esp_frame, &move));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4u));
  GS_EXPECT_EQ(500, master.demanded.left);
  GS_EXPECT_EQ(100, master.demanded.right);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 5u));
  GS_EXPECT_EQ(-100, slave.demanded_electrical);
  slave.applied_electrical = slave.demanded_electrical;
  exchange_slave_feedback(&master, &slave, 6u);

  GS_EXPECT_TRUE(gs_master_make_feedback(&master, master_feedback_frame, 7u));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&combined, master_feedback_frame));
  GS_EXPECT_EQ(GS_PROTOCOL_VERSION, combined.protocol_version);
  GS_EXPECT_EQ(2, combined.accepted_esp_sequence);
  GS_EXPECT_EQ(2, combined.forwarded_slave_sequence);
  GS_EXPECT_EQ(2, combined.accepted_slave_sequence);
  GS_EXPECT_EQ(7, combined.master_enable_epoch);
  GS_EXPECT_EQ(7, combined.slave_enable_epoch);
  GS_EXPECT_EQ(100, combined.right_applied);
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
      .enable_epoch = 1u,
  };

  gs_master_init(&master, 0u);
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &move));
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 1u));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, master.state);
  GS_EXPECT_EQ(0, master.demanded.left);
  GS_EXPECT_EQ(0, master.demanded.right);
  GS_EXPECT_EQ(1, master.invalid_esp_frames);
}

static void test_ack_mismatch_has_bounded_grace(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];

  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);
  establish_ready(&master, &slave, 1u, 1u, 1u);

  const gs_esp_command move = {
      .speed = 100,
      .steer = 100,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 2u,
      .enable_epoch = 1u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(esp_frame, &move));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4u));
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, master.state);
  gs_master_tick(&master, 104u);
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, master.state);
  gs_master_tick(&master, 105u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, master.state);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
}

static void test_slave_feedback_loss_and_fault_stop_master(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];

  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);
  establish_ready(&master, &slave, 1u, 3u, 1u);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.speed = 100,
                                  .steer = 100,
                                  .master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 2u,
                                  .enable_epoch = 3u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4u));
  gs_master_tick(&master, 104u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, master.state);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
  GS_EXPECT_EQ(1, master.fault_epoch);
  GS_EXPECT_EQ(GS_FAULT_MASTER_LINK_TIMEOUT, master.first_fault);

  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);
  establish_ready(&master, &slave, 10u, 4u, 1u);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.speed = 100,
                                  .steer = 100,
                                  .master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 11u,
                                  .enable_epoch = 4u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 4u));
  gs_slave_latch_fault(&slave, GS_FAULT_STALL);
  exchange_slave_feedback(&master, &slave, 5u);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, master.state);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_STALL) != 0u);
  GS_EXPECT_EQ(GS_FAULT_STALL, master.first_fault);
}

static void test_duplicate_sequences_are_idempotent(void) {
  gs_master_coordinator master;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  gs_master_init(&master, 0u);
  gs_esp_command command = {
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 25u,
      .enable_epoch = 9u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 1u));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 2u));
  GS_EXPECT_EQ(1, master.valid_esp_frames);
  GS_EXPECT_EQ(2, master.last_esp_command_ms);

  command.enable_epoch = 10u;
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 3u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master.state);
  GS_EXPECT_EQ(1, master.invalid_esp_frames);
}

static void test_deadband_normalized_before_state_and_transport(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  gs_slave_command command = {0};

  GS_EXPECT_EQ(0, gs_normalize_wheel_command(49));
  GS_EXPECT_EQ(0, gs_normalize_wheel_command(-49));
  GS_EXPECT_EQ(50, gs_normalize_wheel_command(50));
  GS_EXPECT_EQ(-50, gs_normalize_wheel_command(-50));

  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){.speed = 49,
                                  .steer = -49,
                                  .master_flags = GS_COMMAND_DIRECT_LR,
                                  .sequence = 1u,
                                  .enable_epoch = 1u}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master.state);
  GS_EXPECT_EQ(0, master.demanded.left);
  GS_EXPECT_EQ(0, master.demanded.right);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_decode_slave_command(&command, slave_frame));
  GS_EXPECT_EQ(0, command.electrical_command);
  GS_EXPECT_EQ(1, command.enable_epoch);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 2u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, slave.state);
}

static void test_fault_epochs_and_explicit_recovery(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];

  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);
  establish_ready(&master, &slave, 1u, 10u, 1u);
  gs_master_latch_fault(&master, GS_FAULT_STALL);
  gs_slave_latch_fault(&slave, GS_FAULT_STALL);
  GS_EXPECT_EQ(1, master.fault_epoch);
  GS_EXPECT_EQ(1, slave.fault_epoch);

  const gs_esp_command stale_clear = {
      .master_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .sequence = 2u,
      .master_clear_fault_epoch = 0u,
      .slave_clear_fault_epoch = 0u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &stale_clear));
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 4u));
  GS_EXPECT_EQ(GS_CLEAR_REJECT_STALE_EPOCH, master.last_clear_result);

  const gs_esp_command clear = {
      .master_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .sequence = 3u,
      .master_clear_fault_epoch = 1u,
      .slave_clear_fault_epoch = 1u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &clear));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 5u));
  GS_EXPECT_TRUE(gs_master_fault_clear_requested(&master));
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 6u));
  GS_EXPECT_TRUE(gs_slave_fault_clear_requested(&slave));
  gs_master_finish_fault_clear(&master, GS_CLEAR_OK);
  gs_slave_finish_fault_clear(&slave, GS_CLEAR_OK);
  GS_EXPECT_EQ(0, master.faults);
  GS_EXPECT_EQ(0, slave.faults);
  GS_EXPECT_TRUE(master.recovery_required);
  GS_EXPECT_TRUE(slave.recovery_required);

  const gs_esp_command old_session = {
      .speed = 100,
      .steer = 100,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 4u,
      .enable_epoch = 10u,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &old_session));
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 7u));

  establish_ready(&master, &slave, 5u, 11u, 8u);
  GS_EXPECT_FALSE(master.recovery_required);
  GS_EXPECT_FALSE(slave.recovery_required);
}

void gs_test_architecture(void) {
  test_zero_ready_then_sequence_acknowledged_motion();
  test_motion_rejected_until_zero_ready_ack();
  test_ack_mismatch_has_bounded_grace();
  test_slave_feedback_loss_and_fault_stop_master();
  test_duplicate_sequences_are_idempotent();
  test_deadband_normalized_before_state_and_transport();
  test_fault_epochs_and_explicit_recovery();
}
