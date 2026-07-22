/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "gs_master.h"
#include "gs_safety.h"
#include "gs_slave.h"
#include "gs_wheel_mix.h"

static void test_one_esp32_master_slave_flow(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  uint8_t slave_feedback_frame[GS_SLAVE_FEEDBACK_SIZE];
  uint8_t master_feedback_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_slave_command slave_command;
  gs_master_feedback combined;

  gs_master_init(&master, 0);
  gs_slave_init(&slave, 0);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, master.state);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, slave.state);

  GS_EXPECT_TRUE(
      gs_encode_esp_command(esp_frame, &(gs_esp_command){500, 100, 0, 0}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  GS_EXPECT_EQ(600, master.demanded.left);
  GS_EXPECT_EQ(400, master.demanded.right);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_decode_slave_command(&slave_command, slave_frame));
  GS_EXPECT_EQ(-400, slave_command.electrical_command);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 2));
  GS_EXPECT_EQ(-400, slave.demanded_electrical);

  master.local_odometer = 10;
  master.applied = master.demanded;
  slave.odometer = -12;
  slave.applied_electrical = slave.demanded_electrical;
  GS_EXPECT_TRUE(gs_slave_make_feedback(&slave, slave_feedback_frame));
  GS_EXPECT_TRUE(
      gs_master_accept_slave_feedback(&master, slave_feedback_frame));
  GS_EXPECT_TRUE(gs_master_make_feedback(&master, master_feedback_frame));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&combined, master_feedback_frame));
  GS_EXPECT_EQ(10, combined.left_odometer);
  GS_EXPECT_EQ(12, combined.right_odometer);
  GS_EXPECT_EQ(600, combined.left_applied);
  GS_EXPECT_EQ(400, combined.right_applied);
}

static void test_direct_lr_and_timeout_propagate_stop(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  gs_slave_command command;

  gs_master_init(&master, 0);
  gs_slave_init(&slave, 0);
  GS_EXPECT_TRUE(gs_encode_esp_command(
      esp_frame, &(gs_esp_command){-250, 750, GS_COMMAND_DIRECT_LR, 0}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  GS_EXPECT_EQ(-250, master.demanded.left);
  GS_EXPECT_EQ(750, master.demanded.right);
  gs_master_tick(&master, 402);
  GS_EXPECT_EQ(0, master.demanded.left);
  GS_EXPECT_EQ(0, master.demanded.right);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_COMMAND_TIMEOUT) != 0u);
  GS_EXPECT_TRUE(gs_master_make_slave_frame(&master, slave_frame));
  GS_EXPECT_TRUE(gs_decode_slave_command(&command, slave_frame));
  GS_EXPECT_EQ(0, command.electrical_command);
  GS_EXPECT_TRUE((command.flags & GS_COMMAND_DISABLE) != 0u);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 403));
  GS_EXPECT_EQ(0, slave.demanded_electrical);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, slave.state);
}

static void test_slave_stops_on_master_timeout(void) {
  gs_slave_coordinator slave;
  uint8_t frame[GS_SLAVE_COMMAND_SIZE];

  gs_slave_init(&slave, 0);
  GS_EXPECT_TRUE(gs_encode_slave_command(frame, &(gs_slave_command){100, 0}));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, frame, 1));
  gs_slave_tick(&slave, 102);
  GS_EXPECT_EQ(0, slave.demanded_electrical);
  GS_EXPECT_EQ(GS_CONTROLLER_FAULTED, slave.state);
  GS_EXPECT_TRUE((slave.faults & GS_FAULT_MASTER_LINK_TIMEOUT) != 0u);
}

static void test_bad_frames_stop_operational_roles(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];

  gs_master_init(&master, 0);
  GS_EXPECT_TRUE(
      gs_encode_esp_command(esp_frame, &(gs_esp_command){100, 0, 0, 0}));
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1));
  esp_frame[2] ^= 1u;
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, esp_frame, 2));
  GS_EXPECT_EQ(0, master.demanded.left);
  GS_EXPECT_TRUE((master.faults & GS_FAULT_PROTOCOL) != 0u);

  gs_slave_init(&slave, 0);
  GS_EXPECT_TRUE(
      gs_encode_slave_command(slave_frame, &(gs_slave_command){100, 0}));
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 1));
  slave_frame[1] ^= 1u;
  GS_EXPECT_FALSE(gs_slave_accept_master_frame(&slave, slave_frame, 2));
  GS_EXPECT_EQ(0, slave.demanded_electrical);
  GS_EXPECT_TRUE((slave.faults & GS_FAULT_PROTOCOL) != 0u);
}

void gs_test_architecture(void) {
  test_one_esp32_master_slave_flow();
  test_direct_lr_and_timeout_propagate_stop();
  test_slave_stops_on_master_timeout();
  test_bad_frames_stop_operational_roles();
}
