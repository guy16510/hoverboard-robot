/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdint.h>

#include "gs_master.h"
#include "gs_safety.h"
#include "gs_slave.h"
#include "test_harness.h"

static void encode_esp(uint8_t frame[GS_ESP_COMMAND_SIZE], uint16_t sequence,
                       int16_t speed, int16_t steer, uint8_t master_flags,
                       uint8_t slave_flags) {
  const gs_esp_command command = {
      .speed = speed,
      .steer = steer,
      .master_flags = master_flags,
      .slave_flags = slave_flags,
      .sequence = sequence,
  };
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
}

static void encode_slave(uint8_t frame[GS_SLAVE_COMMAND_SIZE], uint16_t sequence,
                         int16_t electrical, uint8_t flags) {
  const gs_slave_command command = {
      .electrical_command = electrical,
      .flags = flags,
      .sequence = sequence,
  };
  GS_EXPECT_TRUE(gs_encode_slave_command(frame, &command));
}

static void test_master_accepts_safe_restart_after_timeout(void) {
  gs_master_coordinator master;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  gs_master_init(&master, 0u);

  encode_esp(frame, 500u, 0, 0, GS_COMMAND_DISABLE, GS_COMMAND_DISABLE);
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 10u));
  GS_EXPECT_EQ(500u, master.last_esp_sequence);

  master.state = GS_CONTROLLER_ACTIVE;
  master.requested = (gs_wheel_pair){100, 100};
  master.demanded = (gs_wheel_pair){100, 100};
  encode_esp(frame, 1u, 0, 0, GS_COMMAND_DISABLE, GS_COMMAND_DISABLE);
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(&master, frame, 100u));
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, master.state);

  GS_EXPECT_TRUE(gs_master_accept_esp_frame(
      &master, frame, 10u + GS_ESP_TIMEOUT_MS + 1u));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, master.state);
  GS_EXPECT_EQ(0, master.requested.left);
  GS_EXPECT_EQ(0, master.requested.right);
  GS_EXPECT_EQ(1u, master.last_esp_sequence);

  encode_esp(frame, 2u, 0, 0, GS_COMMAND_DIRECT_LR, 0u);
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(
      &master, frame, 10u + GS_ESP_TIMEOUT_MS + 2u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, master.state);
}

static void test_master_accepts_safe_restart_while_disabled(void) {
  gs_master_coordinator master;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  gs_master_init(&master, 0u);

  encode_esp(frame, 30000u, 0, 0, GS_COMMAND_DISABLE, GS_COMMAND_DISABLE);
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 1u));
  encode_esp(frame, 1u, 0, 0, GS_COMMAND_DISABLE, GS_COMMAND_DISABLE);
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, frame, 2u));
  GS_EXPECT_EQ(1u, master.last_esp_sequence);
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, master.state);
}

static void test_slave_accepts_safe_restart_after_timeout(void) {
  gs_slave_coordinator slave;
  uint8_t frame[GS_SLAVE_COMMAND_SIZE];
  gs_slave_init(&slave, 0u);

  encode_slave(frame, 500u, 100, 0u);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, frame, 10u));
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, slave.state);

  encode_slave(frame, 1u, 0, GS_COMMAND_DISABLE);
  GS_EXPECT_FALSE(gs_slave_accept_master_frame(&slave, frame, 50u));
  GS_EXPECT_EQ(GS_CONTROLLER_ACTIVE, slave.state);

  GS_EXPECT_TRUE(gs_slave_accept_master_frame(
      &slave, frame, 10u + GS_SLAVE_TIMEOUT_MS + 1u));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, slave.state);
  GS_EXPECT_EQ(0, slave.demanded_electrical);
  GS_EXPECT_EQ(1u, slave.last_master_sequence);

  encode_slave(frame, 2u, 0, 0u);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(
      &slave, frame, 10u + GS_SLAVE_TIMEOUT_MS + 2u));
  GS_EXPECT_EQ(GS_CONTROLLER_READY, slave.state);
}

static void test_stale_motion_is_never_a_resync(void) {
  gs_master_coordinator master;
  gs_slave_coordinator slave;
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_COMMAND_SIZE];
  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);

  encode_esp(esp_frame, 400u, 0, 0, GS_COMMAND_DISABLE, GS_COMMAND_DISABLE);
  GS_EXPECT_TRUE(gs_master_accept_esp_frame(&master, esp_frame, 1u));
  encode_esp(esp_frame, 1u, 100, 100, GS_COMMAND_DIRECT_LR, 0u);
  GS_EXPECT_FALSE(gs_master_accept_esp_frame(
      &master, esp_frame, GS_ESP_TIMEOUT_MS + 2u));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, master.state);

  encode_slave(slave_frame, 400u, 0, GS_COMMAND_DISABLE);
  GS_EXPECT_TRUE(gs_slave_accept_master_frame(&slave, slave_frame, 1u));
  encode_slave(slave_frame, 1u, 100, 0u);
  GS_EXPECT_FALSE(gs_slave_accept_master_frame(
      &slave, slave_frame, GS_SLAVE_TIMEOUT_MS + 2u));
  GS_EXPECT_EQ(GS_CONTROLLER_DISABLED, slave.state);
}

void gs_test_resync(void) {
  test_master_accepts_safe_restart_after_timeout();
  test_master_accepts_safe_restart_while_disabled();
  test_slave_accepts_safe_restart_after_timeout();
  test_stale_motion_is_never_a_resync();
}
