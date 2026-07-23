/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "gs_frame_parser.h"
#include "gs_protocol.h"

static void test_crc_vectors(void) {
  static const uint8_t check[] = "123456789";
  GS_EXPECT_EQ(0x31C3, gs_crc16(check, sizeof(check) - 1u));
}

static void test_exact_sizes_and_endianness(void) {
  const gs_esp_command command = {
      .speed = -1000,
      .steer = 1000,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .slave_flags = 0u,
      .sequence = 0x1234u,
  };
  const uint8_t prefix[] = {0x30, 0x34, 0x12, 0x18, 0xFC,
                            0xE8, 0x03, 0x20, 0x00};
  uint8_t frame[GS_ESP_COMMAND_SIZE] = {0};
  gs_esp_command decoded = {0};

  GS_EXPECT_EQ(2, GS_PROTOCOL_VERSION);
  GS_EXPECT_EQ(11, GS_ESP_COMMAND_SIZE);
  GS_EXPECT_EQ(8, GS_SLAVE_COMMAND_SIZE);
  GS_EXPECT_EQ(18, GS_SLAVE_FEEDBACK_SIZE);
  GS_EXPECT_EQ(32, GS_MASTER_FEEDBACK_SIZE);
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  GS_EXPECT_BYTES(prefix, frame, sizeof(prefix));
  GS_EXPECT_EQ(gs_crc16(frame, GS_ESP_COMMAND_SIZE - 2u),
               (uint16_t)(frame[GS_ESP_COMMAND_SIZE - 2u] |
                          ((uint16_t)frame[GS_ESP_COMMAND_SIZE - 1u] << 8)));
  GS_EXPECT_TRUE(gs_decode_esp_command(&decoded, frame));
  GS_EXPECT_EQ(0x1234, decoded.sequence);
  GS_EXPECT_EQ(-1000, decoded.speed);
  GS_EXPECT_EQ(1000, decoded.steer);
  GS_EXPECT_EQ(GS_COMMAND_DIRECT_LR, decoded.master_flags);
}

static void test_all_frame_round_trips(void) {
  const gs_slave_command command = {
      .electrical_command = -321,
      .flags = GS_COMMAND_DISABLE,
      .sequence = 17u,
  };
  const gs_slave_feedback slave = {
      .state = 7,
      .odometer = -123456,
      .faults = 0xAABBCCDDu,
      .applied_electrical = -222,
      .accepted_sequence = 17u,
      .command_age_ms = 19u,
  };
  const gs_master_feedback master = {
      .protocol_version = GS_PROTOCOL_VERSION,
      .master_state = 1,
      .slave_state = 2,
      .status_flags = GS_FEEDBACK_PEER_HEALTHY,
      .accepted_esp_sequence = 20u,
      .forwarded_slave_sequence = 20u,
      .accepted_slave_sequence = 20u,
      .left_applied = -300,
      .right_applied = 400,
      .left_odometer = -100000,
      .right_odometer = 200000,
      .master_faults = 0x01020304u,
      .slave_faults = 0xA0B0C0D0u,
      .master_command_age_ms = 10u,
      .slave_feedback_age_ms = 11u,
      .slave_command_age_ms = 12u,
  };
  uint8_t command_frame[GS_SLAVE_COMMAND_SIZE];
  uint8_t slave_frame[GS_SLAVE_FEEDBACK_SIZE];
  uint8_t master_frame[GS_MASTER_FEEDBACK_SIZE];
  gs_slave_command command_out = {0};
  gs_slave_feedback slave_out = {0};
  gs_master_feedback master_out = {0};

  GS_EXPECT_TRUE(gs_encode_slave_command(command_frame, &command));
  GS_EXPECT_TRUE(gs_decode_slave_command(&command_out, command_frame));
  GS_EXPECT_EQ(command.electrical_command, command_out.electrical_command);
  GS_EXPECT_EQ(command.flags, command_out.flags);
  GS_EXPECT_EQ(command.sequence, command_out.sequence);

  GS_EXPECT_TRUE(gs_encode_slave_feedback(slave_frame, &slave));
  GS_EXPECT_EQ(GS_SLAVE_FEEDBACK_MARKER, slave_frame[0]);
  GS_EXPECT_TRUE(gs_decode_slave_feedback(&slave_out, slave_frame));
  GS_EXPECT_EQ(slave.state, slave_out.state);
  GS_EXPECT_EQ(slave.odometer, slave_out.odometer);
  GS_EXPECT_EQ(slave.faults, slave_out.faults);
  GS_EXPECT_EQ(slave.applied_electrical, slave_out.applied_electrical);
  GS_EXPECT_EQ(slave.accepted_sequence, slave_out.accepted_sequence);
  GS_EXPECT_EQ(slave.command_age_ms, slave_out.command_age_ms);

  GS_EXPECT_TRUE(gs_encode_master_feedback(master_frame, &master));
  GS_EXPECT_EQ(GS_FEEDBACK_MARKER_0, master_frame[0]);
  GS_EXPECT_EQ(GS_FEEDBACK_MARKER_1, master_frame[1]);
  GS_EXPECT_TRUE(gs_decode_master_feedback(&master_out, master_frame));
  GS_EXPECT_EQ(GS_PROTOCOL_VERSION, master_out.protocol_version);
  GS_EXPECT_EQ(master.accepted_esp_sequence,
               master_out.accepted_esp_sequence);
  GS_EXPECT_EQ(master.forwarded_slave_sequence,
               master_out.forwarded_slave_sequence);
  GS_EXPECT_EQ(master.accepted_slave_sequence,
               master_out.accepted_slave_sequence);
  GS_EXPECT_EQ(master.left_applied, master_out.left_applied);
  GS_EXPECT_EQ(master.right_applied, master_out.right_applied);
  GS_EXPECT_EQ(master.master_faults, master_out.master_faults);
  GS_EXPECT_EQ(master.slave_faults, master_out.slave_faults);
  GS_EXPECT_EQ(master.master_command_age_ms,
               master_out.master_command_age_ms);
  GS_EXPECT_EQ(master.slave_feedback_age_ms,
               master_out.slave_feedback_age_ms);
  GS_EXPECT_EQ(master.slave_command_age_ms,
               master_out.slave_command_age_ms);
}

static void test_semantic_crc_and_version_rejection(void) {
  gs_esp_command valid = {.sequence = 1u};
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  gs_esp_command out = {0};

  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &valid));
  frame[3] ^= 1u;
  GS_EXPECT_FALSE(gs_decode_esp_command(&out, frame));

  valid.speed = 1001;
  GS_EXPECT_FALSE(gs_encode_esp_command(frame, &valid));
  valid.speed = 0;
  valid.master_flags = 0x02;
  GS_EXPECT_FALSE(gs_encode_esp_command(frame, &valid));
  valid.master_flags = 0;
  valid.slave_flags = GS_COMMAND_DIRECT_LR;
  GS_EXPECT_FALSE(gs_encode_esp_command(frame, &valid));

  GS_EXPECT_TRUE(gs_encode_esp_command(
      frame, &(gs_esp_command){.sequence = 2u}));
  frame[0] = 0x2Fu;
  GS_EXPECT_FALSE(gs_decode_esp_command(&out, frame));

  gs_master_feedback feedback = {.protocol_version = GS_PROTOCOL_VERSION};
  uint8_t feedback_frame[GS_MASTER_FEEDBACK_SIZE];
  GS_EXPECT_TRUE(gs_encode_master_feedback(feedback_frame, &feedback));
  feedback_frame[2] = 1u;
  const uint16_t crc =
      gs_crc16(feedback_frame, GS_MASTER_FEEDBACK_SIZE - 2u);
  feedback_frame[GS_MASTER_FEEDBACK_SIZE - 2u] = (uint8_t)crc;
  feedback_frame[GS_MASTER_FEEDBACK_SIZE - 1u] = (uint8_t)(crc >> 8);
  GS_EXPECT_FALSE(gs_decode_master_feedback(&feedback, feedback_frame));
}

static gs_parse_result feed_bytes(gs_frame_parser *parser, const uint8_t *bytes,
                                  size_t count, uint32_t start, uint8_t *out) {
  gs_parse_result result = GS_PARSE_NONE;
  for (size_t i = 0; i < count; ++i) {
    result = gs_frame_parser_feed(parser, bytes[i], start + (uint32_t)i, out);
  }
  return result;
}

static void test_parser_noise_repetition_crc_and_timeout(void) {
  const uint8_t marker[] = {GS_COMMAND_MARKER};
  const uint8_t noise[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  gs_frame_parser parser;
  gs_esp_command command = {.speed = 12, .steer = -34, .sequence = 7u};
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  uint8_t out[GS_MAX_FRAME_SIZE] = {0};

  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  gs_frame_parser_init(&parser, marker, sizeof(marker), sizeof(frame));
  GS_EXPECT_EQ(GS_PARSE_NONE,
               feed_bytes(&parser, noise, sizeof(noise), 0, out));
  GS_EXPECT_EQ(GS_PARSE_FRAME,
               feed_bytes(&parser, frame, sizeof(frame), 20, out));
  GS_EXPECT_BYTES(frame, out, sizeof(frame));
  GS_EXPECT_EQ(GS_PARSE_FRAME,
               feed_bytes(&parser, frame, sizeof(frame), 40, out));

  frame[4] ^= 0x55;
  GS_EXPECT_EQ(GS_PARSE_BAD_CRC,
               feed_bytes(&parser, frame, sizeof(frame), 60, out));

  GS_EXPECT_EQ(GS_PARSE_NONE, feed_bytes(&parser, frame, 3, 100, out));
  GS_EXPECT_EQ(GS_PARSE_NONE, gs_frame_parser_poll(&parser, 202));
  GS_EXPECT_EQ(GS_PARSE_TIMEOUT, gs_frame_parser_poll(&parser, 203));
}

void gs_test_protocol(void) {
  test_crc_vectors();
  test_exact_sizes_and_endianness();
  test_all_frame_round_trips();
  test_semantic_crc_and_version_rejection();
  test_parser_noise_repetition_crc_and_timeout();
}
