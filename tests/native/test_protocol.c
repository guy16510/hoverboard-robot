/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include <string.h>

#include "gs_frame_parser.h"
#include "gs_protocol.h"

static void write_crc(uint8_t *frame, size_t size) {
  const uint16_t crc = gs_crc16(frame, size - 2u);
  frame[size - 2u] = (uint8_t)(crc & 0xFFu);
  frame[size - 1u] = (uint8_t)(crc >> 8);
}

static void test_crc_vector(void) {
  static const uint8_t check[] = "123456789";
  GS_EXPECT_EQ(0x31C3, gs_crc16(check, sizeof(check) - 1u));
}

static void test_exact_sizes_endianness_and_version(void) {
  const gs_esp_command command = {
      .speed = -1000,
      .steer = 1000,
      .master_flags = GS_COMMAND_DIRECT_LR,
      .sequence = 0x1234u,
      .enable_epoch = 0x2222u,
      .master_clear_fault_epoch = 0x3333u,
      .slave_clear_fault_epoch = 0x4444u,
  };
  const uint8_t prefix[] = {0x34, 0x34, 0x12, 0x22, 0x22,
                            0x33, 0x33, 0x44, 0x44, 0x18,
                            0xFC, 0xE8, 0x03, 0x20, 0x00};
  uint8_t frame[GS_ESP_COMMAND_SIZE] = {0};
  gs_esp_command decoded = {0};

  GS_EXPECT_EQ(3, GS_PROTOCOL_VERSION);
  GS_EXPECT_EQ(17, GS_ESP_COMMAND_SIZE);
  GS_EXPECT_EQ(12, GS_SLAVE_COMMAND_SIZE);
  GS_EXPECT_EQ(28, GS_SLAVE_FEEDBACK_SIZE);
  GS_EXPECT_EQ(58, GS_MASTER_FEEDBACK_SIZE);
  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  GS_EXPECT_BYTES(prefix, frame, sizeof(prefix));
  GS_EXPECT_TRUE(gs_decode_esp_command(&decoded, frame));
  GS_EXPECT_EQ(command.sequence, decoded.sequence);
  GS_EXPECT_EQ(command.enable_epoch, decoded.enable_epoch);
  GS_EXPECT_EQ(command.master_clear_fault_epoch,
               decoded.master_clear_fault_epoch);
  GS_EXPECT_EQ(command.slave_clear_fault_epoch,
               decoded.slave_clear_fault_epoch);
  GS_EXPECT_EQ(command.speed, decoded.speed);
  GS_EXPECT_EQ(command.steer, decoded.steer);
}

static void test_all_frame_round_trips(void) {
  const gs_slave_command command = {
      .electrical_command = -321,
      .flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT,
      .sequence = 17u,
      .enable_epoch = 7u,
      .clear_fault_epoch = 3u,
  };
  const gs_slave_feedback slave = {
      .state = 3u,
      .odometer = -123456,
      .faults = 0xAABBCCDDu,
      .first_fault = 0x00000040u,
      .applied_electrical = -222,
      .accepted_sequence = 17u,
      .enable_epoch = 7u,
      .fault_epoch = 3u,
      .command_age_ms = 19u,
      .clear_result = GS_CLEAR_REJECT_SAFETY,
  };
  const gs_master_feedback master = {
      .protocol_version = GS_PROTOCOL_VERSION,
      .master_state = 1u,
      .slave_state = 2u,
      .status_flags = GS_FEEDBACK_PEER_HEALTHY,
      .accepted_esp_sequence = 20u,
      .forwarded_slave_sequence = 20u,
      .accepted_slave_sequence = 20u,
      .master_enable_epoch = 8u,
      .slave_enable_epoch = 8u,
      .master_fault_epoch = 4u,
      .slave_fault_epoch = 5u,
      .master_clear_result = GS_CLEAR_OK,
      .slave_clear_result = GS_CLEAR_REJECT_STALE_EPOCH,
      .left_applied = -300,
      .right_applied = 400,
      .left_odometer = -100000,
      .right_odometer = 200000,
      .master_faults = 0x01020304u,
      .slave_faults = 0xA0B0C0D0u,
      .master_first_fault = 0x4u,
      .slave_first_fault = 0x10u,
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
  GS_EXPECT_BYTES(&command, &command_out, sizeof(command));

  GS_EXPECT_TRUE(gs_encode_slave_feedback(slave_frame, &slave));
  GS_EXPECT_TRUE(gs_decode_slave_feedback(&slave_out, slave_frame));
  GS_EXPECT_BYTES(&slave, &slave_out, sizeof(slave));

  GS_EXPECT_TRUE(gs_encode_master_feedback(master_frame, &master));
  GS_EXPECT_TRUE(gs_decode_master_feedback(&master_out, master_frame));
  GS_EXPECT_BYTES(&master, &master_out, sizeof(master));

  master_frame[2] = 2u;
  write_crc(master_frame, sizeof(master_frame));
  GS_EXPECT_FALSE(gs_decode_master_feedback(&master_out, master_frame));
}

static void test_semantic_and_crc_rejection(void) {
  gs_esp_command valid = {.sequence = 1u};
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  gs_esp_command out = {0};

  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &valid));
  frame[9] ^= 1u;
  GS_EXPECT_FALSE(gs_decode_esp_command(&out, frame));

  valid.speed = 1001;
  GS_EXPECT_FALSE(gs_encode_esp_command(frame, &valid));
  valid.speed = 0;
  valid.master_flags = 0x02u;
  GS_EXPECT_FALSE(gs_encode_esp_command(frame, &valid));
  valid.master_flags = 0u;
  valid.slave_flags = GS_COMMAND_DIRECT_LR;
  GS_EXPECT_FALSE(gs_encode_esp_command(frame, &valid));

  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &(gs_esp_command){.sequence = 2u}));
  frame[0] = 0x30u;
  GS_EXPECT_FALSE(gs_decode_esp_command(&out, frame));
}

static gs_parse_result feed_bytes(gs_frame_parser *parser, const uint8_t *bytes,
                                  size_t count, uint32_t start, uint8_t *out) {
  gs_parse_result result = GS_PARSE_NONE;
  for (size_t i = 0; i < count; ++i) {
    result = gs_frame_parser_feed(parser, bytes[i], start + (uint32_t)i, out);
  }
  return result;
}

static void test_parser_noise_fragmentation_crc_and_timeout(void) {
  const uint8_t marker[] = {GS_COMMAND_MARKER};
  const uint8_t noise[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  gs_frame_parser parser;
  gs_esp_command command = {
      .speed = 12,
      .steer = -34,
      .sequence = 1u,
      .enable_epoch = 1u,
  };
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  uint8_t out[GS_MAX_FRAME_SIZE] = {0};

  GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
  gs_frame_parser_init(&parser, marker, sizeof(marker), sizeof(frame));
  GS_EXPECT_EQ(GS_PARSE_NONE,
               feed_bytes(&parser, noise, sizeof(noise), 0u, out));
  for (size_t index = 0; index + 1u < sizeof(frame); ++index) {
    GS_EXPECT_EQ(GS_PARSE_NONE,
                 gs_frame_parser_feed(&parser, frame[index],
                                      20u + (uint32_t)index, out));
  }
  GS_EXPECT_EQ(GS_PARSE_FRAME,
               gs_frame_parser_feed(&parser, frame[sizeof(frame) - 1u], 40u,
                                    out));
  GS_EXPECT_BYTES(frame, out, sizeof(frame));

  frame[7] ^= 0x55u;
  GS_EXPECT_EQ(GS_PARSE_BAD_CRC,
               feed_bytes(&parser, frame, sizeof(frame), 60u, out));
  GS_EXPECT_EQ(GS_PARSE_NONE, feed_bytes(&parser, frame, 3u, 100u, out));
  GS_EXPECT_EQ(GS_PARSE_NONE, gs_frame_parser_poll(&parser, 202u));
  GS_EXPECT_EQ(GS_PARSE_TIMEOUT, gs_frame_parser_poll(&parser, 203u));
}

static uint32_t next_random(uint32_t *state) {
  *state = *state * 1664525u + 1013904223u;
  return *state;
}

static void test_deterministic_property_round_trips_and_corruption(void) {
  uint32_t random = 0x12345678u;
  for (unsigned iteration = 0u; iteration < 512u; ++iteration) {
    gs_esp_command command = {
        .speed = (int16_t)((int32_t)(next_random(&random) % 2001u) - 1000),
        .steer = (int16_t)((int32_t)(next_random(&random) % 2001u) - 1000),
        .master_flags = (iteration & 1u) != 0u ? GS_COMMAND_DIRECT_LR : 0u,
        .sequence = (uint16_t)next_random(&random),
        .enable_epoch = (uint16_t)next_random(&random),
        .master_clear_fault_epoch = (uint16_t)next_random(&random),
        .slave_clear_fault_epoch = (uint16_t)next_random(&random),
    };
    uint8_t frame[GS_ESP_COMMAND_SIZE];
    gs_esp_command decoded = {0};
    GS_EXPECT_TRUE(gs_encode_esp_command(frame, &command));
    GS_EXPECT_TRUE(gs_decode_esp_command(&decoded, frame));
    GS_EXPECT_BYTES(&command, &decoded, sizeof(command));
    frame[iteration % (GS_ESP_COMMAND_SIZE - 2u)] ^=
        (uint8_t)(1u << (iteration & 7u));
    GS_EXPECT_FALSE(gs_decode_esp_command(&decoded, frame));
  }
}

void gs_test_protocol(void) {
  test_crc_vector();
  test_exact_sizes_endianness_and_version();
  test_all_frame_round_trips();
  test_semantic_and_crc_rejection();
  test_parser_noise_fragmentation_crc_and_timeout();
  test_deterministic_property_round_trips_and_corruption();
}
