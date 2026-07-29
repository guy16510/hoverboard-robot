/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_PROTOCOL_H
#define GS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
  /*
   * Wire epoch 4 deliberately changes every southbound frame discriminator.
   * ESP32, MASTER, and SLAVE images from another epoch cannot exchange valid
   * commands or feedback, so a partial or mistaken flash fails closed.
   */
  GS_PROTOCOL_VERSION = 4,
  GS_COMMAND_MARKER = 0x42,
  GS_SLAVE_FEEDBACK_MARKER = 0x43,
  GS_FEEDBACK_MARKER_0 = 0xCE,
  GS_FEEDBACK_MARKER_1 = 0xB3,
  GS_ESP_COMMAND_SIZE = 11,
  GS_SLAVE_COMMAND_SIZE = 8,
  GS_SLAVE_FEEDBACK_SIZE = 28,
  GS_MASTER_FEEDBACK_SIZE = 67,
  GS_MAX_FRAME_SIZE = GS_MASTER_FEEDBACK_SIZE,
  GS_PARTIAL_FRAME_TIMEOUT_MS = 100,
  GS_COMMAND_LIMIT = 1000,
};

typedef enum {
  GS_COMMAND_CLEAR_FAULT = 1u << 0,
  GS_COMMAND_DIRECT_LR = 1u << 5,
  GS_COMMAND_DISABLE = 1u << 6,
  GS_COMMAND_SHUTDOWN = 1u << 7,
} gs_command_flag;

typedef enum {
  GS_FEEDBACK_PEER_HEALTHY = 1u << 0,
  GS_FEEDBACK_PA4_RAW_HIGH = 1u << 1,
  GS_FEEDBACK_PA4_BYPASS = 1u << 2,
  GS_FEEDBACK_CLEAR_PENDING = 1u << 3,
  GS_FEEDBACK_TRANSPORT_REMOTE_RX_OVERFLOW = 1u << 4,
  GS_FEEDBACK_TRANSPORT_REMOTE_TX_OVERFLOW = 1u << 5,
  GS_FEEDBACK_TRANSPORT_LINK_RX_OVERFLOW = 1u << 6,
  GS_FEEDBACK_TRANSPORT_LINK_TX_OVERFLOW = 1u << 7,
} gs_feedback_flag;

typedef enum {
  GS_MOTOR_FEEDBACK_BRIDGE_ENABLED = 1u << 0,
  GS_MOTOR_FEEDBACK_PA4_RAW_HIGH = 1u << 1,
  GS_MOTOR_FEEDBACK_CLEAR_PENDING = 1u << 2,
} gs_motor_feedback_flag;

typedef enum {
  GS_MASTER_MOTOR_LEFT_BRIDGE_ENABLED = 1u << 0,
  GS_MASTER_MOTOR_RIGHT_BRIDGE_ENABLED = 1u << 1,
  GS_MASTER_MOTOR_SLAVE_PA4_RAW_HIGH = 1u << 2,
} gs_master_motor_status_flag;

typedef enum {
  GS_CONTROLLER_DISABLED = 0,
  GS_CONTROLLER_READY,
  GS_CONTROLLER_ACTIVE,
  GS_CONTROLLER_FAULTED,
  GS_CONTROLLER_SHUTDOWN,
} gs_controller_state;

typedef struct {
  int16_t speed;
  int16_t steer;
  uint8_t master_flags;
  uint8_t slave_flags;
  uint16_t sequence;
} gs_esp_command;

typedef struct {
  gs_esp_command in_flight;
  uint32_t sequence_started_ms;
  uint16_t sequence;
  bool sent;
} gs_command_sequencer;

typedef struct {
  int16_t electrical_command;
  uint8_t flags;
  uint16_t sequence;
} gs_slave_command;

typedef struct {
  uint8_t state;
  int32_t odometer;
  uint32_t faults;
  int16_t applied_electrical;
  uint16_t accepted_sequence;
  uint16_t command_age_ms;
  uint8_t hall;
  uint8_t status_flags;
  uint16_t compare_offset;
  uint16_t hall_glitch_count;
  uint16_t command_invalid_frames;
  uint16_t command_framing_errors;
} gs_slave_feedback;

typedef struct {
  uint8_t protocol_version;
  uint8_t master_state;
  uint8_t slave_state;
  uint8_t status_flags;
  uint16_t accepted_esp_sequence;
  uint16_t forwarded_slave_sequence;
  uint16_t accepted_slave_sequence;
  int16_t left_applied;
  int16_t right_applied;
  int32_t left_odometer;
  int32_t right_odometer;
  uint32_t master_faults;
  uint32_t slave_faults;
  uint16_t master_command_age_ms;
  uint16_t slave_feedback_age_ms;
  uint16_t slave_command_age_ms;
  uint8_t left_hall;
  uint8_t right_hall;
  uint16_t left_compare_offset;
  uint16_t right_compare_offset;
  uint8_t motor_status_flags;
  uint16_t remote_rx_bytes;
  uint16_t remote_valid_frames;
  uint16_t remote_invalid_frames;
  uint16_t remote_framing_errors;
  uint16_t left_hall_glitch_count;
  uint16_t right_hall_glitch_count;
  uint16_t slave_feedback_invalid_frames;
  uint16_t slave_feedback_framing_errors;
  uint16_t slave_command_invalid_frames;
  uint16_t slave_command_framing_errors;
} gs_master_feedback;

uint16_t gs_crc16(const uint8_t *data, size_t length);
uint16_t gs_age_ms_u16(uint32_t now_ms, uint32_t then_ms);

void gs_command_sequencer_init(gs_command_sequencer *sequencer);
const gs_esp_command *
gs_command_sequencer_select(gs_command_sequencer *sequencer,
                            const gs_esp_command *desired, bool exact_ack,
                            uint32_t now_ms);
bool gs_command_sequencer_ack_expired(const gs_command_sequencer *sequencer,
                                      bool exact_ack, uint32_t now_ms,
                                      uint32_t timeout_ms);
bool gs_master_feedback_exact_ack(const gs_master_feedback *feedback,
                                  uint16_t expected_sequence,
                                  bool command_sent);
bool gs_master_feedback_motion_ready(const gs_master_feedback *feedback,
                                     uint16_t expected_sequence,
                                     bool command_sent);
bool gs_master_feedback_runtime_healthy(const gs_master_feedback *feedback);

bool gs_encode_esp_command(uint8_t out[GS_ESP_COMMAND_SIZE],
                           const gs_esp_command *command);
bool gs_decode_esp_command(gs_esp_command *out,
                           const uint8_t frame[GS_ESP_COMMAND_SIZE]);
bool gs_encode_slave_command(uint8_t out[GS_SLAVE_COMMAND_SIZE],
                             const gs_slave_command *command);
bool gs_decode_slave_command(gs_slave_command *out,
                             const uint8_t frame[GS_SLAVE_COMMAND_SIZE]);
bool gs_encode_slave_feedback(uint8_t out[GS_SLAVE_FEEDBACK_SIZE],
                              const gs_slave_feedback *feedback);
bool gs_decode_slave_feedback(gs_slave_feedback *out,
                              const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE]);
bool gs_encode_master_feedback(uint8_t out[GS_MASTER_FEEDBACK_SIZE],
                               const gs_master_feedback *feedback);
bool gs_decode_master_feedback(gs_master_feedback *out,
                               const uint8_t frame[GS_MASTER_FEEDBACK_SIZE]);

#endif
