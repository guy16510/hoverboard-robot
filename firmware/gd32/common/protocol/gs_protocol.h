/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_PROTOCOL_H
#define GS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
  GS_PROTOCOL_VERSION = 3,
  GS_COMMAND_MARKER = 0x34,
  GS_SLAVE_FEEDBACK_MARKER = 0x35,
  GS_FEEDBACK_MARKER_0 = 0xCF,
  GS_FEEDBACK_MARKER_1 = 0xB3,
  GS_ESP_COMMAND_SIZE = 17,
  GS_SLAVE_COMMAND_SIZE = 12,
  GS_SLAVE_FEEDBACK_SIZE = 28,
  GS_MASTER_FEEDBACK_SIZE = 58,
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
} gs_feedback_flag;

typedef enum {
  GS_CLEAR_NONE = 0,
  GS_CLEAR_OK,
  GS_CLEAR_REJECT_STALE_EPOCH,
  GS_CLEAR_REJECT_STATE,
  GS_CLEAR_REJECT_SAFETY,
  GS_CLEAR_REQUIRES_RESET,
} gs_clear_result;

typedef struct {
  int16_t speed;
  int16_t steer;
  uint8_t master_flags;
  uint8_t slave_flags;
  uint16_t sequence;
  uint16_t enable_epoch;
  uint16_t master_clear_fault_epoch;
  uint16_t slave_clear_fault_epoch;
} gs_esp_command;

typedef struct {
  int16_t electrical_command;
  uint8_t flags;
  uint16_t sequence;
  uint16_t enable_epoch;
  uint16_t clear_fault_epoch;
} gs_slave_command;

typedef struct {
  uint8_t state;
  int32_t odometer;
  uint32_t faults;
  uint32_t first_fault;
  int16_t applied_electrical;
  uint16_t accepted_sequence;
  uint16_t enable_epoch;
  uint16_t fault_epoch;
  uint16_t command_age_ms;
  uint8_t clear_result;
} gs_slave_feedback;

typedef struct {
  uint8_t protocol_version;
  uint8_t master_state;
  uint8_t slave_state;
  uint8_t status_flags;
  uint16_t accepted_esp_sequence;
  uint16_t forwarded_slave_sequence;
  uint16_t accepted_slave_sequence;
  uint16_t master_enable_epoch;
  uint16_t slave_enable_epoch;
  uint16_t master_fault_epoch;
  uint16_t slave_fault_epoch;
  uint8_t master_clear_result;
  uint8_t slave_clear_result;
  int16_t left_applied;
  int16_t right_applied;
  int32_t left_odometer;
  int32_t right_odometer;
  uint32_t master_faults;
  uint32_t slave_faults;
  uint32_t master_first_fault;
  uint32_t slave_first_fault;
  uint16_t master_command_age_ms;
  uint16_t slave_feedback_age_ms;
  uint16_t slave_command_age_ms;
} gs_master_feedback;

uint16_t gs_crc16(const uint8_t *data, size_t length);
uint16_t gs_age_ms_u16(uint32_t now_ms, uint32_t then_ms);

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
