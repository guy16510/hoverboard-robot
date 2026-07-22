/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Chris Burns
 * Clean-room GAUSSTOP implementation informed by RoboDurden upstream.
 */
#ifndef GS_PROTOCOL_H
#define GS_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
  GS_COMMAND_MARKER = 0x2F,
  GS_FEEDBACK_MARKER_0 = 0xCD,
  GS_FEEDBACK_MARKER_1 = 0xAB,
  GS_ESP_COMMAND_SIZE = 9,
  GS_SLAVE_COMMAND_SIZE = 6,
  GS_SLAVE_FEEDBACK_SIZE = 12,
  GS_MASTER_FEEDBACK_SIZE = 26,
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

typedef struct {
  int16_t speed;
  int16_t steer;
  uint8_t master_flags;
  uint8_t slave_flags;
} gs_esp_command;

typedef struct {
  int16_t electrical_command;
  uint8_t flags;
} gs_slave_command;

typedef struct {
  uint8_t state;
  int32_t odometer;
  uint32_t faults;
} gs_slave_feedback;

typedef struct {
  uint8_t master_state;
  uint8_t slave_state;
  int16_t left_applied;
  int16_t right_applied;
  int32_t left_odometer;
  int32_t right_odometer;
  uint32_t master_faults;
  uint32_t slave_faults;
} gs_master_feedback;

uint16_t gs_crc16(const uint8_t *data, size_t length);

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
