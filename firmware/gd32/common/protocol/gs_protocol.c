/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_protocol.h"

#include <limits.h>
#include <string.h>

enum {
  GS_RESERVED_FLAGS = 0x1E,
  GS_SLAVE_INVALID_FLAGS = GS_RESERVED_FLAGS | GS_COMMAND_DIRECT_LR,
};

static void write_u16(uint8_t *out, uint16_t value) {
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *out, uint32_t value) {
  out[0] = (uint8_t)(value & 0xFFu);
  out[1] = (uint8_t)((value >> 8) & 0xFFu);
  out[2] = (uint8_t)((value >> 16) & 0xFFu);
  out[3] = (uint8_t)(value >> 24);
}

static uint16_t read_u16(const uint8_t *in) {
  return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static uint32_t read_u32(const uint8_t *in) {
  return (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) |
         ((uint32_t)in[3] << 24);
}

static bool command_in_range(int16_t value) {
  return value >= -GS_COMMAND_LIMIT && value <= GS_COMMAND_LIMIT;
}

static bool crc_matches(const uint8_t *frame, size_t size) {
  return size >= 2u &&
         gs_crc16(frame, size - 2u) == read_u16(&frame[size - 2u]);
}

static void append_crc(uint8_t *frame, size_t size) {
  write_u16(&frame[size - 2u], gs_crc16(frame, size - 2u));
}

uint16_t gs_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0;
  if (data == NULL && length != 0u) {
    return 0;
  }
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (unsigned bit = 0; bit < 8u; ++bit) {
      crc = (crc & 0x8000u) != 0u ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

uint16_t gs_age_ms_u16(uint32_t now_ms, uint32_t then_ms) {
  const uint32_t age = now_ms - then_ms;
  return age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
}

bool gs_encode_esp_command(uint8_t out[GS_ESP_COMMAND_SIZE],
                           const gs_esp_command *command) {
  if (out == NULL || command == NULL || !command_in_range(command->speed) ||
      !command_in_range(command->steer) ||
      (command->master_flags & GS_RESERVED_FLAGS) != 0u ||
      (command->slave_flags & GS_SLAVE_INVALID_FLAGS) != 0u) {
    return false;
  }
  out[0] = GS_COMMAND_MARKER;
  write_u16(&out[1], command->sequence);
  write_u16(&out[3], (uint16_t)command->speed);
  write_u16(&out[5], (uint16_t)command->steer);
  out[7] = command->master_flags;
  out[8] = command->slave_flags;
  append_crc(out, GS_ESP_COMMAND_SIZE);
  return true;
}

bool gs_decode_esp_command(gs_esp_command *out,
                           const uint8_t frame[GS_ESP_COMMAND_SIZE]) {
  gs_esp_command decoded;
  if (out == NULL || frame == NULL || frame[0] != GS_COMMAND_MARKER ||
      !crc_matches(frame, GS_ESP_COMMAND_SIZE)) {
    return false;
  }
  decoded.sequence = read_u16(&frame[1]);
  decoded.speed = (int16_t)read_u16(&frame[3]);
  decoded.steer = (int16_t)read_u16(&frame[5]);
  decoded.master_flags = frame[7];
  decoded.slave_flags = frame[8];
  if (!command_in_range(decoded.speed) || !command_in_range(decoded.steer) ||
      (decoded.master_flags & GS_RESERVED_FLAGS) != 0u ||
      (decoded.slave_flags & GS_SLAVE_INVALID_FLAGS) != 0u) {
    return false;
  }
  *out = decoded;
  return true;
}

bool gs_encode_slave_command(uint8_t out[GS_SLAVE_COMMAND_SIZE],
                             const gs_slave_command *command) {
  if (out == NULL || command == NULL ||
      !command_in_range(command->electrical_command) ||
      (command->flags & GS_SLAVE_INVALID_FLAGS) != 0u) {
    return false;
  }
  out[0] = GS_COMMAND_MARKER;
  write_u16(&out[1], command->sequence);
  write_u16(&out[3], (uint16_t)command->electrical_command);
  out[5] = command->flags;
  append_crc(out, GS_SLAVE_COMMAND_SIZE);
  return true;
}

bool gs_decode_slave_command(gs_slave_command *out,
                             const uint8_t frame[GS_SLAVE_COMMAND_SIZE]) {
  gs_slave_command decoded;
  if (out == NULL || frame == NULL || frame[0] != GS_COMMAND_MARKER ||
      !crc_matches(frame, GS_SLAVE_COMMAND_SIZE)) {
    return false;
  }
  decoded.sequence = read_u16(&frame[1]);
  decoded.electrical_command = (int16_t)read_u16(&frame[3]);
  decoded.flags = frame[5];
  if (!command_in_range(decoded.electrical_command) ||
      (decoded.flags & GS_SLAVE_INVALID_FLAGS) != 0u) {
    return false;
  }
  *out = decoded;
  return true;
}

bool gs_encode_slave_feedback(uint8_t out[GS_SLAVE_FEEDBACK_SIZE],
                              const gs_slave_feedback *feedback) {
  if (out == NULL || feedback == NULL) {
    return false;
  }
  out[0] = GS_SLAVE_FEEDBACK_MARKER;
  out[1] = feedback->state;
  write_u16(&out[2], feedback->accepted_sequence);
  write_u16(&out[4], (uint16_t)feedback->applied_electrical);
  write_u32(&out[6], (uint32_t)feedback->odometer);
  write_u32(&out[10], feedback->faults);
  write_u16(&out[14], feedback->command_age_ms);
  append_crc(out, GS_SLAVE_FEEDBACK_SIZE);
  return true;
}

bool gs_decode_slave_feedback(gs_slave_feedback *out,
                              const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE]) {
  if (out == NULL || frame == NULL || frame[0] != GS_SLAVE_FEEDBACK_MARKER ||
      !crc_matches(frame, GS_SLAVE_FEEDBACK_SIZE)) {
    return false;
  }
  out->state = frame[1];
  out->accepted_sequence = read_u16(&frame[2]);
  out->applied_electrical = (int16_t)read_u16(&frame[4]);
  out->odometer = (int32_t)read_u32(&frame[6]);
  out->faults = read_u32(&frame[10]);
  out->command_age_ms = read_u16(&frame[14]);
  return true;
}

bool gs_encode_master_feedback(uint8_t out[GS_MASTER_FEEDBACK_SIZE],
                               const gs_master_feedback *feedback) {
  if (out == NULL || feedback == NULL ||
      feedback->protocol_version != GS_PROTOCOL_VERSION) {
    return false;
  }
  out[0] = GS_FEEDBACK_MARKER_0;
  out[1] = GS_FEEDBACK_MARKER_1;
  out[2] = feedback->protocol_version;
  out[3] = feedback->master_state;
  out[4] = feedback->slave_state;
  out[5] = feedback->status_flags;
  write_u16(&out[6], feedback->accepted_esp_sequence);
  write_u16(&out[8], feedback->forwarded_slave_sequence);
  write_u16(&out[10], feedback->accepted_slave_sequence);
  write_u16(&out[12], (uint16_t)feedback->left_applied);
  write_u16(&out[14], (uint16_t)feedback->right_applied);
  write_u32(&out[16], (uint32_t)feedback->left_odometer);
  write_u32(&out[20], (uint32_t)feedback->right_odometer);
  write_u32(&out[24], feedback->master_faults);
  write_u32(&out[28], feedback->slave_faults);
  write_u16(&out[32], feedback->master_command_age_ms);
  write_u16(&out[34], feedback->slave_feedback_age_ms);
  write_u16(&out[36], feedback->slave_command_age_ms);
  append_crc(out, GS_MASTER_FEEDBACK_SIZE);
  return true;
}

bool gs_decode_master_feedback(gs_master_feedback *out,
                               const uint8_t frame[GS_MASTER_FEEDBACK_SIZE]) {
  if (out == NULL || frame == NULL || frame[0] != GS_FEEDBACK_MARKER_0 ||
      frame[1] != GS_FEEDBACK_MARKER_1 ||
      !crc_matches(frame, GS_MASTER_FEEDBACK_SIZE) ||
      frame[2] != GS_PROTOCOL_VERSION) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->protocol_version = frame[2];
  out->master_state = frame[3];
  out->slave_state = frame[4];
  out->status_flags = frame[5];
  out->accepted_esp_sequence = read_u16(&frame[6]);
  out->forwarded_slave_sequence = read_u16(&frame[8]);
  out->accepted_slave_sequence = read_u16(&frame[10]);
  out->left_applied = (int16_t)read_u16(&frame[12]);
  out->right_applied = (int16_t)read_u16(&frame[14]);
  out->left_odometer = (int32_t)read_u32(&frame[16]);
  out->right_odometer = (int32_t)read_u32(&frame[20]);
  out->master_faults = read_u32(&frame[24]);
  out->slave_faults = read_u32(&frame[28]);
  out->master_command_age_ms = read_u16(&frame[32]);
  out->slave_feedback_age_ms = read_u16(&frame[34]);
  out->slave_command_age_ms = read_u16(&frame[36]);
  return true;
}

_Static_assert(GS_ESP_COMMAND_SIZE == 11, "ESP32 command wire size changed");
_Static_assert(GS_SLAVE_COMMAND_SIZE == 8, "slave command wire size changed");
_Static_assert(GS_SLAVE_FEEDBACK_SIZE == 18, "slave feedback wire size changed");
_Static_assert(GS_MASTER_FEEDBACK_SIZE == 40, "master feedback wire size changed");
