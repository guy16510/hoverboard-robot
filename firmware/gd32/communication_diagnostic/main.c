/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdint.h>

#include "gausstop_board.h"
#include "gs_frame_parser.h"
#include "gs_master.h"
#include "gs_protocol.h"
#include "gs_safety.h"

enum { GS_DIAGNOSTIC_RECENT_BYTES = 32 };

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t remote_raw_bytes;
  uint32_t remote_valid_frames;
  uint32_t remote_crc_failures;
  uint32_t remote_replies;
  uint32_t remote_beacons;
  uint32_t recent_write_index;
  uint8_t recent_remote_bytes[GS_DIAGNOSTIC_RECENT_BYTES];
} gs_communication_diagnostic_counters;

volatile gs_communication_diagnostic_counters gs_communication_diagnostic = {
    .magic = 0x47534447u,
    .version = GS_PROTOCOL_VERSION,
};

static bool send_remote_feedback(const gs_slave_feedback *slave_status,
                                 uint32_t diagnostic_faults,
                                 uint16_t accepted_esp_sequence,
                                 uint16_t forwarded_slave_sequence,
                                 uint32_t now_ms) {
  const gs_master_feedback feedback = {
      .protocol_version = GS_PROTOCOL_VERSION,
      .master_state = GS_CONTROLLER_DISABLED,
      .slave_state = slave_status->state,
      .accepted_esp_sequence = accepted_esp_sequence,
      .forwarded_slave_sequence = forwarded_slave_sequence,
      .accepted_slave_sequence = slave_status->accepted_sequence,
      .slave_enable_epoch = slave_status->enable_epoch,
      .slave_fault_epoch = slave_status->fault_epoch,
      .slave_clear_result = slave_status->clear_result,
      .right_odometer = slave_status->odometer,
      .master_faults = diagnostic_faults,
      .slave_faults = slave_status->faults,
      .master_first_fault = diagnostic_faults,
      .slave_first_fault = slave_status->first_fault,
      .slave_command_age_ms = slave_status->command_age_ms,
  };
  uint8_t reply[GS_MASTER_FEEDBACK_SIZE];
  (void)now_ms;
  return gs_encode_master_feedback(reply, &feedback) &&
         gs_board_uart_write(GS_UART_REMOTE, reply, sizeof(reply));
}

int main(void) {
  gs_board_safe_gpio_init();
  gs_board_time_init();
  gs_board_uart_init(GS_UART_REMOTE, true);
  gs_board_uart_init(GS_UART_LINK, true);
  const uint8_t command_marker[] = {GS_COMMAND_MARKER};
  const uint8_t slave_feedback_marker[] = {GS_SLAVE_FEEDBACK_MARKER};
  gs_frame_parser remote_parser;
  gs_frame_parser link_parser;
  gs_frame_parser_init(&remote_parser, command_marker, 1, GS_ESP_COMMAND_SIZE);
  gs_frame_parser_init(&link_parser, slave_feedback_marker, 1,
                       GS_SLAVE_FEEDBACK_SIZE);
  uint32_t next_slave_ms = 0;
  uint32_t next_beacon_ms = 0;
  uint16_t accepted_esp_sequence = 0;
  uint16_t forwarded_slave_sequence = 0;
  gs_slave_feedback slave_status = {
      .state = GS_CONTROLLER_DISABLED,
      .command_age_ms = UINT16_MAX,
  };
  uint32_t diagnostic_faults = 0;

  for (;;) {
    uint8_t byte = 0;
    uint8_t frame[GS_MAX_FRAME_SIZE];
    while (gs_board_uart_read(GS_UART_REMOTE, &byte)) {
      const uint32_t recent_index =
          gs_communication_diagnostic.recent_write_index;
      gs_communication_diagnostic.recent_remote_bytes
          [recent_index % GS_DIAGNOSTIC_RECENT_BYTES] = byte;
      gs_communication_diagnostic.recent_write_index = recent_index + 1u;
      ++gs_communication_diagnostic.remote_raw_bytes;
      const gs_parse_result result =
          gs_frame_parser_feed(&remote_parser, byte, gs_board_millis(), frame);
      if (result == GS_PARSE_FRAME) {
        gs_esp_command command;
        if (!gs_decode_esp_command(&command, frame)) {
          diagnostic_faults |= GS_FAULT_PROTOCOL;
        } else {
          accepted_esp_sequence = command.sequence;
          ++gs_communication_diagnostic.remote_valid_frames;
        }
        if (send_remote_feedback(&slave_status, diagnostic_faults,
                                 accepted_esp_sequence,
                                 forwarded_slave_sequence,
                                 gs_board_millis())) {
          ++gs_communication_diagnostic.remote_replies;
        }
      } else if (result == GS_PARSE_BAD_CRC) {
        ++gs_communication_diagnostic.remote_crc_failures;
        diagnostic_faults |= GS_FAULT_PROTOCOL;
      }
    }
    while (gs_board_uart_read(GS_UART_LINK, &byte)) {
      const gs_parse_result result =
          gs_frame_parser_feed(&link_parser, byte, gs_board_millis(), frame);
      if (result == GS_PARSE_FRAME) {
        (void)gs_decode_slave_feedback(&slave_status, frame);
      } else if (result == GS_PARSE_BAD_CRC) {
        diagnostic_faults |= GS_FAULT_PROTOCOL;
      }
    }
    const uint32_t now = gs_board_millis();
    if ((int32_t)(now - next_beacon_ms) >= 0) {
      next_beacon_ms = now + 100u;
      if (send_remote_feedback(&slave_status, diagnostic_faults,
                               accepted_esp_sequence,
                               forwarded_slave_sequence, now)) {
        ++gs_communication_diagnostic.remote_beacons;
      }
    }
    if ((int32_t)(now - next_slave_ms) >= 0) {
      uint8_t disabled[GS_SLAVE_COMMAND_SIZE];
      ++forwarded_slave_sequence;
      if (forwarded_slave_sequence == 0u) {
        ++forwarded_slave_sequence;
      }
      const gs_slave_command command = {
          .electrical_command = 0,
          .flags = GS_COMMAND_DISABLE,
          .sequence = forwarded_slave_sequence,
      };
      next_slave_ms = now + 20u;
      if (gs_encode_slave_command(disabled, &command)) {
        (void)gs_board_uart_write(GS_UART_LINK, disabled, sizeof(disabled));
      }
    }
  }
}
