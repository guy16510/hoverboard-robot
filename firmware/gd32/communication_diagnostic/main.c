/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdint.h>

#include "gausstop_board.h"
#include "gs_frame_parser.h"
#include "gs_master.h"
#include "gs_protocol.h"
#include "gs_safety.h"

int main(void) {
  gs_board_safe_gpio_init();
  gs_board_time_init();
  gs_board_uart_init(GS_UART_REMOTE, true);
  gs_board_uart_init(GS_UART_LINK, true);
  const uint8_t command_marker[] = {GS_COMMAND_MARKER};
  gs_frame_parser remote_parser;
  gs_frame_parser link_parser;
  gs_frame_parser_init(&remote_parser, command_marker, 1, GS_ESP_COMMAND_SIZE);
  gs_frame_parser_init(&link_parser, command_marker, 1, GS_SLAVE_FEEDBACK_SIZE);
  uint32_t next_slave_ms = 0;
  gs_slave_feedback slave_status = {GS_CONTROLLER_DISABLED, 0, 0};
  uint32_t diagnostic_faults = 0;

  for (;;) {
    uint8_t byte = 0;
    uint8_t frame[GS_MAX_FRAME_SIZE];
    while (gs_board_uart_read(GS_UART_REMOTE, &byte)) {
      const gs_parse_result result =
          gs_frame_parser_feed(&remote_parser, byte, gs_board_millis(), frame);
      if (result == GS_PARSE_FRAME) {
        gs_esp_command ignored;
        if (!gs_decode_esp_command(&ignored, frame)) {
          diagnostic_faults |= GS_FAULT_PROTOCOL;
        }
        const gs_master_feedback feedback = {GS_CONTROLLER_DISABLED,
                                             slave_status.state,
                                             0,
                                             0,
                                             0,
                                             slave_status.odometer,
                                             diagnostic_faults,
                                             slave_status.faults};
        uint8_t reply[GS_MASTER_FEEDBACK_SIZE];
        if (gs_encode_master_feedback(reply, &feedback)) {
          (void)gs_board_uart_write(GS_UART_REMOTE, reply, sizeof(reply));
        }
      } else if (result == GS_PARSE_BAD_CRC) {
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
    if ((int32_t)(now - next_slave_ms) >= 0) {
      uint8_t disabled[GS_SLAVE_COMMAND_SIZE];
      const gs_slave_command command = {0, GS_COMMAND_DISABLE};
      next_slave_ms = now + 20u;
      if (gs_encode_slave_command(disabled, &command)) {
        (void)gs_board_uart_write(GS_UART_LINK, disabled, sizeof(disabled));
      }
    }
  }
}
