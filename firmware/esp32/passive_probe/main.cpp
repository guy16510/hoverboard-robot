/* SPDX-License-Identifier: GPL-3.0-only
 * New receive-only ESP32 probe. The controller UART has no TX pin and this
 * translation unit contains no write operation on that UART.
 */
#include <Arduino.h>

extern "C" {
#include "gs_frame_parser.h"
#include "gs_protocol.h"
}

namespace {
constexpr int kControllerRx = 35;
HardwareSerial controller_rx(2);
gs_frame_parser parser;
uint32_t frames;
uint32_t crc_errors;

void service_controller_rx() {
  uint8_t frame[GS_MAX_FRAME_SIZE];
  while (controller_rx.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(controller_rx.read());
    const gs_parse_result result =
        gs_frame_parser_feed(&parser, byte, millis(), frame);
    if (result == GS_PARSE_FRAME) {
      gs_master_feedback feedback;
      if (gs_decode_master_feedback(&feedback, frame)) {
        ++frames;
        Serial.printf(
            "frame=%lu states=%u,%u odo=%ld,%ld faults=0x%08lx,0x%08lx\n",
            static_cast<unsigned long>(frames), feedback.master_state,
            feedback.slave_state, static_cast<long>(feedback.left_odometer),
            static_cast<long>(feedback.right_odometer),
            static_cast<unsigned long>(feedback.master_faults),
            static_cast<unsigned long>(feedback.slave_faults));
      }
    } else if (result == GS_PARSE_BAD_CRC) {
      ++crc_errors;
      Serial.printf("crc_error=%lu\n", static_cast<unsigned long>(crc_errors));
    }
  }
}
} // namespace

void setup() {
  Serial.begin(115200);
  controller_rx.begin(19200, SERIAL_8N1, kControllerRx, -1);
  const uint8_t marker[] = {GS_FEEDBACK_MARKER_0, GS_FEEDBACK_MARKER_1};
  gs_frame_parser_init(&parser, marker, 2, GS_MASTER_FEEDBACK_SIZE);
  Serial.println("GAUSSTOP passive probe: controller RX only; TX disabled.");
}

void loop() {
  service_controller_rx();
  (void)gs_frame_parser_poll(&parser, millis());
}
