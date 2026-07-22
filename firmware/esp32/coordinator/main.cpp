/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Chris Burns
 * New ESP32 coordinator for one-master/one-slave GAUSSTOP topology.
 */
#include <Arduino.h>

extern "C" {
#include "gs_console.h"
#include "gs_frame_parser.h"
#include "gs_protocol.h"
}

namespace {
constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kControllerBaud = 19200;
constexpr int kControllerRx = 35;
constexpr int kControllerTx = 17;
constexpr uint32_t kHeartbeatMs = 50;

HardwareSerial controller_uart(2);
gs_console_state console_state;
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
char console_line[GS_CONSOLE_MAX_LINE + 1];
size_t console_length;
bool console_overflow;
bool clear_fault_pending;
uint32_t next_heartbeat_ms;
uint32_t last_feedback_ms;
uint32_t command_frames;
uint32_t feedback_frames;
uint32_t crc_errors;

void print_help() {
  Serial.println("enable | drive SPEED STEER | lr LEFT RIGHT | forward VALUE");
  Serial.println("reverse VALUE | ramp STEP | stop | disable | clearfault");
  Serial.println("shutdown | status | help");
}

void print_status() {
  Serial.printf(
      "enabled=%u shutdown=%u mode=%s target=%d,%d ramped=%d,%d ramp=%u "
      "feedback_age_ms=%lu tx=%lu rx=%lu crc_errors=%lu states=%u,%u "
      "odometers=%ld,%ld faults=0x%08lx,0x%08lx\n",
      console_state.enabled ? 1u : 0u, console_state.shutdown ? 1u : 0u,
      console_state.target.mode == GS_DRIVE_DIRECT_LR ? "lr" : "drive",
      console_state.target.first, console_state.target.second,
      console_state.current.left, console_state.current.right,
      console_state.ramp_per_tick,
      static_cast<unsigned long>(millis() - last_feedback_ms),
      static_cast<unsigned long>(command_frames),
      static_cast<unsigned long>(feedback_frames),
      static_cast<unsigned long>(crc_errors), feedback.master_state,
      feedback.slave_state, static_cast<long>(feedback.left_odometer),
      static_cast<long>(feedback.right_odometer),
      static_cast<unsigned long>(feedback.master_faults),
      static_cast<unsigned long>(feedback.slave_faults));
}

void execute_console_line() {
  if (console_overflow || console_length == 0u) {
    if (console_overflow) {
      Serial.println("ERR overlong command");
    }
  } else {
    const gs_console_result result =
        gs_console_execute(&console_state, console_line, console_length);
    switch (result) {
    case GS_CONSOLE_APPLIED:
      Serial.println("OK");
      break;
    case GS_CONSOLE_STATUS:
      print_status();
      break;
    case GS_CONSOLE_HELP:
      print_help();
      break;
    case GS_CONSOLE_CLEAR_FAULT:
      clear_fault_pending = true;
      Serial.println("OK clearfault queued");
      break;
    case GS_CONSOLE_REJECTED:
    default:
      Serial.println("ERR command rejected; state unchanged");
      break;
    }
  }
  console_length = 0u;
  console_overflow = false;
}

void service_console() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming == '\n') {
      execute_console_line();
    } else if (incoming != '\r') {
      if (console_length < GS_CONSOLE_MAX_LINE) {
        console_line[console_length++] = static_cast<char>(incoming);
      } else {
        console_overflow = true;
      }
    }
  }
}

void send_heartbeat() {
  gs_console_ramp_tick(&console_state);
  gs_esp_command command{};
  if (console_state.shutdown) {
    command.master_flags = GS_COMMAND_DISABLE | GS_COMMAND_SHUTDOWN;
    command.slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_SHUTDOWN;
  } else if (!console_state.enabled) {
    command.master_flags = GS_COMMAND_DISABLE;
    command.slave_flags = GS_COMMAND_DISABLE;
    if (clear_fault_pending) {
      command.master_flags |= GS_COMMAND_CLEAR_FAULT;
      command.slave_flags |= GS_COMMAND_CLEAR_FAULT;
      clear_fault_pending = false;
    }
  } else {
    command.speed = console_state.current.left;
    command.steer = console_state.current.right;
    command.master_flags = GS_COMMAND_DIRECT_LR;
  }
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  if (gs_encode_esp_command(frame, &command)) {
    controller_uart.write(frame, sizeof(frame));
    ++command_frames;
  }
}

void service_feedback() {
  uint8_t frame[GS_MAX_FRAME_SIZE];
  while (controller_uart.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(controller_uart.read());
    const gs_parse_result result =
        gs_frame_parser_feed(&feedback_parser, byte, millis(), frame);
    if (result == GS_PARSE_FRAME) {
      if (gs_decode_master_feedback(&feedback, frame)) {
        ++feedback_frames;
        last_feedback_ms = millis();
      } else {
        ++crc_errors;
      }
    } else if (result == GS_PARSE_BAD_CRC) {
      ++crc_errors;
    }
  }
  (void)gs_frame_parser_poll(&feedback_parser, millis());
}
} // namespace

void setup() {
  gs_console_init(&console_state);
  Serial.begin(kConsoleBaud);
  controller_uart.begin(kControllerBaud, SERIAL_8N1, kControllerRx,
                        kControllerTx);
  const uint8_t marker[] = {GS_FEEDBACK_MARKER_0, GS_FEEDBACK_MARKER_1};
  gs_frame_parser_init(&feedback_parser, marker, 2, GS_MASTER_FEEDBACK_SIZE);
  next_heartbeat_ms = millis();
  last_feedback_ms = millis();
  Serial.println(
      "GAUSSTOP coordinator ready; disabled with zero demand. Type help.");
}

void loop() {
  service_console();
  service_feedback();
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_heartbeat_ms) >= 0) {
    next_heartbeat_ms = now + kHeartbeatMs;
    send_heartbeat();
  }
}
