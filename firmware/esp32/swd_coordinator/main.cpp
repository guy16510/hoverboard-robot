/* SPDX-License-Identifier: GPL-3.0-only */
#include <Arduino.h>

extern "C" {
#include "gs_console.h"
#include "gs_frame_parser.h"
#include "gs_protocol.h"
}

namespace {
constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kControllerBaud = 19200;
constexpr int kControllerRx = 35; // MASTER PA14/SWCLK -> ESP32 GPIO35
constexpr int kControllerTx = 17; // ESP32 GPIO17 -> MASTER PA13/SWDIO
constexpr uint32_t kHeartbeatMs = 20;
constexpr uint32_t kPiCommandTimeoutMs = 500;

/* Reserved for the later deterministic balance loop. */
constexpr int kMpu6050Sda = 21;
constexpr int kMpu6050Scl = 22;
constexpr uint32_t kFutureBalanceLoopHz = 200;

HardwareSerial controller_uart(2);
gs_console_state console_state;
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
char console_line[GS_CONSOLE_MAX_LINE + 1];
size_t console_length;
bool console_overflow;
bool clear_fault_pending;
bool command_timeout_announced;
uint32_t next_heartbeat_ms;
uint32_t last_feedback_ms;
uint32_t last_control_input_ms;
uint32_t command_frames;
uint32_t feedback_frames;
uint32_t crc_errors;

bool motion_requested() {
  return console_state.target.first != 0 || console_state.target.second != 0 ||
         console_state.current.left != 0 || console_state.current.right != 0;
}

void force_safe_disable() {
  console_state.enabled = false;
  console_state.target = {GS_DRIVE_DIRECT_LR, 0, 0};
  console_state.current = {0, 0};
}

void print_help() {
  Serial.println("enable | lr LEFT RIGHT | drive SPEED STEER | stop | disable");
  Serial.println("forward VALUE | reverse VALUE | ramp STEP | clearfault");
  Serial.println("shutdown | status | help");
  Serial.println("Motion commands must be refreshed within 500 ms.");
}

void print_status() {
  Serial.printf(
      "enabled=%u shutdown=%u mode=%s target=%d,%d ramped=%d,%d ramp=%u "
      "command_age_ms=%lu feedback_age_ms=%lu tx=%lu rx=%lu crc_errors=%lu "
      "states=%u,%u odometers=%ld,%ld faults=0x%08lx,0x%08lx "
      "mpu_future_sda=%d mpu_future_scl=%d balance_future_hz=%lu\n",
      console_state.enabled ? 1u : 0u, console_state.shutdown ? 1u : 0u,
      console_state.target.mode == GS_DRIVE_DIRECT_LR ? "lr" : "drive",
      console_state.target.first, console_state.target.second,
      console_state.current.left, console_state.current.right,
      console_state.ramp_per_tick,
      static_cast<unsigned long>(millis() - last_control_input_ms),
      static_cast<unsigned long>(millis() - last_feedback_ms),
      static_cast<unsigned long>(command_frames),
      static_cast<unsigned long>(feedback_frames),
      static_cast<unsigned long>(crc_errors), feedback.master_state,
      feedback.slave_state, static_cast<long>(feedback.left_odometer),
      static_cast<long>(feedback.right_odometer),
      static_cast<unsigned long>(feedback.master_faults),
      static_cast<unsigned long>(feedback.slave_faults), kMpu6050Sda,
      kMpu6050Scl, static_cast<unsigned long>(kFutureBalanceLoopHz));
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
      last_control_input_ms = millis();
      command_timeout_announced = false;
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
      last_control_input_ms = millis();
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

void enforce_pi_timeout() {
  if (!console_state.enabled || !motion_requested()) {
    return;
  }
  if (static_cast<uint32_t>(millis() - last_control_input_ms) <=
      kPiCommandTimeoutMs) {
    return;
  }
  force_safe_disable();
  if (!command_timeout_announced) {
    command_timeout_announced = true;
    Serial.println("SAFE STOP: movement command timeout");
  }
}

void send_heartbeat() {
  enforce_pi_timeout();
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
  last_control_input_ms = millis();
  Serial.println("GAUSSTOP SWD coordinator ready, motors disabled. Type help.");
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
