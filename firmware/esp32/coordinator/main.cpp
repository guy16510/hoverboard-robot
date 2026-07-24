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
constexpr int kControllerRx = 35;
constexpr int kControllerTx = 17;
constexpr uint32_t kHeartbeatMs = 20;
constexpr uint32_t kPiCommandTimeoutMs = 500;
constexpr uint32_t kAckTimeoutMs = 100;
constexpr uint32_t kFeedbackTimeoutMs = 150;

constexpr int kMpu6050Sda = 21;
constexpr int kMpu6050Scl = 22;
constexpr uint32_t kFutureBalanceLoopHz = 200;

HardwareSerial controller_uart(2);
gs_console_state console_state;
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
gs_esp_command last_command{};
char console_line[GS_CONSOLE_MAX_LINE + 1];
size_t console_length;
bool console_overflow;
bool clear_fault_pending;
bool command_timeout_announced;
bool session_ready;
bool command_sent;
uint16_t command_sequence;
uint16_t last_sent_sequence;
uint32_t next_heartbeat_ms;
uint32_t last_feedback_ms;
uint32_t last_control_input_ms;
uint32_t last_sequence_change_ms;
uint32_t command_frames;
uint32_t feedback_frames;
uint32_t crc_errors;
uint32_t ack_timeouts;

bool motion_requested() {
  return console_state.target.first != 0 || console_state.target.second != 0 ||
         console_state.current.left != 0 || console_state.current.right != 0;
}

bool feedback_faulted() {
  return feedback.master_faults != 0u || feedback.slave_faults != 0u;
}

bool exact_ack() {
  return command_sent && feedback.accepted_esp_sequence == last_sent_sequence &&
         feedback.forwarded_slave_sequence == last_sent_sequence &&
         feedback.accepted_slave_sequence == last_sent_sequence;
}

bool zero_ready_ack() {
  return exact_ack() && feedback.master_state == 1u &&
         feedback.slave_state == 1u && !feedback_faulted() &&
         (feedback.status_flags & GS_FEEDBACK_PEER_HEALTHY) != 0u;
}

void force_safe_disable() {
  console_state.enabled = false;
  console_state.target = {GS_DRIVE_DIRECT_LR, 0, 0};
  console_state.current = {0, 0};
  session_ready = false;
}

void print_help() {
  Serial.println("enable | lr LEFT RIGHT | drive SPEED STEER | stop | disable");
  Serial.println("forward VALUE | reverse VALUE | ramp STEP | clearfault");
  Serial.println("shutdown | status | help");
  Serial.println("Motion requires READY acknowledgment and fresh 100 ms "
                 "sequence acknowledgment.");
}

void print_status() {
  Serial.printf(
      "STATUS {\"protocol\":%u,\"enabled\":%u,\"shutdown\":%u,"
      "\"session_ready\":%u,\"mode\":\"%s\",\"requested\":[%d,%d],"
      "\"ramped\":[%d,%d],\"applied\":[%d,%d],\"ramp\":%u,"
      "\"sequence\":%u,\"master_ack\":%u,\"slave_forwarded\":%u,"
      "\"slave_ack\":%u,\"exact_ack\":%u,\"peer_healthy\":%u,"
      "\"pa4_raw_high\":%u,\"pa4_bypass\":%u,\"clear_pending\":%u,"
      "\"transport_overflows\":[%u,%u,%u,%u],"
      "\"command_age_ms\":%u,\"slave_feedback_age_ms\":%u,"
      "\"slave_command_age_ms\":%u,\"esp_feedback_age_ms\":%lu,"
      "\"states\":[%u,%u],\"faults\":[%lu,%lu],\"tx\":%lu,"
      "\"rx\":%lu,\"crc_errors\":%lu,\"ack_timeouts\":%lu,"
      "\"remote_parser\":[%u,%u,%u,%u],"
      "\"mpu_future\":[%d,%d,%lu]}\n",
      feedback.protocol_version, console_state.enabled ? 1u : 0u,
      console_state.shutdown ? 1u : 0u, session_ready ? 1u : 0u,
      console_state.target.mode == GS_DRIVE_DIRECT_LR ? "lr" : "drive",
      console_state.target.first, console_state.target.second,
      console_state.current.left, console_state.current.right,
      feedback.left_applied, feedback.right_applied,
      console_state.ramp_per_tick, last_sent_sequence,
      feedback.accepted_esp_sequence, feedback.forwarded_slave_sequence,
      feedback.accepted_slave_sequence, exact_ack() ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_PEER_HEALTHY) != 0u ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_PA4_RAW_HIGH) != 0u ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_PA4_BYPASS) != 0u ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_CLEAR_PENDING) != 0u ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_TRANSPORT_REMOTE_RX_OVERFLOW) != 0u
          ? 1u
          : 0u,
      (feedback.status_flags & GS_FEEDBACK_TRANSPORT_REMOTE_TX_OVERFLOW) != 0u
          ? 1u
          : 0u,
      (feedback.status_flags & GS_FEEDBACK_TRANSPORT_LINK_RX_OVERFLOW) != 0u
          ? 1u
          : 0u,
      (feedback.status_flags & GS_FEEDBACK_TRANSPORT_LINK_TX_OVERFLOW) != 0u
          ? 1u
          : 0u,
      feedback.master_command_age_ms, feedback.slave_feedback_age_ms,
      feedback.slave_command_age_ms,
      static_cast<unsigned long>(millis() - last_feedback_ms),
      feedback.master_state, feedback.slave_state,
      static_cast<unsigned long>(feedback.master_faults),
      static_cast<unsigned long>(feedback.slave_faults),
      static_cast<unsigned long>(command_frames),
      static_cast<unsigned long>(feedback_frames),
      static_cast<unsigned long>(crc_errors),
      static_cast<unsigned long>(ack_timeouts), feedback.remote_rx_bytes,
      feedback.remote_valid_frames, feedback.remote_invalid_frames,
      feedback.remote_framing_errors, kMpu6050Sda, kMpu6050Scl,
      static_cast<unsigned long>(kFutureBalanceLoopHz));
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
      if (!console_state.enabled) {
        session_ready = false;
      }
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
      Serial.println("QUEUED clearfault, awaiting controller confirmation");
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

bool same_payload(const gs_esp_command &first, const gs_esp_command &second) {
  return first.speed == second.speed && first.steer == second.steer &&
         first.master_flags == second.master_flags &&
         first.slave_flags == second.slave_flags;
}

gs_esp_command desired_command() {
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
    }
  } else {
    command.master_flags = GS_COMMAND_DIRECT_LR;
    if (session_ready) {
      command.speed = console_state.current.left;
      command.steer = console_state.current.right;
    }
  }
  return command;
}

void send_heartbeat() {
  enforce_pi_timeout();
  gs_console_ramp_tick(&console_state);
  gs_esp_command command = desired_command();
  if (!command_sent || !same_payload(command, last_command)) {
    ++command_sequence;
    if (command_sequence == 0u) {
      ++command_sequence;
    }
    last_sequence_change_ms = millis();
  }
  command.sequence = command_sequence;
  last_command = command;
  last_sent_sequence = command.sequence;
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  if (gs_encode_esp_command(frame, &command)) {
    controller_uart.write(frame, sizeof(frame));
    ++command_frames;
    command_sent = true;
  }
}

void enforce_controller_health() {
  const uint32_t now = millis();
  if (console_state.enabled && !session_ready && zero_ready_ack()) {
    session_ready = true;
    Serial.println("READY acknowledged by MASTER and SLAVE");
  }
  if (clear_fault_pending && exact_ack() && !feedback_faulted() &&
      (feedback.status_flags & GS_FEEDBACK_CLEAR_PENDING) == 0u) {
    clear_fault_pending = false;
    Serial.println("CLEAR confirmed by MASTER and SLAVE");
  }
  if (!console_state.enabled) {
    return;
  }
  if (feedback_faulted() ||
      static_cast<uint32_t>(now - last_feedback_ms) > kFeedbackTimeoutMs) {
    force_safe_disable();
    Serial.println("SAFE STOP: stale or faulted controller feedback");
    return;
  }
  if (!exact_ack() &&
      static_cast<uint32_t>(now - last_sequence_change_ms) > kAckTimeoutMs) {
    ++ack_timeouts;
    force_safe_disable();
    Serial.println("SAFE STOP: command sequence was not acknowledged");
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
  enforce_controller_health();
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
  last_sequence_change_ms = millis();
  Serial.println("GAUSSTOP SWD coordinator protocol v2 ready, motors disabled. "
                 "Type help.");
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
