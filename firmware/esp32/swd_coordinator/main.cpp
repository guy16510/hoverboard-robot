/* SPDX-License-Identifier: GPL-3.0-only */
#include <Arduino.h>
#include <driver/rmt.h>

extern "C" {
#include "gausstop_swd_pulse.h"
#include "gs_console.h"
#include "gs_frame_parser.h"
#include "gs_protocol.h"
}

namespace {
constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kControllerBaud = 19200;
constexpr int kControllerRx = 35;
constexpr int kControllerTx = 17;
constexpr uint32_t kHeartbeatMs = 100;
constexpr uint32_t kPiCommandTimeoutMs = 500;
constexpr uint32_t kAckTimeoutMs = 500;
constexpr uint32_t kFeedbackTimeoutMs = 500;
constexpr rmt_channel_t kCommandRmtChannel = RMT_CHANNEL_0;
constexpr uint8_t kCommandRmtClockDivider = 80;
constexpr uint16_t kCommandPulseUnitUs = 80;
constexpr size_t kCommandRmtItems = GS_SWD_PULSE_FRAME_SYMBOLS + 1u;

constexpr int kMpu6050Sda = 21;
constexpr int kMpu6050Scl = 22;
constexpr uint32_t kFutureBalanceLoopHz = 200;

static_assert(GS_SWD_PULSE_FRAME_BYTES == GS_ESP_COMMAND_SIZE,
              "pulse transport must carry one exact ESP command frame");
static_assert(kCommandRmtItems <= 64u,
              "pulse command must fit in one ESP32 RMT memory block");
static_assert(kCommandPulseUnitUs >= GS_SWD_PULSE_MIN_UNIT_US &&
                  kCommandPulseUnitUs <= GS_SWD_PULSE_MAX_UNIT_US,
              "pulse command unit must remain inside the adaptive decoder");

HardwareSerial controller_uart(2);
gs_console_state console_state;
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
gs_command_sequencer command_sequencer;
char console_line[GS_CONSOLE_MAX_LINE + 1];
size_t console_length;
bool console_overflow;
bool clear_fault_pending;
bool command_timeout_announced;
bool session_ready;
bool command_transport_ready;
bool command_transport_error_announced;
uint32_t next_heartbeat_ms;
uint32_t last_feedback_ms;
uint32_t last_control_input_ms;
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
  return gs_master_feedback_exact_ack(
      &feedback, command_sequencer.in_flight.sequence, command_sequencer.sent);
}

bool zero_ready_ack() {
  return gs_master_feedback_motion_ready(
      &feedback, command_sequencer.in_flight.sequence, command_sequencer.sent);
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
  Serial.println("Motion requires READY acknowledgment and fresh 500 ms "
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
      "\"slave_pa4_raw_high\":%u,\"halls\":[%u,%u],"
      "\"compare\":[%u,%u],\"bridge\":[%u,%u],"
      "\"odometers\":[%ld,%ld],"
      "\"command_age_ms\":%u,\"slave_feedback_age_ms\":%u,"
      "\"slave_command_age_ms\":%u,\"esp_feedback_age_ms\":%lu,"
      "\"states\":[%u,%u],\"faults\":[%lu,%lu],\"tx\":%lu,"
      "\"rx\":%lu,\"crc_errors\":%lu,\"ack_timeouts\":%lu,"
      "\"remote_parser\":[%u,%u,%u,%u],"
      "\"command_transport\":\"pulse-rmt-v1\","
      "\"mpu_future\":[%d,%d,%lu]}\n",
      feedback.protocol_version, console_state.enabled ? 1u : 0u,
      console_state.shutdown ? 1u : 0u, session_ready ? 1u : 0u,
      console_state.target.mode == GS_DRIVE_DIRECT_LR ? "lr" : "drive",
      console_state.target.first, console_state.target.second,
      console_state.current.left, console_state.current.right,
      feedback.left_applied, feedback.right_applied,
      console_state.ramp_per_tick, command_sequencer.in_flight.sequence,
      feedback.accepted_esp_sequence, feedback.forwarded_slave_sequence,
      feedback.accepted_slave_sequence, exact_ack() ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_PEER_HEALTHY) != 0u ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_PA4_RAW_HIGH) != 0u ? 1u : 0u,
      (feedback.status_flags & GS_FEEDBACK_PA4_BYPASS) != 0u ? 1u : 0u,
      clear_fault_pending ||
              (feedback.status_flags & GS_FEEDBACK_CLEAR_PENDING) != 0u
          ? 1u
          : 0u,
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
      (feedback.motor_status_flags & GS_MASTER_MOTOR_SLAVE_PA4_RAW_HIGH) != 0u
          ? 1u
          : 0u,
      feedback.left_hall, feedback.right_hall, feedback.left_compare_offset,
      feedback.right_compare_offset,
      (feedback.motor_status_flags & GS_MASTER_MOTOR_LEFT_BRIDGE_ENABLED) != 0u
          ? 1u
          : 0u,
      (feedback.motor_status_flags & GS_MASTER_MOTOR_RIGHT_BRIDGE_ENABLED) != 0u
          ? 1u
          : 0u,
      static_cast<long>(feedback.left_odometer),
      static_cast<long>(feedback.right_odometer),
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

bool init_command_transport() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(
      static_cast<gpio_num_t>(kControllerTx), kCommandRmtChannel);
  config.clk_div = kCommandRmtClockDivider;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
  if (rmt_config(&config) != ESP_OK) {
    return false;
  }
  return rmt_driver_install(kCommandRmtChannel, 0u, 0u) == ESP_OK;
}

bool transmit_command_frame(const uint8_t frame[GS_SWD_PULSE_FRAME_BYTES]) {
  rmt_item32_t items[kCommandRmtItems] = {};
  items[0].level0 = 0u;
  items[0].duration0 = kCommandPulseUnitUs * GS_SWD_PULSE_SYNC_UNITS;
  items[0].level1 = 1u;
  items[0].duration1 = kCommandPulseUnitUs * 2u;

  size_t item_index = 1u;
  for (size_t byte = 0u; byte < GS_SWD_PULSE_FRAME_BYTES; ++byte) {
    for (uint8_t symbol_index = 0u;
         symbol_index < GS_SWD_PULSE_SYMBOLS_PER_BYTE; ++symbol_index) {
      const uint8_t symbol = static_cast<uint8_t>(
          (frame[byte] >> (symbol_index * GS_SWD_PULSE_SYMBOL_BITS)) & 0x03u);
      items[item_index].level0 = 0u;
      items[item_index].duration0 =
          static_cast<uint16_t>((symbol + 1u) * kCommandPulseUnitUs);
      items[item_index].level1 = 1u;
      items[item_index].duration1 = kCommandPulseUnitUs;
      ++item_index;
    }
  }
  return rmt_write_items(kCommandRmtChannel, items,
                         static_cast<int>(item_index), true) == ESP_OK;
}

void send_heartbeat() {
  enforce_pi_timeout();
  gs_console_ramp_tick_when_ready(&console_state, session_ready);
  const gs_esp_command desired = desired_command();
  const gs_esp_command *command = gs_command_sequencer_select(
      &command_sequencer, &desired, exact_ack(), millis());
  uint8_t frame[GS_ESP_COMMAND_SIZE];
  if (command == nullptr || !gs_encode_esp_command(frame, command)) {
    return;
  }
  if (command_transport_ready && transmit_command_frame(frame)) {
    ++command_frames;
    return;
  }
  force_safe_disable();
  if (!command_transport_error_announced) {
    command_transport_error_announced = true;
    Serial.println("SAFE STOP: ESP32 pulse transport failed");
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
      (session_ready && !gs_master_feedback_runtime_healthy(&feedback)) ||
      static_cast<uint32_t>(now - last_feedback_ms) > kFeedbackTimeoutMs) {
    force_safe_disable();
    Serial.println("SAFE STOP: stale or faulted controller feedback");
    return;
  }
  if (gs_command_sequencer_ack_expired(&command_sequencer, exact_ack(), now,
                                       kAckTimeoutMs)) {
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
  gs_command_sequencer_init(&command_sequencer);
  Serial.begin(kConsoleBaud);
  controller_uart.begin(kControllerBaud, SERIAL_8N1, kControllerRx, -1);
  command_transport_ready = init_command_transport();
  const uint8_t marker[] = {GS_FEEDBACK_MARKER_0, GS_FEEDBACK_MARKER_1};
  gs_frame_parser_init(&feedback_parser, marker, 2, GS_MASTER_FEEDBACK_SIZE);
  next_heartbeat_ms = millis();
  last_feedback_ms = millis();
  last_control_input_ms = millis();
  if (!command_transport_ready) {
    command_transport_error_announced = true;
    Serial.println("FATAL: ESP32 pulse transport initialization failed");
  }
  Serial.println(
      "GAUSSTOP SWD coordinator protocol v2, pulse-rmt-v1 commands ready, "
      "motors disabled. Type help.");
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
