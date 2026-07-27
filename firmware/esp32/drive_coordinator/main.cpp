/* SPDX-License-Identifier: GPL-3.0-only */
#include <Arduino.h>
#include <Wire.h>
#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

#include "command_arbiter.h"
#include "differential_drive_mixer.h"
#include "serial_command_source.h"
#include "serial_messages.h"

extern "C" {
#include "gausstop_swd_pulse.h"
#include "gs_frame_parser.h"
#include "gs_protocol.h"
}

namespace {
using namespace gs::balance;
using namespace gs::drive;

constexpr uint32_t kConsoleBaud = 115200u;
constexpr uint32_t kControllerBaud = 19200u;
constexpr int kControllerRx = 35;
constexpr int kControllerTx = 17;
constexpr int kMpuSda = 21;
constexpr int kMpuScl = 22;
constexpr int kLocalDisarmPin = 0;
constexpr uint8_t kMpuAddress = 0x68u;
constexpr uint32_t kLoopPeriodUs = 10000u;
constexpr uint32_t kMotorHeartbeatUs = 100000u;
constexpr uint32_t kFeedbackTimeoutUs = 500000u;
constexpr uint32_t kParserTimeoutUs = 50000u;
constexpr uint32_t kAckTimeoutMs = 500u;
constexpr float kMaximumTiltDeg = 45.0f;
constexpr size_t kMaximumSerialBytesPerPass = 64u;
constexpr size_t kMaximumFeedbackBytesPerPass = 64u;
constexpr rmt_channel_t kCommandRmtChannel = RMT_CHANNEL_0;
constexpr uint8_t kCommandRmtClockDivider = 80u;
constexpr uint16_t kCommandPulseUnitUs = 80u;
constexpr size_t kCommandRmtItems = GS_SWD_PULSE_FRAME_SYMBOLS + 1u;

static_assert(GS_SWD_PULSE_FRAME_BYTES == GS_ESP_COMMAND_SIZE,
              "pulse transport must carry one command frame");

HardwareSerial controller_uart(2);
SerialCommandSource serial_source(kParserTimeoutUs);
CommandArbiter command_arbiter;
DifferentialDriveMixer mixer(DifferentialDriveConfig{});
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
gs_command_sequencer command_sequencer;

bool command_transport_ready = false;
bool imu_healthy = false;
bool armed = false;
bool faulted = false;
uint8_t operating_mode = 0u;
uint64_t last_feedback_us = 0u;
uint64_t next_loop_us = 0u;
uint64_t next_motor_heartbeat_us = 0u;
uint16_t timed_out_sequence = 0u;
float pitch_deg = 0.0f;
float roll_deg = 0.0f;
float requested_left = 0.0f;
float requested_right = 0.0f;

uint64_t nowMicros() { return static_cast<uint64_t>(esp_timer_get_time()); }

bool initializeCommandTransport() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(
      static_cast<gpio_num_t>(kControllerTx), kCommandRmtChannel);
  config.clk_div = kCommandRmtClockDivider;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
  return rmt_config(&config) == ESP_OK &&
         rmt_driver_install(kCommandRmtChannel, 0u, 0u) == ESP_OK;
}

bool transmitCommandFrame(const uint8_t frame[GS_SWD_PULSE_FRAME_BYTES]) {
  if (rmt_wait_tx_done(kCommandRmtChannel, 0u) != ESP_OK) {
    return false;
  }
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
                         static_cast<int>(item_index), false) == ESP_OK;
}

bool exactAck() {
  return gs_master_feedback_exact_ack(
      &feedback, command_sequencer.in_flight.sequence, command_sequencer.sent);
}

bool feedbackFresh(uint64_t now_us) {
  return last_feedback_us != 0u && now_us - last_feedback_us <= kFeedbackTimeoutUs;
}

bool feedbackHealthy(uint64_t now_us) {
  return feedbackFresh(now_us) && gs_master_feedback_runtime_healthy(&feedback);
}

bool safeToArm(uint64_t now_us) {
  return operating_mode == 2u && imu_healthy && feedbackHealthy(now_us) &&
         command_transport_ready && exactAck() && !faulted &&
         std::fabs(pitch_deg) < kMaximumTiltDeg &&
         std::fabs(roll_deg) < kMaximumTiltDeg;
}

void stopMotion() {
  requested_left = 0.0f;
  requested_right = 0.0f;
  mixer.stop();
}

void writeFrame(const SerialFrame &frame) {
  uint8_t encoded[kSerialMaximumFrameSize] = {};
  const size_t length = SerialProtocol::encode(frame, encoded, sizeof(encoded));
  if (length != 0u) {
    Serial.write(encoded, length);
  }
}

void acknowledge(const SerialFrame &request, uint8_t status = 0u) {
  SerialFrame response;
  response.type = SerialMessageType::kAcknowledgment;
  response.sequence = request.sequence;
  response.payload_length = 2u;
  response.payload[0] = static_cast<uint8_t>(request.type);
  response.payload[1] = status;
  writeFrame(response);
}

void respondCapabilities(const SerialFrame &request) {
  ProtocolCapabilities capabilities;
  capabilities.dry_run = false;
  capabilities.web_enabled = false;
  capabilities.control_rate_hz = static_cast<uint16_t>(1000000u / kLoopPeriodUs);
  capabilities.motor_rate_hz =
      static_cast<uint16_t>(1000000u / kMotorHeartbeatUs);
  capabilities.configuration_keys = 0u;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length = SerialMessageCodec::encodeCapabilities(
      capabilities, payload, sizeof(payload));
  SerialFrame response;
  response.type = SerialMessageType::kCapabilities;
  response.sequence = request.sequence;
  response.payload_length = static_cast<uint16_t>(length);
  std::copy(payload, payload + length, response.payload.begin());
  writeFrame(response);
}

void respondStatus(const SerialFrame &request) {
  ProtocolStatus status;
  status.state = faulted ? 6u : (armed ? 4u : 2u);
  status.operating_mode = operating_mode;
  status.active_source = static_cast<uint8_t>(command_arbiter.activeSource());
  status.health_flags = (imu_healthy ? 1u : 0u) |
                        (feedbackFresh(nowMicros()) ? 1u << 2u : 0u) |
                        (armed ? 1u << 4u : 0u) | 1u << 6u;
  status.faults = faulted ? 1u : 0u;
  status.loop_overruns = 0u;
  status.rejected_serial_frames = serial_source.rejectedFrames();
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeStatus(status, payload, sizeof(payload));
  SerialFrame response;
  response.type = SerialMessageType::kStatus;
  response.sequence = request.sequence;
  response.payload_length = static_cast<uint16_t>(length);
  std::copy(payload, payload + length, response.payload.begin());
  writeFrame(response);
}

void serviceNorthboundSerial(uint64_t now_us) {
  size_t serviced = 0u;
  while (Serial.available() > 0 && serviced < kMaximumSerialBytesPerPass) {
    const auto byte = static_cast<uint8_t>(Serial.read());
    ++serviced;
    const SerialParseResult result = serial_source.feed(byte, now_us);
    if (result != SerialParseResult::kFrame) {
      continue;
    }
    const SerialFrame &frame = serial_source.lastFrame();
    if (frame.type == SerialMessageType::kHello ||
        frame.type == SerialMessageType::kCapabilities) {
      armed = false;
      stopMotion();
      respondCapabilities(frame);
    } else if (frame.type == SerialMessageType::kStatus) {
      respondStatus(frame);
    } else {
      acknowledge(frame);
    }
  }
  ControlRequest request{};
  if (serial_source.latest(request)) {
    command_arbiter.submit(request);
  }
}

void serviceFeedback(uint64_t now_us) {
  uint8_t frame[GS_MAX_FRAME_SIZE] = {};
  size_t serviced = 0u;
  while (controller_uart.available() > 0 &&
         serviced < kMaximumFeedbackBytesPerPass) {
    const auto byte = static_cast<uint8_t>(controller_uart.read());
    ++serviced;
    const gs_parse_result result = gs_frame_parser_feed(
        &feedback_parser, byte, static_cast<uint32_t>(now_us / 1000u), frame);
    if (result == GS_PARSE_FRAME && gs_decode_master_feedback(&feedback, frame)) {
      last_feedback_us = now_us;
    }
  }
  (void)gs_frame_parser_poll(&feedback_parser,
                             static_cast<uint32_t>(now_us / 1000u));
}

void serviceImu() {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(0x3Bu);
  if (Wire.endTransmission(false) != 0u ||
      Wire.requestFrom(kMpuAddress, static_cast<uint8_t>(6u), true) != 6u) {
    imu_healthy = false;
    return;
  }
  const int16_t ax = static_cast<int16_t>((Wire.read() << 8u) | Wire.read());
  const int16_t ay = static_cast<int16_t>((Wire.read() << 8u) | Wire.read());
  const int16_t az = static_cast<int16_t>((Wire.read() << 8u) | Wire.read());
  const float x = static_cast<float>(ax);
  const float y = static_cast<float>(ay);
  const float z = static_cast<float>(az);
  pitch_deg = std::atan2(-x, std::sqrt(y * y + z * z)) * 57.2957795f;
  roll_deg = std::atan2(y, z) * 57.2957795f;
  imu_healthy = std::isfinite(pitch_deg) && std::isfinite(roll_deg);
}

void applyControlRequest(const ControlRequest &request, uint64_t now_us) {
  if (request.set_operating_mode) {
    operating_mode = request.operating_mode;
    armed = false;
    stopMotion();
  }
  if (request.emergency_stop || request.disarm ||
      digitalRead(kLocalDisarmPin) == LOW) {
    armed = false;
    stopMotion();
    return;
  }
  if (request.clear_fault && imu_healthy && feedbackHealthy(now_us)) {
    faulted = false;
  }
  if (request.arm && safeToArm(now_us)) {
    armed = true;
  }
  if (!armed || request.source == CommandSource::kNone) {
    stopMotion();
    return;
  }
  const DifferentialDriveOutput output = mixer.update(
      request.linear_velocity, request.yaw_rate,
      static_cast<float>(kLoopPeriodUs) / 1000000.0f);
  if (!output.valid) {
    faulted = true;
    armed = false;
    stopMotion();
    return;
  }
  requested_left = output.left;
  requested_right = output.right;
}

void serviceMotorHeartbeat(uint64_t now_us) {
  if (now_us < next_motor_heartbeat_us || !command_transport_ready) {
    return;
  }
  next_motor_heartbeat_us = now_us + kMotorHeartbeatUs;
  gs_esp_command command{};
  if (armed && !faulted && feedbackHealthy(now_us) && imu_healthy &&
      std::fabs(pitch_deg) < kMaximumTiltDeg &&
      std::fabs(roll_deg) < kMaximumTiltDeg) {
    command.master_flags = GS_COMMAND_DIRECT_LR;
    command.speed = static_cast<int16_t>(requested_left);
    command.steer = static_cast<int16_t>(requested_right);
  } else {
    command.master_flags = GS_COMMAND_DISABLE;
    command.slave_flags = GS_COMMAND_DISABLE;
    armed = false;
    stopMotion();
  }

  const gs_esp_command *selected = gs_command_sequencer_select(
      &command_sequencer, &command, exactAck(),
      static_cast<uint32_t>(now_us / 1000u));
  uint8_t frame[GS_ESP_COMMAND_SIZE] = {};
  if (selected == nullptr || !gs_encode_esp_command(frame, selected) ||
      !transmitCommandFrame(frame)) {
    faulted = true;
    armed = false;
    stopMotion();
    return;
  }
  if (gs_command_sequencer_ack_expired(
          &command_sequencer, exactAck(),
          static_cast<uint32_t>(now_us / 1000u), kAckTimeoutMs) &&
      timed_out_sequence != command_sequencer.in_flight.sequence) {
    timed_out_sequence = command_sequencer.in_flight.sequence;
    faulted = true;
    armed = false;
    stopMotion();
  }
}

} // namespace

void setup() {
  pinMode(kLocalDisarmPin, INPUT_PULLUP);
  Serial.begin(kConsoleBaud);
  controller_uart.begin(kControllerBaud, SERIAL_8N1, kControllerRx, -1);
  Wire.begin(kMpuSda, kMpuScl, 400000u);
  Wire.beginTransmission(kMpuAddress);
  Wire.write(0x6Bu);
  Wire.write(0u);
  imu_healthy = Wire.endTransmission(true) == 0u;
  gs_frame_parser_init(&feedback_parser, 100u);
  gs_command_sequencer_init(&command_sequencer);
  command_transport_ready = initializeCommandTransport();
  faulted = !command_transport_ready;
  next_loop_us = nowMicros();
  next_motor_heartbeat_us = next_loop_us;
  esp_task_wdt_init(2u, true);
  esp_task_wdt_add(nullptr);
}

void loop() {
  const uint64_t now_us = nowMicros();
  serviceNorthboundSerial(now_us);
  serviceFeedback(now_us);
  if (now_us >= next_loop_us) {
    next_loop_us = now_us + kLoopPeriodUs;
    serviceImu();
    const ControlRequest request = command_arbiter.resolve(now_us);
    applyControlRequest(request, now_us);
  }
  serviceMotorHeartbeat(now_us);
  esp_task_wdt_reset();
  delay(1);
}
