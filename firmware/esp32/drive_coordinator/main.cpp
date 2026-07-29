/* SPDX-License-Identifier: GPL-3.0-only */
#include <Arduino.h>
#include <Wire.h>
#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "command_arbiter.h"
#include "controller_fault_clear.h"
#include "differential_drive_mixer.h"
#include "drive_safety.h"
#include "motor_transport_metrics.h"
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
constexpr uint32_t kControlPeriodUs = 10000u;
constexpr uint32_t kMotorHeartbeatUs = 100000u;
constexpr uint32_t kTelemetryPeriodUs = 100000u;
constexpr uint32_t kFeedbackTimeoutUs = 500000u;
constexpr uint32_t kSerialConnectionTimeoutUs = 750000u;
constexpr uint32_t kImuTimeoutUs = 100000u;
constexpr uint32_t kParserTimeoutUs = 50000u;
constexpr uint32_t kAckTimeoutMs = 500u;
constexpr uint8_t kFeedbackCrcThreshold = 3u;
constexpr size_t kMaximumSerialBytesPerPass = 64u;
constexpr size_t kMaximumFeedbackBytesPerPass = 64u;
constexpr uint16_t kWarningFeedbackCrc = 1u << 0;
constexpr uint16_t kWarningHallGlitch = 1u << 1;
constexpr uint16_t kWarningControllerLink = 1u << 2;
constexpr rmt_channel_t kCommandRmtChannel = RMT_CHANNEL_0;
constexpr uint8_t kCommandRmtClockDivider = 80u;
constexpr uint16_t kCommandPulseUnitUs = 80u;
constexpr size_t kCommandRmtItems = GS_SWD_PULSE_FRAME_SYMBOLS + 1u;

static_assert(GS_SWD_PULSE_FRAME_BYTES == GS_ESP_COMMAND_SIZE,
              "pulse transport must carry one exact command frame");
static_assert(kCommandRmtItems <= 64u,
              "pulse command must fit one RMT memory block");
static_assert(kDriveOutputLimit == 250.0f,
              "manual drive output must remain at the validated ceiling");

class SerialTelemetrySink {
public:
  bool queueAcknowledgment(SerialMessageType request_type, uint16_t sequence) {
    SerialFrame response;
    response.type = SerialMessageType::kAcknowledgment;
    response.sequence = sequence;
    response.payload_length = 2u;
    response.payload[0] = static_cast<uint8_t>(request_type);
    response.payload[1] = 0u;
    return queue_.push(response);
  }

  bool queueError(SerialMessageType request_type, uint16_t sequence,
                  uint8_t error_code) {
    SerialFrame response;
    response.type = SerialMessageType::kErrorResponse;
    response.sequence = sequence;
    response.payload_length = 4u;
    response.payload[0] = static_cast<uint8_t>(request_type);
    response.payload[1] = error_code;
    response.payload[2] = 0u;
    response.payload[3] = 0u;
    return queue_.push(response);
  }

  bool queuePayload(SerialMessageType type, uint16_t sequence,
                    const uint8_t *payload, size_t payload_length) {
    if (payload_length > kSerialMaximumPayloadSize ||
        (payload == nullptr && payload_length != 0u)) {
      return false;
    }
    SerialFrame response;
    response.type = type;
    response.sequence = sequence;
    response.payload_length = static_cast<uint16_t>(payload_length);
    for (size_t index = 0u; index < payload_length; ++index) {
      response.payload[index] = payload[index];
    }
    return queue_.push(response);
  }

  void service() {
    if (transmit_offset_ < transmit_length_) {
      writeAvailable();
      return;
    }
    if (transmit_length_ != 0u) {
      transmit_offset_ = 0u;
      transmit_length_ = 0u;
    }
    SerialFrame frame{};
    if (!queue_.pop(frame)) {
      return;
    }
    transmit_length_ = SerialProtocol::encode(frame, transmit_buffer_,
                                              sizeof(transmit_buffer_));
    writeAvailable();
  }

  uint32_t droppedFrames() const { return queue_.droppedFrames(); }

private:
  void writeAvailable() {
    const size_t available = static_cast<size_t>(Serial.availableForWrite());
    const size_t remaining = transmit_length_ - transmit_offset_;
    const size_t write_length = std::min(available, remaining);
    if (write_length == 0u) {
      return;
    }
    Serial.write(&transmit_buffer_[transmit_offset_], write_length);
    transmit_offset_ += write_length;
  }

  SerialFrameQueue queue_{};
  uint8_t transmit_buffer_[kSerialMaximumFrameSize] = {};
  size_t transmit_length_ = 0u;
  size_t transmit_offset_ = 0u;
};

DifferentialDriveConfig driveMixerConfig() {
  DifferentialDriveConfig config;
  config.maximum_command = kDriveOutputLimit;
  config.slew_per_second = 25000.0f;
  return config;
}

HardwareSerial controller_uart(2);
SerialCommandSource serial_source(kParserTimeoutUs);
CommandArbiter command_arbiter;
DifferentialDriveMixer mixer(driveMixerConfig());
DriveSafetyGate safety_gate;
SerialTelemetrySink telemetry_sink;
MotorTransportMetrics transport_metrics;
ControllerFaultClear controller_fault_clear;
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
gs_command_sequencer command_sequencer;

ControlRequest active_request{};
DifferentialDriveOutput latest_mix{};
bool command_transport_ready = false;
bool serial_connected = false;
bool imu_healthy = false;
bool acknowledgment_timeout_latched = false;
bool feedback_crc_latched = false;
uint64_t last_serial_frame_us = 0u;
uint64_t last_feedback_us = 0u;
uint64_t last_imu_sample_us = 0u;
uint64_t next_control_us = 0u;
uint64_t next_motor_heartbeat_us = 0u;
uint64_t next_telemetry_us = 0u;
uint64_t last_motor_applied_us = 0u;
uint16_t timed_out_sequence = 0u;
uint16_t telemetry_sequence = 0u;
uint32_t command_frames = 0u;
uint32_t feedback_frames = 0u;
uint32_t feedback_crc_errors = 0u;
uint8_t consecutive_feedback_crc_errors = 0u;
uint32_t acknowledgment_timeouts = 0u;
uint32_t imu_errors = 0u;
float pitch_deg = 0.0f;
float roll_deg = 0.0f;
int16_t acceleration_raw[3] = {};
ProtocolFirstFault first_fault_snapshot{};
bool first_fault_valid = false;
bool recovery_clear_requested = false;
bool recovery_clear_completed = false;

uint64_t nowMicros() { return static_cast<uint64_t>(esp_timer_get_time()); }

int16_t boundedCommand(float value) {
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<int16_t>(
      std::lround(std::clamp(value, -kDriveOutputLimit, kDriveOutputLimit)));
}

int16_t scaledI16(float value, float scale) {
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<int16_t>(
      std::lround(std::clamp(value * scale, -32768.0f, 32767.0f)));
}

uint16_t scaledU16(float value, float scale) {
  if (!std::isfinite(value) || value <= 0.0f) {
    return 0u;
  }
  return static_cast<uint16_t>(std::lround(std::min(value * scale, 65535.0f)));
}

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

bool zeroReadyAck() {
  return gs_master_feedback_motion_ready(
      &feedback, command_sequencer.in_flight.sequence, command_sequencer.sent);
}

bool feedbackFresh(uint64_t now_us) {
  return feedback_frames != 0u &&
         now_us - last_feedback_us <= kFeedbackTimeoutUs;
}

bool feedbackHealthy(uint64_t now_us) {
  return feedbackFresh(now_us) && gs_master_feedback_runtime_healthy(&feedback);
}

bool leaseActive(const ControlRequest &request, uint64_t now_us) {
  return request.source == CommandSource::kSerial &&
         request.lease_expires_us >= now_us;
}

DriveSafetyInputs safetyInputs(uint64_t now_us, bool lease_active) {
  DriveSafetyInputs inputs;
  inputs.serial_connected = serial_connected;
  inputs.lease_active = lease_active;
  inputs.command_transport_ready = command_transport_ready;
  inputs.feedback_fresh = feedbackFresh(now_us);
  inputs.feedback_runtime_healthy = feedbackHealthy(now_us);
  inputs.exact_zero_acknowledged = zeroReadyAck();
  // Manual lifted-wheel drive bring-up does not depend on chassis orientation
  // or an IMU that may not yet be mounted. Motor/controller feedback remains
  // fully fail-closed.
  inputs.imu_healthy = true;
  inputs.acknowledgment_timed_out = acknowledgment_timeout_latched;
  inputs.feedback_crc_error = feedback_crc_latched;
  inputs.local_disarm = digitalRead(kLocalDisarmPin) == LOW;
  inputs.master_faults = feedback.master_faults;
  inputs.slave_faults = feedback.slave_faults;
  inputs.pitch_deg = 0.0f;
  inputs.roll_deg = 0.0f;
  return inputs;
}

void stopMotion() {
  active_request.linear_velocity = 0.0f;
  active_request.yaw_rate = 0.0f;
  mixer.stop();
  latest_mix = {};
}

void captureFirstFault(uint32_t drive_faults) {
  if (first_fault_valid) {
    return;
  }
  first_fault_snapshot.drive_faults = drive_faults;
  first_fault_snapshot.master_faults = feedback.master_faults;
  first_fault_snapshot.slave_faults = feedback.slave_faults;
  first_fault_snapshot.master_state = feedback.master_state;
  first_fault_snapshot.slave_state = feedback.slave_state;
  first_fault_snapshot.left_hall = feedback.left_hall;
  first_fault_snapshot.right_hall = feedback.right_hall;
  first_fault_snapshot.commanded_left = boundedCommand(latest_mix.left);
  first_fault_snapshot.commanded_right = boundedCommand(latest_mix.right);
  first_fault_snapshot.applied_left = feedback.left_applied;
  first_fault_snapshot.applied_right = feedback.right_applied;
  first_fault_snapshot.esp32_uptime_ms =
      static_cast<uint32_t>(nowMicros() / 1000u);
  first_fault_valid = true;
}

void tripSafety(uint32_t reason) {
  captureFirstFault(safety_gate.faults() | reason);
  safety_gate.disarm(reason);
  serial_source.disconnect();
  command_arbiter.disconnect(CommandSource::kSerial);
  recovery_clear_requested = false;
  recovery_clear_completed = false;
  stopMotion();
}

void serviceImu(uint64_t now_us) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(0x3Bu);
  const uint8_t transmission_status = Wire.endTransmission(false);
  const size_t requested =
      transmission_status == 0u
          ? Wire.requestFrom(static_cast<uint8_t>(kMpuAddress),
                             static_cast<size_t>(6u), true)
          : 0u;
  if (transmission_status != 0u || requested != 6u) {
    ++imu_errors;
    imu_healthy = false;
    return;
  }
  for (size_t axis = 0u; axis < 3u; ++axis) {
    acceleration_raw[axis] =
        static_cast<int16_t>((Wire.read() << 8u) | Wire.read());
  }
  const float x = static_cast<float>(acceleration_raw[0]);
  const float y = static_cast<float>(acceleration_raw[1]);
  const float z = static_cast<float>(acceleration_raw[2]);
  const float magnitude_squared = x * x + y * y + z * z;
  pitch_deg = std::atan2(-x, std::sqrt(y * y + z * z)) * 57.2957795f;
  roll_deg = std::atan2(y, z) * 57.2957795f;
  imu_healthy = std::isfinite(pitch_deg) && std::isfinite(roll_deg) &&
                std::isfinite(magnitude_squared) && magnitude_squared > 1000.0f;
  if (!imu_healthy) {
    ++imu_errors;
    return;
  }
  last_imu_sample_us = now_us;
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
    if (result == GS_PARSE_FRAME &&
        gs_decode_master_feedback(&feedback, frame)) {
      ++feedback_frames;
      consecutive_feedback_crc_errors = 0u;
      last_feedback_us = now_us;
      const uint32_t applied_before =
          transport_metrics.statistics().applied_commands;
      transport_metrics.observe(feedback.accepted_esp_sequence,
                                feedback.forwarded_slave_sequence,
                                feedback.accepted_slave_sequence, now_us);
      if (transport_metrics.statistics().applied_commands > applied_before) {
        last_motor_applied_us = now_us;
      }
      (void)controller_fault_clear.observe(feedback,
                                           command_sequencer.in_flight.sequence,
                                           command_sequencer.sent);
      if (recovery_clear_requested && !controller_fault_clear.pending() &&
          zeroReadyAck()) {
        safety_gate.clearRecoverableFaults(safetyInputs(now_us, true));
        if (safety_gate.faults() == 0u) {
          recovery_clear_requested = false;
          recovery_clear_completed = true;
        }
      }
      if (feedback.master_faults != 0u && !controller_fault_clear.pending()) {
        tripSafety(kDriveFaultMasterController);
      }
      if (feedback.slave_faults != 0u && !controller_fault_clear.pending()) {
        tripSafety(kDriveFaultSlaveController);
      }
      continue;
    }
    if (result == GS_PARSE_BAD_CRC) {
      ++feedback_crc_errors;
      if (consecutive_feedback_crc_errors != UINT8_MAX) {
        ++consecutive_feedback_crc_errors;
      }
      if (consecutive_feedback_crc_errors >= kFeedbackCrcThreshold) {
        feedback_crc_latched = true;
        tripSafety(kDriveFaultFeedbackCrc);
      }
    }
  }
  (void)gs_frame_parser_poll(&feedback_parser,
                             static_cast<uint32_t>(now_us / 1000u));
}

bool queueCapabilities(uint16_t sequence) {
  ProtocolCapabilities capabilities;
  capabilities.dry_run = false;
  capabilities.web_enabled = false;
  capabilities.supported_modes = static_cast<uint8_t>(1u << kManualDriveMode);
  capabilities.control_rate_hz =
      static_cast<uint16_t>(1000000u / kControlPeriodUs);
  capabilities.motor_rate_hz =
      static_cast<uint16_t>(1000000u / kMotorHeartbeatUs);
  capabilities.configuration_keys = 0u;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length = SerialMessageCodec::encodeCapabilities(
      capabilities, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kCapabilities, sequence,
                                     payload, length);
}

bool queueStatus(uint16_t sequence, uint64_t now_us) {
  ProtocolStatus status;
  status.state = safety_gate.faults() != 0u ? 6u
                 : safety_gate.armed()      ? 4u
                                            : 2u;
  status.operating_mode = safety_gate.operatingMode();
  status.active_source = static_cast<uint8_t>(command_arbiter.activeSource());
  status.health_flags =
      (imu_healthy ? 1u << 0u : 0u) | (feedbackFresh(now_us) ? 1u << 2u : 0u) |
      (feedbackHealthy(now_us) ? 1u << 3u : 0u) |
      (safety_gate.armed() ? 1u << 4u : 0u) | (zeroReadyAck() ? 1u << 5u : 0u) |
      (serial_connected ? 1u << 6u : 0u);
  status.faults = safety_gate.faults();
  status.loop_overruns = telemetry_sink.droppedFrames();
  status.rejected_serial_frames = serial_source.rejectedFrames();
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeStatus(status, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kStatus, sequence,
                                     payload, length);
}

bool queueImuTelemetry(uint16_t sequence, uint64_t now_us) {
  ProtocolImuTelemetry value;
  value.address = kMpuAddress;
  value.calibrated = true;
  value.valid = imu_healthy;
  for (size_t axis = 0u; axis < 3u; ++axis) {
    value.acceleration_milli_g[axis] = scaledI16(
        static_cast<float>(acceleration_raw[axis]) / 16384.0f, 1000.0f);
  }
  value.raw_pitch_centi_deg = scaledI16(pitch_deg, 100.0f);
  value.filtered_pitch_centi_deg = scaledI16(pitch_deg, 100.0f);
  value.i2c_errors = imu_errors;
  value.sample_age_us = last_imu_sample_us == 0u
                            ? UINT32_MAX
                            : static_cast<uint32_t>(std::min<uint64_t>(
                                  now_us - last_imu_sample_us, UINT32_MAX));
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeImu(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kImuTelemetry, sequence,
                                     payload, length);
}

bool queueMotorTelemetry(uint16_t sequence) {
  const auto statistics = transport_metrics.statistics();
  ProtocolMotorTelemetry value;
  value.calculated_left = boundedCommand(latest_mix.left);
  value.calculated_right = boundedCommand(latest_mix.right);
  value.applied_left = feedback.left_applied;
  value.applied_right = feedback.right_applied;
  value.sequence = command_sequencer.in_flight.sequence;
  value.flags = safety_gate.armed() ? 1u : 0u;
  value.transmitted_frames = statistics.transmitted_frames;
  value.feedback_frames = feedback_frames;
  value.crc_errors = feedback_crc_errors;
  value.acknowledgment_timeouts = acknowledgment_timeouts;
  value.last_ack_latency_us = statistics.last_ack_latency_us;
  value.maximum_ack_latency_us = statistics.maximum_ack_latency_us;
  value.last_apply_latency_us = statistics.last_apply_latency_us;
  value.maximum_apply_latency_us = statistics.maximum_apply_latency_us;
  value.transmit_rate_centi_hz = scaledU16(statistics.transmit_rate_hz, 100.0f);
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeMotor(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kMotorTelemetry,
                                     sequence, payload, length);
}

bool queueDriveTelemetry(uint16_t sequence, uint64_t now_us) {
  ProtocolDriveTelemetry value;
  value.requested_linear_milli =
      scaledI16(active_request.linear_velocity, 1000.0f);
  value.requested_yaw_milli = scaledI16(active_request.yaw_rate, 1000.0f);
  value.mixed_left = boundedCommand(latest_mix.target_left);
  value.mixed_right = boundedCommand(latest_mix.target_right);
  value.commanded_left = boundedCommand(latest_mix.left);
  value.commanded_right = boundedCommand(latest_mix.right);
  value.applied_left = feedback.left_applied;
  value.applied_right = feedback.right_applied;
  value.safety_faults = safety_gate.faults();
  value.active_source = static_cast<uint8_t>(command_arbiter.activeSource());
  value.operating_mode = safety_gate.operatingMode();
  value.arm_state = safety_gate.armed() ? 1u : 0u;
  value.flags = (safety_gate.neutralObserved() ? 1u << 0u : 0u) |
                (feedbackFresh(now_us) ? 1u << 1u : 0u) |
                (zeroReadyAck() ? 1u << 2u : 0u) |
                (serial_connected ? 1u << 3u : 0u);
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeDrive(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kDriveTelemetry,
                                     sequence, payload, length);
}

bool queueOdometry(uint16_t sequence, uint64_t now_us) {
  ProtocolOdometry value;
  value.left = feedback.left_odometer;
  value.right = feedback.right_odometer;
  value.velocity_milli = 0;
  value.timestamp_us = now_us;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeOdometry(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kOdometry, sequence,
                                     payload, length);
}

bool queueFaults(uint16_t sequence) {
  ProtocolFaults value;
  value.balance = safety_gate.faults();
  value.master = feedback.master_faults;
  value.slave = feedback.slave_faults;
  value.feedback_health = static_cast<uint32_t>(feedback.status_flags) |
                          static_cast<uint32_t>(feedback.motor_status_flags)
                              << 8u;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeFaults(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kActiveFaults, sequence,
                                     payload, length);
}

bool queueControllerTelemetry(uint16_t sequence) {
  ProtocolControllerTelemetry value;
  value.master_state = feedback.master_state;
  value.slave_state = feedback.slave_state;
  value.status_flags = feedback.status_flags;
  value.motor_status_flags = feedback.motor_status_flags;
  value.master_command_age_ms = feedback.master_command_age_ms;
  value.slave_feedback_age_ms = feedback.slave_feedback_age_ms;
  value.slave_command_age_ms = feedback.slave_command_age_ms;
  value.left_hall = feedback.left_hall;
  value.right_hall = feedback.right_hall;
  value.left_compare_offset = feedback.left_compare_offset;
  value.right_compare_offset = feedback.right_compare_offset;
  value.remote_rx_bytes = feedback.remote_rx_bytes;
  value.remote_valid_frames = feedback.remote_valid_frames;
  value.remote_invalid_frames = feedback.remote_invalid_frames;
  value.remote_framing_errors = feedback.remote_framing_errors;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeController(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kControllerTelemetry,
                                     sequence, payload, length);
}

bool queueResilienceTelemetry(uint16_t sequence) {
  ProtocolResilienceTelemetry value;
  value.warning_flags = static_cast<uint16_t>(
      (consecutive_feedback_crc_errors != 0u ? kWarningFeedbackCrc : 0u) |
      ((feedback.left_hall_glitch_count != 0u ||
        feedback.right_hall_glitch_count != 0u)
           ? kWarningHallGlitch
           : 0u) |
      ((feedback.slave_feedback_invalid_frames != 0u ||
        feedback.slave_feedback_framing_errors != 0u ||
        feedback.slave_command_invalid_frames != 0u ||
        feedback.slave_command_framing_errors != 0u)
           ? kWarningControllerLink
           : 0u));
  value.feedback_crc_streak = consecutive_feedback_crc_errors;
  value.feedback_crc_threshold = kFeedbackCrcThreshold;
  value.feedback_crc_total = feedback_crc_errors;
  value.left_hall_glitches = feedback.left_hall_glitch_count;
  value.right_hall_glitches = feedback.right_hall_glitch_count;
  value.slave_feedback_invalid_frames = feedback.slave_feedback_invalid_frames;
  value.slave_feedback_framing_errors = feedback.slave_feedback_framing_errors;
  value.slave_command_invalid_frames = feedback.slave_command_invalid_frames;
  value.slave_command_framing_errors = feedback.slave_command_framing_errors;
  value.first_fault = first_fault_snapshot;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeResilience(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kResilienceTelemetry,
                                     sequence, payload, length);
}

bool decodeMovement(const SerialFrame &frame, MovementCommand &movement) {
  return MovementCommandCodec::decode(frame.payload.data(),
                                      frame.payload_length, movement);
}

bool rejectUnsafeMotion(const SerialFrame &frame, bool nonzero) {
  if (!nonzero ||
      (safety_gate.armed() && safety_gate.faults() == 0u &&
       !recovery_clear_requested && !controller_fault_clear.pending())) {
    return false;
  }
  serial_source.disconnect();
  (void)telemetry_sink.queueError(
      frame.type, frame.sequence,
      static_cast<uint8_t>(SerialErrorCode::kUnsafeState));
  return true;
}

bool isMovementFrame(const SerialFrame &frame) {
  return frame.type == SerialMessageType::kSetLinearVelocity ||
         frame.type == SerialMessageType::kSetYawRate ||
         frame.type == SerialMessageType::kSetVelocityAndYaw ||
         frame.type == SerialMessageType::kSetDirectMotor ||
         (frame.type == SerialMessageType::kHeartbeat &&
          frame.payload_length != 0u);
}

bool handleAcceptedFrame(const SerialFrame &frame, uint64_t now_us) {
  if (frame.type == SerialMessageType::kHello) {
    serial_connected = true;
    last_serial_frame_us = now_us;
    command_arbiter.disconnect(CommandSource::kSerial);
    safety_gate.onConnectionEstablished();
    safety_gate.setOperatingMode(0u);
    stopMotion();
    (void)queueCapabilities(frame.sequence);
    return true;
  }
  if (!serial_connected) {
    tripSafety(kDriveFaultSerialDisconnected);
    (void)telemetry_sink.queueError(
        frame.type, frame.sequence,
        static_cast<uint8_t>(SerialErrorCode::kUnsafeState));
    return false;
  }

  last_serial_frame_us = now_us;
  switch (frame.type) {
  case SerialMessageType::kCapabilities:
    (void)queueCapabilities(frame.sequence);
    return true;
  case SerialMessageType::kStatus:
    (void)queueStatus(frame.sequence, now_us);
    return true;
  case SerialMessageType::kImuTelemetry:
    (void)queueImuTelemetry(frame.sequence, now_us);
    return true;
  case SerialMessageType::kMotorTelemetry:
    (void)queueMotorTelemetry(frame.sequence);
    return true;
  case SerialMessageType::kDriveTelemetry:
    (void)queueDriveTelemetry(frame.sequence, now_us);
    return true;
  case SerialMessageType::kOdometry:
    (void)queueOdometry(frame.sequence, now_us);
    return true;
  case SerialMessageType::kActiveFaults:
    (void)queueFaults(frame.sequence);
    return true;
  case SerialMessageType::kControllerTelemetry:
    (void)queueControllerTelemetry(frame.sequence);
    return true;
  case SerialMessageType::kResilienceTelemetry:
    (void)queueResilienceTelemetry(frame.sequence);
    return true;
  case SerialMessageType::kSetOperatingMode:
    safety_gate.setOperatingMode(frame.payload[0]);
    stopMotion();
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  case SerialMessageType::kSetLinearVelocity:
  case SerialMessageType::kSetYawRate:
  case SerialMessageType::kSetVelocityAndYaw: {
    MovementCommand movement{};
    if (!decodeMovement(frame, movement)) {
      tripSafety(kDriveFaultMalformedCommand);
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kMalformed));
      return false;
    }
    const bool nonzero =
        movement.linear_velocity_milli != 0 || movement.yaw_rate_milli != 0;
    if (rejectUnsafeMotion(frame, nonzero)) {
      return false;
    }
    safety_gate.observeDemand(
        static_cast<float>(movement.linear_velocity_milli) / 1000.0f,
        static_cast<float>(movement.yaw_rate_milli) / 1000.0f);
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  }
  case SerialMessageType::kSetDirectMotor: {
    DirectMotorCommand direct{};
    if (!DirectMotorCommandCodec::decode(frame.payload.data(),
                                         frame.payload_length, direct)) {
      tripSafety(kDriveFaultMalformedCommand);
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kMalformed));
      return false;
    }
    if (rejectUnsafeMotion(frame, direct.left != 0 || direct.right != 0)) {
      return false;
    }
    safety_gate.observeDemand(static_cast<float>(direct.left),
                              static_cast<float>(direct.right));
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  }
  case SerialMessageType::kHeartbeat:
    if (frame.payload_length != 0u) {
      MovementCommand movement{};
      if (!decodeMovement(frame, movement)) {
        tripSafety(kDriveFaultMalformedCommand);
        (void)telemetry_sink.queueError(
            frame.type, frame.sequence,
            static_cast<uint8_t>(SerialErrorCode::kMalformed));
        return false;
      }
      const bool nonzero =
          movement.linear_velocity_milli != 0 || movement.yaw_rate_milli != 0;
      if (rejectUnsafeMotion(frame, nonzero)) {
        return false;
      }
      safety_gate.observeDemand(
          static_cast<float>(movement.linear_velocity_milli) / 1000.0f,
          static_cast<float>(movement.yaw_rate_milli) / 1000.0f);
    }
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  case SerialMessageType::kArm: {
    const DriveSafetyInputs inputs = safetyInputs(now_us, true);
    if (safety_gate.requestArm(inputs)) {
      if (recovery_clear_completed) {
        first_fault_snapshot = {};
        first_fault_valid = false;
        recovery_clear_completed = false;
      }
      (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    } else {
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kUnsafeState));
    }
    return true;
  }
  case SerialMessageType::kStop:
    safety_gate.disarm();
    command_arbiter.disconnect(CommandSource::kSerial);
    stopMotion();
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  case SerialMessageType::kDisarm:
  case SerialMessageType::kEmergencyStop:
    safety_gate.disarm();
    command_arbiter.disconnect(CommandSource::kSerial);
    stopMotion();
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  case SerialMessageType::kClearFault: {
    if (!safety_gate.neutralObserved()) {
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kUnsafeState));
      return false;
    }
    stopMotion();
    recovery_clear_requested = true;
    recovery_clear_completed = false;
    if (feedback.master_faults != 0u || feedback.slave_faults != 0u) {
      controller_fault_clear.request();
    }
    acknowledgment_timeout_latched = false;
    feedback_crc_latched = false;
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  }
  default:
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return true;
  }
}

void rejectSerialFrame(SerialParseResult result) {
  if (result == SerialParseResult::kInvalidCrc) {
    tripSafety(kDriveFaultMalformedCommand);
    (void)telemetry_sink.queueError(
        SerialMessageType::kHello, 0u,
        static_cast<uint8_t>(SerialErrorCode::kInvalidCrc));
  } else if (result == SerialParseResult::kMalformed) {
    tripSafety(kDriveFaultMalformedCommand);
    (void)telemetry_sink.queueError(serial_source.lastFrameType(),
                                    serial_source.lastFrameSequence(),
                                    serial_source.lastErrorCode());
  } else if (result == SerialParseResult::kTimeout) {
    tripSafety(kDriveFaultMalformedCommand);
  }
  command_arbiter.disconnect(CommandSource::kSerial);
  stopMotion();
}

void serviceNorthboundSerial(uint64_t now_us) {
  size_t serviced = 0u;
  while (Serial.available() > 0 && serviced < kMaximumSerialBytesPerPass) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    ++serviced;
    const SerialParseResult result = serial_source.feed(byte, now_us);
    if (result == SerialParseResult::kFrame) {
      if (handleAcceptedFrame(serial_source.lastFrame(), now_us)) {
        if (isMovementFrame(serial_source.lastFrame())) {
          ControlRequest request{};
          if (serial_source.latest(request)) {
            command_arbiter.submit(request);
          }
        }
      }
    } else if (result != SerialParseResult::kIncomplete) {
      rejectSerialFrame(result);
    }
  }
}

void serviceSerialTimeout(uint64_t now_us) {
  if (!serial_connected || last_serial_frame_us == 0u ||
      now_us - last_serial_frame_us <= kSerialConnectionTimeoutUs) {
    return;
  }
  serial_connected = false;
  serial_source.disconnect();
  command_arbiter.disconnect(CommandSource::kSerial);
  safety_gate.onConnectionLost();
  tripSafety(kDriveFaultSerialDisconnected);
}

void runControlLoop(uint64_t now_us) {
  serviceImu(now_us);
  command_arbiter.setLocalDisarm(digitalRead(kLocalDisarmPin) == LOW);
  const ControlRequest request = command_arbiter.resolve(now_us);
  active_request = request;
  const bool lease_active = leaseActive(request, now_us);
  DriveSafetyInputs inputs = safetyInputs(now_us, lease_active);

  if (request.set_operating_mode) {
    safety_gate.setOperatingMode(request.operating_mode);
  }
  if (request.emergency_stop || request.disarm) {
    safety_gate.disarm();
  }
  const uint32_t faults_before_evaluation = safety_gate.faults();
  safety_gate.evaluate(inputs);
  if (safety_gate.faults() != faults_before_evaluation &&
      safety_gate.faults() != 0u) {
    tripSafety(safety_gate.faults());
  }
  inputs = safetyInputs(now_us, lease_active);

  if (!safety_gate.outputEnabled(inputs)) {
    mixer.stop();
    latest_mix = {};
    return;
  }

  latest_mix = mixer.update(request.linear_velocity, request.yaw_rate,
                            static_cast<float>(kControlPeriodUs) / 1000000.0f);
  if (!latest_mix.valid || std::fabs(latest_mix.left) > kDriveOutputLimit ||
      std::fabs(latest_mix.right) > kDriveOutputLimit) {
    tripSafety(kDriveFaultMalformedCommand);
  }
}

void serviceMotorHeartbeat(uint64_t now_us) {
  if (now_us < next_motor_heartbeat_us || !command_transport_ready) {
    return;
  }
  next_motor_heartbeat_us = now_us + kMotorHeartbeatUs;
  const DriveSafetyInputs inputs =
      safetyInputs(now_us, leaseActive(active_request, now_us));
  gs_esp_command requested{};
  if (safety_gate.outputEnabled(inputs)) {
    requested.master_flags = GS_COMMAND_DIRECT_LR;
    requested.speed = boundedCommand(latest_mix.left);
    requested.steer = boundedCommand(latest_mix.right);
  } else if (safety_gate.zeroEstablishmentAllowed(inputs)) {
    // Bring both motor controllers to READY with an exact zero command. This
    // cannot energize either bridge, but it lets their end-to-end zero
    // acknowledgment satisfy the full ARM gate.
    requested.master_flags = GS_COMMAND_DIRECT_LR;
    requested.speed = 0;
    requested.steer = 0;
  } else {
    requested.master_flags = GS_COMMAND_DISABLE;
    requested.slave_flags = GS_COMMAND_DISABLE;
    requested.speed = 0;
    requested.steer = 0;
  }
  controller_fault_clear.apply(requested);

  const gs_esp_command *selected =
      gs_command_sequencer_select(&command_sequencer, &requested, exactAck(),
                                  static_cast<uint32_t>(now_us / 1000u));
  uint8_t frame[GS_ESP_COMMAND_SIZE] = {};
  if (selected == nullptr || !gs_encode_esp_command(frame, selected) ||
      !transmitCommandFrame(frame)) {
    command_transport_ready = false;
    tripSafety(kDriveFaultTransportUnavailable);
    return;
  }
  ++command_frames;
  transport_metrics.recordTransmission(now_us);
  transport_metrics.beginCommand(selected->sequence, now_us);
  if (gs_command_sequencer_ack_expired(&command_sequencer, exactAck(),
                                       static_cast<uint32_t>(now_us / 1000u),
                                       kAckTimeoutMs) &&
      timed_out_sequence != command_sequencer.in_flight.sequence) {
    timed_out_sequence = command_sequencer.in_flight.sequence;
    ++acknowledgment_timeouts;
    transport_metrics.recordTimeout();
    acknowledgment_timeout_latched = true;
    tripSafety(kDriveFaultAcknowledgmentTimeout);
  }
}

void queueTelemetryStream(uint64_t now_us) {
  if (!serial_connected || now_us < next_telemetry_us) {
    return;
  }
  next_telemetry_us = now_us + kTelemetryPeriodUs;
  const uint16_t sequence = ++telemetry_sequence;
  (void)queueStatus(sequence, now_us);
  (void)queueDriveTelemetry(sequence, now_us);
  (void)queueMotorTelemetry(sequence);
  (void)queueOdometry(sequence, now_us);
  (void)queueFaults(sequence);
  (void)queueControllerTelemetry(sequence);
  (void)queueResilienceTelemetry(sequence);
}

} // namespace

void setup() {
  pinMode(kLocalDisarmPin, INPUT_PULLUP);
  Serial.begin(kConsoleBaud);
  controller_uart.begin(kControllerBaud, SERIAL_8N1, kControllerRx, -1);
  Wire.setTimeOut(2u);
  Wire.begin(kMpuSda, kMpuScl, 400000u);
  Wire.beginTransmission(kMpuAddress);
  Wire.write(0x6Bu);
  Wire.write(0u);
  imu_healthy = Wire.endTransmission(true) == 0u;
  gs_frame_parser_init(&feedback_parser, GS_PARTIAL_FRAME_TIMEOUT_MS);
  gs_command_sequencer_init(&command_sequencer);
  command_transport_ready = initializeCommandTransport();
  if (!command_transport_ready) {
    safety_gate.disarm(kDriveFaultTransportUnavailable);
  }
  const uint64_t now_us = nowMicros();
  next_control_us = now_us;
  next_motor_heartbeat_us = now_us;
  next_telemetry_us = now_us + kTelemetryPeriodUs;
  esp_task_wdt_init(2u, true);
  esp_task_wdt_add(nullptr);
}

void loop() {
  const uint64_t now_us = nowMicros();
  telemetry_sink.service();
  serviceNorthboundSerial(now_us);
  serviceSerialTimeout(now_us);
  serviceFeedback(now_us);
  if (now_us >= next_control_us) {
    next_control_us = now_us + kControlPeriodUs;
    runControlLoop(now_us);
  }
  serviceMotorHeartbeat(now_us);
  queueTelemetryStream(now_us);
  telemetry_sink.service();
  esp_task_wdt_reset();
  delay(1);
}
