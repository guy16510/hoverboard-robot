/* SPDX-License-Identifier: GPL-3.0-only */
#include <Arduino.h>
#include <Wire.h>
#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "balance_configuration.h"
#include "balance_controller.h"
#include "balance_state_machine.h"
#include "balance_user_config.h"
#include "command_arbiter.h"
#include "complementary_pitch_estimator.h"
#include "control_runtime.h"
#include "loop_metrics.h"
#include "motor_transport_metrics.h"
#include "mpu6050.h"
#include "serial_command_source.h"
#include "serial_messages.h"

#if GS_ENABLE_WEB_CONTROL
#include "web_command_source.h"
#include "web_controller_adapter.h"
#endif

extern "C" {
#include "gausstop_swd_pulse.h"
#include "gs_frame_parser.h"
#include "gs_protocol.h"
}

#ifndef GS_BALANCE_DRY_RUN
#define GS_BALANCE_DRY_RUN 1
#endif

#ifndef GS_ENABLE_WEB_CONTROL
#define GS_ENABLE_WEB_CONTROL 0
#endif

#ifndef GS_STAGE5_TRANSPORT_ONLY
#define GS_STAGE5_TRANSPORT_ONLY 0
#endif

#if GS_ENABLE_WEB_CONTROL
#ifndef GS_WIFI_SSID
#define GS_WIFI_SSID ""
#endif
#ifndef GS_WIFI_PASSWORD
#define GS_WIFI_PASSWORD ""
#endif
#endif

namespace {
using namespace gs::balance;

constexpr uint32_t kConsoleBaud = 115200u;
constexpr uint32_t kControllerBaud = 19200u;
constexpr int kControllerRx = 35;
constexpr int kControllerTx = 17;
constexpr int kMpuSda = 21;
constexpr int kMpuScl = 22;
constexpr int kLocalDisarmPin = 0;
constexpr uint32_t kControlPeriodUs = 5000u;
constexpr uint32_t kMotorHeartbeatUs = 100000u;
constexpr uint32_t kTelemetryPeriodUs = 100000u;
constexpr uint32_t kFeedbackTimeoutUs = 500000u;
constexpr uint32_t kAckTimeoutMs = 500u;
constexpr uint32_t kSerialParserTimeoutUs = 50000u;
constexpr uint32_t kTaskWatchdogTimeoutSeconds = 2u;
constexpr size_t kMaximumFeedbackBytesPerPass = 64u;
constexpr size_t kMaximumSerialBytesPerPass = 64u;
constexpr rmt_channel_t kCommandRmtChannel = RMT_CHANNEL_0;
constexpr uint8_t kCommandRmtClockDivider = 80u;
constexpr uint16_t kCommandPulseUnitUs = 80u;
constexpr size_t kCommandRmtItems = GS_SWD_PULSE_FRAME_SYMBOLS + 1u;

static_assert(GS_SWD_PULSE_FRAME_BYTES == GS_ESP_COMMAND_SIZE,
              "pulse transport must carry one exact command frame");
static_assert(kCommandRmtItems <= 64u,
              "pulse command must fit one RMT memory block");
static_assert(GS_BALANCE_DRY_RUN == 0 || GS_BALANCE_DRY_RUN == 1,
              "GS_BALANCE_DRY_RUN must be zero or one");
static_assert(GS_STAGE5_TRANSPORT_ONLY == 0 ||
                  GS_STAGE5_TRANSPORT_ONLY == 1,
              "GS_STAGE5_TRANSPORT_ONLY must be zero or one");
static_assert(GS_STAGE5_TRANSPORT_ONLY == 0 || GS_BALANCE_DRY_RUN == 0,
              "Stage 5 transport image must permit bounded direct output");

class EspClock final : public IClock {
public:
  uint64_t nowMicros() const override {
    return static_cast<uint64_t>(esp_timer_get_time());
  }
};

class WireMpu6050Bus final : public IMpu6050Bus {
public:
  explicit WireMpu6050Bus(TwoWire &wire) : wire_(wire) {}

  bool begin(uint32_t frequency_hz) override {
    wire_.setTimeOut(2u);
    return wire_.begin(kMpuSda, kMpuScl, frequency_hz);
  }

  bool read(uint8_t address, uint8_t reg, uint8_t *bytes,
            size_t length) override {
    wire_.beginTransmission(address);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0u) {
      return false;
    }
    const size_t received = wire_.requestFrom(address, length, true);
    if (received != length) {
      return false;
    }
    for (size_t index = 0u; index < length; ++index) {
      bytes[index] = static_cast<uint8_t>(wire_.read());
    }
    return true;
  }

  bool write(uint8_t address, uint8_t reg, uint8_t value) override {
    wire_.beginTransmission(address);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission(true) == 0u;
  }

private:
  TwoWire &wire_;
};

class SwdMotorCommandSink final : public IMotorCommandSink {
public:
  explicit SwdMotorCommandSink(IClock &clock) : clock_(clock) {}

  void write(const MotorCommand &command) override {
    latest_ = command;
    latest_.created_us = clock_.nowMicros();
  }

  const MotorCommand &latest() const { return latest_; }

private:
  IClock &clock_;
  MotorCommand latest_{};
};

class SerialTelemetrySink final : public ITelemetrySink {
public:
  void publish(const PitchEstimate &estimate,
               const BalanceOutput &output) override {
    estimate_ = estimate;
    output_ = output;
  }

  const PitchEstimate &estimate() const { return estimate_; }
  const BalanceOutput &output() const { return output_; }

  bool queueAcknowledgment(SerialMessageType request_type, uint16_t sequence) {
    SerialFrame response;
    response.type = SerialMessageType::kAcknowledgment;
    response.sequence = sequence;
    response.payload_length = 2u;
    response.payload[0] = static_cast<uint8_t>(request_type);
    response.payload[1] = 0u;
    return queue(response);
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
    return queue(response);
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
    return queue(response);
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

  bool busy() const {
    return transmit_offset_ < transmit_length_ || queue_.size() != 0u;
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

  bool queue(const SerialFrame &frame) { return queue_.push(frame); }

  PitchEstimate estimate_{};
  BalanceOutput output_{};
  SerialFrameQueue queue_{};
  uint8_t transmit_buffer_[kSerialMaximumFrameSize] = {};
  size_t transmit_length_ = 0u;
  size_t transmit_offset_ = 0u;
};

EspClock clock_source;
WireMpu6050Bus mpu_bus(Wire);
Mpu6050Config mpu_config;
Mpu6050Imu imu(mpu_bus, clock_source, mpu_config);
ComplementaryPitchEstimator estimator(ComplementaryPitchConfig{});
CascadedBalanceConfig initialBalanceConfig() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  config.upright_offset_deg = user_config::kUprightMountingOffsetDeg;
  return config;
}
BalanceStateConfig initialStateConfig() {
  BalanceStateConfig config;
  config.fall_angle_deg = user_config::kFallAngleDeg;
  return config;
}
CascadedBalanceConfig balance_config = initialBalanceConfig();
CascadedBalanceController controller(balance_config);
SwdMotorCommandSink motor_sink(clock_source);
ControlRuntime control_runtime(controller, motor_sink, GS_BALANCE_DRY_RUN != 0);
BalanceStateMachine state_machine(initialStateConfig());
CommandArbiter command_arbiter;
SerialCommandSource serial_source(kSerialParserTimeoutUs);
SerialTelemetrySink telemetry_sink;
LoopMetrics loop_metrics(kControlPeriodUs);
MotorTransportMetrics transport_metrics;
#if GS_ENABLE_WEB_CONTROL
WebCommandSource web_source;
WebControllerAdapter web_adapter(web_source);
#endif

HardwareSerial controller_uart(2);
gs_frame_parser feedback_parser;
gs_master_feedback feedback{};
gs_command_sequencer command_sequencer;

ImuSample latest_sample{};
bool imu_started = false;
bool estimator_started = false;
bool command_transport_ready = false;
uint64_t next_control_us = 0u;
uint64_t previous_control_us = 0u;
uint64_t next_motor_heartbeat_us = 0u;
uint64_t next_telemetry_us = 0u;
uint64_t last_feedback_us = 0u;
uint64_t last_motor_applied_us = 0u;
uint32_t command_frames = 0u;
uint32_t feedback_frames = 0u;
uint32_t feedback_crc_errors = 0u;
uint32_t ack_timeouts = 0u;
uint32_t loop_overruns = 0u;
uint16_t timed_out_sequence = 0u;
uint8_t operating_mode = 0u;
float latest_wheel_velocity = 0.0f;
bool controller_healthy = true;
bool motor_transport_fault = false;
bool scheduler_fault = false;
SafetySnapshot latest_safety{};

const char *stateName(BalanceState state);

bool feedbackFaulted() {
  return feedback_frames != 0u &&
         !gs_master_feedback_runtime_healthy(&feedback);
}

bool feedbackFresh(uint64_t now_us) {
  return feedback_frames != 0u &&
         now_us - last_feedback_us <= kFeedbackTimeoutUs;
}

bool exactAck() {
  return gs_master_feedback_exact_ack(
      &feedback, command_sequencer.in_flight.sequence, command_sequencer.sent);
}

bool zeroReadyAck() {
  return gs_master_feedback_motion_ready(
      &feedback, command_sequencer.in_flight.sequence, command_sequencer.sent);
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

gs_esp_command requestedMotorCommand() {
  gs_esp_command command{};
  const MotorCommand &motor = motor_sink.latest();
  if (GS_BALANCE_DRY_RUN != 0 || scheduler_fault) {
    command.master_flags = GS_COMMAND_DISABLE;
    command.slave_flags = GS_COMMAND_DISABLE;
    return command;
  }
#if GS_STAGE5_TRANSPORT_ONLY
  command.master_flags = GS_COMMAND_DIRECT_LR;
  if (operating_mode != 3u || !motor.enabled) {
    command.speed = 0;
    command.steer = 0;
    return command;
  }
#else
  if (!motor.enabled) {
    command.master_flags = GS_COMMAND_DISABLE;
    command.slave_flags = GS_COMMAND_DISABLE;
    return command;
  }
#endif
  command.master_flags = GS_COMMAND_DIRECT_LR;
  command.speed = static_cast<int16_t>(motor.left);
  command.steer = static_cast<int16_t>(motor.right);
  return command;
}

void serviceMotorHeartbeat(uint64_t now_us) {
  if (now_us < next_motor_heartbeat_us || !command_transport_ready) {
    return;
  }
  next_motor_heartbeat_us = now_us + kMotorHeartbeatUs;
  const gs_esp_command requested = requestedMotorCommand();
  const gs_esp_command *selected =
      gs_command_sequencer_select(&command_sequencer, &requested, exactAck(),
                                  static_cast<uint32_t>(now_us / 1000u));
  uint8_t frame[GS_ESP_COMMAND_SIZE] = {};
  if (selected == nullptr || !gs_encode_esp_command(frame, selected) ||
      !transmitCommandFrame(frame)) {
    motor_transport_fault = true;
    state_machine.disarm();
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
    ++ack_timeouts;
    transport_metrics.recordTimeout();
    motor_transport_fault = true;
    state_machine.disarm();
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
    if (result == GS_PARSE_FRAME &&
        gs_decode_master_feedback(&feedback, frame)) {
      ++feedback_frames;
      last_feedback_us = now_us;
      const uint32_t applied_before =
          transport_metrics.statistics().applied_commands;
      transport_metrics.observe(feedback.accepted_esp_sequence,
                                feedback.forwarded_slave_sequence,
                                feedback.accepted_slave_sequence, now_us);
      if (transport_metrics.statistics().applied_commands > applied_before) {
        last_motor_applied_us = now_us;
      }
      continue;
    }
    if (result == GS_PARSE_BAD_CRC) {
      ++feedback_crc_errors;
    }
  }
  (void)gs_frame_parser_poll(&feedback_parser,
                             static_cast<uint32_t>(now_us / 1000u));
}

int16_t scaledI16(float value, float scale) {
  if (!std::isfinite(value)) {
    return 0;
  }
  const float scaled = std::clamp(value * scale, -32768.0f, 32767.0f);
  return static_cast<int16_t>(std::lround(scaled));
}

int32_t scaledI32(float value, float scale) {
  if (!std::isfinite(value)) {
    return 0;
  }
  const double scaled = std::clamp(static_cast<double>(value) * scale,
                                   -2147483648.0, 2147483647.0);
  return static_cast<int32_t>(std::llround(scaled));
}

uint16_t scaledU16(float value, float scale) {
  if (!std::isfinite(value) || value <= 0.0f) {
    return 0u;
  }
  const float scaled = std::min(value * scale, 65535.0f);
  return static_cast<uint16_t>(std::lround(scaled));
}

float measuredSampleRate() {
  const auto &diagnostics = imu.diagnostics();
  if (diagnostics.valid_samples <= 1u ||
      diagnostics.last_sample_us <= diagnostics.first_sample_us) {
    return 0.0f;
  }
  return static_cast<float>(diagnostics.valid_samples - 1u) * 1000000.0f /
         static_cast<float>(diagnostics.last_sample_us -
                            diagnostics.first_sample_us);
}

uint32_t balanceFaultMask() {
  uint32_t faults = 0u;
  faults |= latest_safety.imu_healthy ? 0u : 1u << 0u;
  faults |= latest_safety.motor_feedback_healthy ? 0u : 1u << 1u;
  faults |= latest_safety.loop_healthy ? 0u : 1u << 2u;
  faults |= latest_safety.controller_fault ? 1u << 3u : 0u;
  faults |= state_machine.state() == BalanceState::kFallen ? 1u << 4u : 0u;
  faults |= motor_transport_fault ? 1u << 5u : 0u;
  return faults;
}

bool queueCapabilities(uint16_t sequence) {
  ProtocolCapabilities capabilities;
  capabilities.dry_run = GS_BALANCE_DRY_RUN != 0;
  capabilities.web_enabled = GS_ENABLE_WEB_CONTROL != 0;
  capabilities.control_rate_hz =
      static_cast<uint16_t>(1000000u / kControlPeriodUs);
  capabilities.motor_rate_hz =
      static_cast<uint16_t>(1000000u / kMotorHeartbeatUs);
  capabilities.configuration_keys =
      static_cast<uint16_t>(BalanceConfigKey::kCount);
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length = SerialMessageCodec::encodeCapabilities(
      capabilities, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kCapabilities, sequence,
                                     payload, length);
}

bool queueStatus(uint16_t sequence) {
  ProtocolStatus status;
  status.state = static_cast<uint8_t>(state_machine.state());
  status.operating_mode = operating_mode;
  status.active_source = static_cast<uint8_t>(command_arbiter.activeSource());
  status.health_flags = (latest_safety.imu_healthy ? 1u << 0u : 0u) |
                        (latest_safety.calibrated ? 1u << 1u : 0u) |
                        (latest_safety.motor_feedback_healthy ? 1u << 2u : 0u) |
                        (latest_safety.loop_healthy ? 1u << 3u : 0u) |
                        (state_machine.outputEnabled() ? 1u << 4u : 0u) |
                        (GS_BALANCE_DRY_RUN != 0 ? 1u << 5u : 0u) |
                        (controller_healthy ? 1u << 6u : 0u);
  status.faults = balanceFaultMask();
  status.loop_overruns = loop_overruns;
  status.rejected_serial_frames = serial_source.rejectedFrames();
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeStatus(status, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kStatus, sequence,
                                     payload, length);
}

bool queueImuTelemetry(uint16_t sequence, uint64_t now_us) {
  const auto &diagnostics = imu.diagnostics();
  const auto &pitch = telemetry_sink.estimate();
  ProtocolImuTelemetry value;
  value.address = imu.address();
  value.calibrated = diagnostics.calibration_complete;
  value.valid = pitch.valid;
  value.acceleration_milli_g[0] = scaledI16(latest_sample.accel_g.x, 1000.0f);
  value.acceleration_milli_g[1] = scaledI16(latest_sample.accel_g.y, 1000.0f);
  value.acceleration_milli_g[2] = scaledI16(latest_sample.accel_g.z, 1000.0f);
  value.gyroscope_centi_dps[0] = scaledI16(diagnostics.raw_gyro_dps.x, 100.0f);
  value.gyroscope_centi_dps[1] = scaledI16(diagnostics.raw_gyro_dps.y, 100.0f);
  value.gyroscope_centi_dps[2] = scaledI16(diagnostics.raw_gyro_dps.z, 100.0f);
  value.raw_pitch_centi_deg = scaledI16(pitch.raw_pitch_deg, 100.0f);
  value.filtered_pitch_centi_deg = scaledI16(pitch.filtered_pitch_deg, 100.0f);
  value.pitch_rate_centi_dps = scaledI16(pitch.pitch_rate_dps, 100.0f);
  value.sample_rate_centi_hz = scaledU16(measuredSampleRate(), 100.0f);
  value.i2c_errors = diagnostics.i2c_errors;
  value.missed_samples = diagnostics.missed_samples;
  value.sample_age_us = LoopMetrics::ageUs(now_us, latest_sample.timestamp_us);
  value.calibration_samples = diagnostics.calibration_samples;
  value.gyro_bias_centi_dps[0] = scaledI16(diagnostics.gyro_bias_dps.x, 100.0f);
  value.gyro_bias_centi_dps[1] = scaledI16(diagnostics.gyro_bias_dps.y, 100.0f);
  value.gyro_bias_centi_dps[2] = scaledI16(diagnostics.gyro_bias_dps.z, 100.0f);
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeImu(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kImuTelemetry, sequence,
                                     payload, length);
}

bool queueMotorTelemetry(uint16_t sequence) {
  const auto &output = telemetry_sink.output();
  const auto statistics = transport_metrics.statistics();
  ProtocolMotorTelemetry value;
  value.calculated_left = scaledI16(output.left, 1.0f);
  value.calculated_right = scaledI16(output.right, 1.0f);
  value.applied_left = feedback.left_applied;
  value.applied_right = feedback.right_applied;
  value.sequence = command_sequencer.in_flight.sequence;
  value.flags = motor_sink.latest().enabled ? 1u : 0u;
  value.transmitted_frames = statistics.transmitted_frames;
  value.feedback_frames = feedback_frames;
  value.crc_errors = feedback_crc_errors;
  value.acknowledgment_timeouts = ack_timeouts;
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

bool queueOdometry(uint16_t sequence, uint64_t now_us) {
  ProtocolOdometry value;
  value.left = feedback.left_odometer;
  value.right = feedback.right_odometer;
  value.velocity_milli = scaledI32(latest_wheel_velocity, 1000.0f);
  value.timestamp_us = now_us;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length =
      SerialMessageCodec::encodeOdometry(value, payload, sizeof(payload));
  return telemetry_sink.queuePayload(SerialMessageType::kOdometry, sequence,
                                     payload, length);
}

bool queueFaults(uint16_t sequence) {
  ProtocolFaults value;
  value.balance = balanceFaultMask();
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

void respondToSerialFrame(const SerialFrame &frame, uint64_t now_us) {
  switch (frame.type) {
  case SerialMessageType::kHello:
  case SerialMessageType::kCapabilities:
    (void)queueCapabilities(frame.sequence);
    return;
  case SerialMessageType::kStatus:
    (void)queueStatus(frame.sequence);
    return;
  case SerialMessageType::kImuTelemetry:
    (void)queueImuTelemetry(frame.sequence, now_us);
    return;
  case SerialMessageType::kMotorTelemetry:
    (void)queueMotorTelemetry(frame.sequence);
    return;
  case SerialMessageType::kOdometry:
    (void)queueOdometry(frame.sequence, now_us);
    return;
  case SerialMessageType::kActiveFaults:
    (void)queueFaults(frame.sequence);
    return;
  case SerialMessageType::kConfigurationRead: {
    const auto key = static_cast<BalanceConfigKey>(frame.payload[0]);
    uint8_t payload[kSerialMaximumPayloadSize] = {};
    const size_t length = BalanceConfigurationCodec::encodeCurrent(
        key, balance_config, payload, sizeof(payload));
    if (length == 0u) {
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kInvalidConfiguration));
      return;
    }
    (void)telemetry_sink.queuePayload(SerialMessageType::kConfigurationRead,
                                      frame.sequence, payload, length);
    return;
  }
  case SerialMessageType::kConfigurationUpdate: {
    if (state_machine.state() != BalanceState::kDisarmed) {
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kUnsafeState));
      return;
    }
    CascadedBalanceConfig updated = balance_config;
    if (!BalanceConfigurationCodec::applyUpdate(
            frame.payload.data(), frame.payload_length, updated)) {
      (void)telemetry_sink.queueError(
          frame.type, frame.sequence,
          static_cast<uint8_t>(SerialErrorCode::kInvalidConfiguration));
      return;
    }
    balance_config = updated;
    controller.configure(balance_config);
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return;
  }
  default:
    (void)telemetry_sink.queueAcknowledgment(frame.type, frame.sequence);
    return;
  }
}

void serviceNorthboundSerial(uint64_t now_us) {
  size_t serviced = 0u;
  while (Serial.available() > 0 && serviced < kMaximumSerialBytesPerPass) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    ++serviced;
    const SerialParseResult result = serial_source.feed(byte, now_us);
    if (result == SerialParseResult::kFrame) {
      respondToSerialFrame(serial_source.lastFrame(), now_us);
    }
    if (result == SerialParseResult::kMalformed) {
      (void)telemetry_sink.queueError(serial_source.lastFrameType(),
                                      serial_source.lastFrameSequence(),
                                      serial_source.lastErrorCode());
    }
    if (result == SerialParseResult::kInvalidCrc) {
      (void)telemetry_sink.queueError(
          SerialMessageType::kHello, 0u,
          static_cast<uint8_t>(SerialErrorCode::kInvalidCrc));
    }
  }
  ControlRequest request{};
  if (serial_source.latest(request)) {
    command_arbiter.submit(request);
  }
}

#if GS_ENABLE_WEB_CONTROL
void serviceWebControl(uint64_t now_us) {
  const auto &pitch = telemetry_sink.estimate();
  const auto &output = telemetry_sink.output();
  web_adapter.service(
      now_us, stateName(state_machine.state()), pitch.filtered_pitch_deg,
      output.left, output.right, latest_safety.imu_healthy,
      latest_safety.calibrated, latest_safety.motor_feedback_healthy,
      GS_BALANCE_DRY_RUN != 0,
      balanceFaultMask() | feedback.master_faults | feedback.slave_faults);
  ControlRequest request{};
  if (web_source.latest(request)) {
    command_arbiter.submit(request);
  }
}
#endif

SafetySnapshot safetySnapshot(uint64_t now_us, const PitchEstimate &pitch) {
  const auto &diagnostics = imu.diagnostics();
  SafetySnapshot safety;
  safety.imu_healthy =
      imu_started && !imu.timedOut(now_us) &&
      (!diagnostics.calibration_complete || pitch.valid);
  safety.calibrated = diagnostics.calibration_complete;
  safety.approximately_upright =
      std::fabs(balanceFramePitchDeg(pitch.filtered_pitch_deg,
                                     balance_config)) <=
      user_config::kArmingToleranceDeg;
  safety.motor_feedback_healthy = feedbackFresh(now_us) && !feedbackFaulted();
  safety.motor_transport_healthy =
      command_transport_ready && !motor_transport_fault;
  safety.zero_output_acknowledged = zeroReadyAck();
  safety.loop_healthy = !scheduler_fault && loop_overruns < 5u;
  safety.controller_fault = !controller_healthy;
  return safety;
}

float wheelVelocity(uint64_t now_us) {
  static int32_t previous_left = 0;
  static int32_t previous_right = 0;
  static uint64_t previous_us = 0u;
  static uint32_t processed_feedback_frames = 0u;
  if (processed_feedback_frames == feedback_frames) {
    return latest_wheel_velocity;
  }
  processed_feedback_frames = feedback_frames;
  if (previous_us == 0u || last_feedback_us <= previous_us) {
    previous_left = feedback.left_odometer;
    previous_right = feedback.right_odometer;
    previous_us = last_feedback_us;
    latest_wheel_velocity = 0.0f;
    return 0.0f;
  }
  const float dt =
      static_cast<float>(last_feedback_us - previous_us) / 1000000.0f;
  const int32_t left_delta = feedback.left_odometer - previous_left;
  const int32_t right_delta = feedback.right_odometer - previous_right;
  previous_left = feedback.left_odometer;
  previous_right = feedback.right_odometer;
  previous_us = last_feedback_us;
  latest_wheel_velocity =
      static_cast<float>(left_delta + right_delta) * 0.5f / dt;
  (void)now_us;
  return latest_wheel_velocity;
}

void applyRequest(const ControlRequest &request, const SafetySnapshot &safety) {
  if (request.set_operating_mode) {
    const bool changed = operating_mode != request.operating_mode;
    operating_mode = request.operating_mode;
    if (changed || operating_mode == 0u) {
      state_machine.disarm();
      controller.reset(0.0f, 0.0f);
    }
  }
  if (request.emergency_stop || request.disarm) {
    state_machine.disarm();
    controller.reset(0.0f, 0.0f);
    return;
  }
  if (request.clear_fault) {
    state_machine.clearFault(safety);
  }
  if (request.arm && operating_mode != 0u) {
    state_machine.arm(safety);
  }
  const bool driving =
      (operating_mode == 2u &&
       (request.linear_velocity != 0.0f || request.yaw_rate != 0.0f)) ||
      (operating_mode == 3u && request.direct_motor &&
       (request.direct_left != 0.0f || request.direct_right != 0.0f));
  state_machine.setDriving(driving);
}

void runControlLoop(uint64_t scheduled_us) {
  const uint64_t started_us = clock_source.nowMicros();
  const uint32_t period_us =
      previous_control_us == 0u
          ? kControlPeriodUs
          : static_cast<uint32_t>(started_us - previous_control_us);
  previous_control_us = started_us;
  if (period_us > kControlPeriodUs * 2u) {
    ++loop_overruns;
    scheduler_fault = true;
  }

  ImuSample sample{};
  if (imu_started && imu.sample(sample)) {
    latest_sample = sample;
    if (!imu.diagnostics().calibration_complete) {
      estimator.reset(sample);
      estimator_started = false;
    } else if (!estimator_started) {
      estimator.reset(sample);
      estimator_started = true;
    } else {
      estimator.update(sample);
    }
  }
  const PitchEstimate &pitch = estimator.estimate();
  SafetySnapshot safety = safetySnapshot(started_us, pitch);
  latest_safety = safety;
  state_machine.update(
      safety, balanceFramePitchDeg(pitch.filtered_pitch_deg, balance_config),
      started_us);
  command_arbiter.setLocalDisarm(digitalRead(kLocalDisarmPin) == LOW);
  const ControlRequest request = command_arbiter.resolve(started_us);
  if (request.clear_fault) {
    loop_overruns = 0u;
    scheduler_fault = false;
    if (command_transport_ready && feedbackFresh(started_us) &&
        !feedbackFaulted() && exactAck()) {
      motor_transport_fault = false;
    }
    safety = safetySnapshot(started_us, pitch);
    latest_safety = safety;
  }
  applyRequest(request, safety);

  BalanceInput input;
  input.pitch_deg = pitch.filtered_pitch_deg;
  input.pitch_rate_dps = pitch.pitch_rate_dps;
  input.wheel_velocity = wheelVelocity(started_us);
  input.desired_velocity = request.linear_velocity;
  input.desired_yaw_rate = request.yaw_rate;
  input.dt_seconds = pitch.dt_seconds > 0.0f ? pitch.dt_seconds : 0.005f;
  const BalanceOutput output =
      operating_mode == 3u
          ? control_runtime.stepDirect(
                request.direct_motor ? request.direct_left : 0.0f,
                request.direct_motor ? request.direct_right : 0.0f,
                state_machine.outputEnabled())
          : control_runtime.step(input, state_machine.outputEnabled());
  controller_healthy = output.valid;
  telemetry_sink.publish(pitch, output);

  const uint64_t finished_us = clock_source.nowMicros();
  LoopObservation observation;
  observation.actual_start_us = started_us;
  observation.period_us = period_us;
  observation.execution_us = static_cast<uint32_t>(finished_us - started_us);
  observation.sensor_sample_age_us =
      LoopMetrics::ageUs(finished_us, latest_sample.timestamp_us);
  observation.motor_command_age_us =
      LoopMetrics::ageUs(started_us, last_motor_applied_us);
  loop_metrics.record(observation);
  if (finished_us > scheduled_us + kControlPeriodUs) {
    ++loop_overruns;
    scheduler_fault = true;
  }
}

const char *stateName(BalanceState state) {
  switch (state) {
  case BalanceState::kBoot:
    return "BOOT";
  case BalanceState::kImuCalibrating:
    return "IMU_CALIBRATING";
  case BalanceState::kDisarmed:
    return "DISARMED";
  case BalanceState::kArmedBalance:
    return "ARMED_BALANCE";
  case BalanceState::kDriving:
    return "DRIVING";
  case BalanceState::kFallen:
    return "FALLEN";
  case BalanceState::kFault:
    return "FAULT";
  }
  return "UNKNOWN";
}

void serviceTelemetry(uint64_t now_us) {
  static char message[2048] = {};
  static size_t message_length = 0u;
  static size_t message_offset = 0u;
  telemetry_sink.service();
  if (telemetry_sink.busy()) {
    return;
  }
  if (message_offset < message_length) {
    const size_t available = static_cast<size_t>(Serial.availableForWrite());
    const size_t remaining = message_length - message_offset;
    const size_t write_length = std::min(available, remaining);
    if (write_length != 0u) {
      Serial.write(reinterpret_cast<const uint8_t *>(&message[message_offset]),
                   write_length);
      message_offset += write_length;
    }
    return;
  }
  if (now_us < next_telemetry_us) {
    return;
  }
  next_telemetry_us = now_us + kTelemetryPeriodUs;
  const auto &diagnostics = imu.diagnostics();
  const auto stats = loop_metrics.statistics();
  const auto &pitch = telemetry_sink.estimate();
  const auto &output = telemetry_sink.output();
  const float sample_hz = measuredSampleRate();
  const auto transport = transport_metrics.statistics();
  const int written = std::snprintf(
      message, sizeof(message),
      "BALANCE {\"dry_run\":%u,\"mode\":%u,\"state\":\"%s\","
      "\"mpu_address\":%u,"
      "\"calibrated\":%u,\"calibration_samples\":%u,"
      "\"gyro_bias\":[%.4f,%.4f,%.4f],"
      "\"accel\":[%.4f,%.4f,%.4f],"
      "\"gyro_raw\":[%.4f,%.4f,%.4f],"
      "\"gyro_corrected\":[%.4f,%.4f,%.4f],\"pitch_raw\":%.4f,"
      "\"pitch_filtered\":%.4f,\"pitch_rate\":%.4f,"
      "\"calculated\":[%.3f,%.3f],\"pitch_reference\":%.4f,"
      "\"pitch_terms\":[%.4f,%.4f,%.4f],"
      "\"velocity_terms\":[%.4f,%.4f],\"yaw_term\":%.4f,"
      "\"integrals\":[%.4f,%.4f],\"saturated\":%u,"
      "\"controller_valid\":%u,\"i2c_errors\":%lu,"
      "\"missed_samples\":%lu,\"sample_hz\":%.3f,"
      "\"loop_hz\":%.3f,\"period_us\":[%lu,%lu],"
      "\"worst_jitter_us\":%lu,\"missed_deadlines\":%lu,"
      "\"max_execution_us\":%lu,\"sample_age_us\":%lu,"
      "\"motor_age_us\":%lu,\"motor_rate_hz\":%.3f,"
      "\"tx\":%lu,\"rx\":%lu,\"crc_errors\":%lu,"
      "\"ack_timeouts\":%lu,\"ack_latency_us\":[%lu,%lu],"
      "\"apply_latency_us\":[%lu,%lu],\"serial_response_drops\":%lu,"
      "\"faults\":[%lu,%lu],"
      "\"feedback\":{\"protocol\":%u,\"states\":[%u,%u],"
      "\"status_flags\":[%u,%u],\"sequences\":[%u,%u,%u],"
      "\"applied\":[%d,%d],\"ages_ms\":[%u,%u,%u],"
      "\"hall\":[%u,%u],\"compare\":[%u,%u],"
      "\"remote\":[%u,%u,%u,%u]}}\n",
      GS_BALANCE_DRY_RUN != 0 ? 1u : 0u, operating_mode,
      stateName(state_machine.state()), imu.address(),
      diagnostics.calibration_complete ? 1u : 0u,
      static_cast<unsigned>(diagnostics.calibration_samples),
      diagnostics.gyro_bias_dps.x, diagnostics.gyro_bias_dps.y,
      diagnostics.gyro_bias_dps.z, latest_sample.accel_g.x,
      latest_sample.accel_g.y, latest_sample.accel_g.z,
      diagnostics.raw_gyro_dps.x, diagnostics.raw_gyro_dps.y,
      diagnostics.raw_gyro_dps.z, latest_sample.gyro_dps.x,
      latest_sample.gyro_dps.y, latest_sample.gyro_dps.z, pitch.raw_pitch_deg,
      pitch.filtered_pitch_deg, pitch.pitch_rate_dps, output.left, output.right,
      output.pitch_reference, output.pitch_proportional, output.pitch_integral,
      output.pitch_derivative, output.velocity_proportional,
      output.velocity_integral, output.yaw_correction, output.inner_integral,
      output.outer_integral, output.saturated ? 1u : 0u, output.valid ? 1u : 0u,
      static_cast<unsigned long>(diagnostics.i2c_errors),
      static_cast<unsigned long>(diagnostics.missed_samples), sample_hz,
      stats.average_frequency_hz,
      static_cast<unsigned long>(stats.minimum_period_us),
      static_cast<unsigned long>(stats.maximum_period_us),
      static_cast<unsigned long>(stats.worst_jitter_us),
      static_cast<unsigned long>(stats.missed_deadlines),
      static_cast<unsigned long>(stats.maximum_execution_us),
      static_cast<unsigned long>(stats.sensor_sample_age_us),
      static_cast<unsigned long>(stats.motor_command_age_us),
      transport.transmit_rate_hz, static_cast<unsigned long>(command_frames),
      static_cast<unsigned long>(feedback_frames),
      static_cast<unsigned long>(feedback_crc_errors),
      static_cast<unsigned long>(ack_timeouts),
      static_cast<unsigned long>(transport.last_ack_latency_us),
      static_cast<unsigned long>(transport.maximum_ack_latency_us),
      static_cast<unsigned long>(transport.last_apply_latency_us),
      static_cast<unsigned long>(transport.maximum_apply_latency_us),
      static_cast<unsigned long>(telemetry_sink.droppedFrames()),
      static_cast<unsigned long>(feedback.master_faults),
      static_cast<unsigned long>(feedback.slave_faults),
      static_cast<unsigned>(feedback.protocol_version),
      static_cast<unsigned>(feedback.master_state),
      static_cast<unsigned>(feedback.slave_state),
      static_cast<unsigned>(feedback.status_flags),
      static_cast<unsigned>(feedback.motor_status_flags),
      static_cast<unsigned>(feedback.accepted_esp_sequence),
      static_cast<unsigned>(feedback.forwarded_slave_sequence),
      static_cast<unsigned>(feedback.accepted_slave_sequence),
      static_cast<int>(feedback.left_applied),
      static_cast<int>(feedback.right_applied),
      static_cast<unsigned>(feedback.master_command_age_ms),
      static_cast<unsigned>(feedback.slave_feedback_age_ms),
      static_cast<unsigned>(feedback.slave_command_age_ms),
      static_cast<unsigned>(feedback.left_hall),
      static_cast<unsigned>(feedback.right_hall),
      static_cast<unsigned>(feedback.left_compare_offset),
      static_cast<unsigned>(feedback.right_compare_offset),
      static_cast<unsigned>(feedback.remote_rx_bytes),
      static_cast<unsigned>(feedback.remote_valid_frames),
      static_cast<unsigned>(feedback.remote_invalid_frames),
      static_cast<unsigned>(feedback.remote_framing_errors));
  message_length = written <= 0 ? 0u
                                : std::min(static_cast<size_t>(written),
                                           sizeof(message) - 1u);
  message_offset = 0u;
}

} // namespace

void setup() {
  Serial.begin(kConsoleBaud);
  (void)esp_task_wdt_init(kTaskWatchdogTimeoutSeconds, true);
  (void)esp_task_wdt_add(nullptr);
  pinMode(kLocalDisarmPin, INPUT_PULLUP);
  controller_uart.begin(kControllerBaud, SERIAL_8N1, kControllerRx, -1);
  const uint8_t marker[] = {GS_FEEDBACK_MARKER_0, GS_FEEDBACK_MARKER_1};
  gs_frame_parser_init(&feedback_parser, marker, 2u, GS_MASTER_FEEDBACK_SIZE);
  gs_command_sequencer_init(&command_sequencer);
  command_transport_ready = initializeCommandTransport();
  imu_started = imu.begin();
#if GS_ENABLE_WEB_CONTROL
  web_adapter.begin(GS_WIFI_SSID, GS_WIFI_PASSWORD);
#endif
  const uint64_t now_us = clock_source.nowMicros();
  next_control_us = now_us + kControlPeriodUs;
  next_motor_heartbeat_us = now_us;
  next_telemetry_us = now_us;
  last_feedback_us = now_us;
  Serial.printf("GAUSSTOP balance coordinator v1: dry_run=%u, control_hz=200, "
                "motor_hz=10, MPU SDA=%d SCL=%d, motors disabled\n",
                GS_BALANCE_DRY_RUN != 0 ? 1u : 0u, kMpuSda, kMpuScl);
}

void loop() {
  uint64_t now_us = clock_source.nowMicros();
  if (static_cast<int64_t>(now_us - next_control_us) >= 0) {
    const uint64_t scheduled_us = next_control_us;
    next_control_us += kControlPeriodUs;
    runControlLoop(scheduled_us);
    now_us = clock_source.nowMicros();
    if (static_cast<int64_t>(now_us - next_control_us) >= 0) {
      next_control_us = now_us + kControlPeriodUs;
      ++loop_overruns;
    }
  }
  serviceFeedback(now_us);
  serviceNorthboundSerial(now_us);
  serviceMotorHeartbeat(now_us);
  serviceTelemetry(now_us);
#if GS_ENABLE_WEB_CONTROL
  serviceWebControl(now_us);
#endif
  (void)esp_task_wdt_reset();
}
