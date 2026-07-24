/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "serial_protocol.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gs::balance {

struct ProtocolCapabilities {
  bool dry_run = true;
  bool web_enabled = false;
  uint8_t supported_modes = 0x07u;
  uint16_t control_rate_hz = 200u;
  uint16_t motor_rate_hz = 10u;
  uint16_t configuration_keys = 0u;
};

struct ProtocolStatus {
  uint8_t state = 0u;
  uint8_t operating_mode = 0u;
  uint8_t active_source = 0u;
  uint8_t health_flags = 0u;
  uint32_t faults = 0u;
  uint32_t loop_overruns = 0u;
  uint32_t rejected_serial_frames = 0u;
};

struct ProtocolImuTelemetry {
  uint8_t address = 0u;
  bool calibrated = false;
  bool valid = false;
  int16_t acceleration_milli_g[3] = {};
  int16_t gyroscope_centi_dps[3] = {};
  int16_t raw_pitch_centi_deg = 0;
  int16_t filtered_pitch_centi_deg = 0;
  int16_t pitch_rate_centi_dps = 0;
  uint16_t sample_rate_centi_hz = 0u;
  uint32_t i2c_errors = 0u;
  uint32_t missed_samples = 0u;
  uint32_t sample_age_us = 0u;
  uint16_t calibration_samples = 0u;
  int16_t gyro_bias_centi_dps[3] = {};
};

struct ProtocolMotorTelemetry {
  int16_t calculated_left = 0;
  int16_t calculated_right = 0;
  int16_t applied_left = 0;
  int16_t applied_right = 0;
  uint16_t sequence = 0u;
  uint16_t flags = 0u;
  uint32_t transmitted_frames = 0u;
  uint32_t feedback_frames = 0u;
  uint32_t crc_errors = 0u;
  uint32_t acknowledgment_timeouts = 0u;
  uint32_t last_ack_latency_us = 0u;
  uint32_t maximum_ack_latency_us = 0u;
  uint32_t last_apply_latency_us = 0u;
  uint32_t maximum_apply_latency_us = 0u;
  uint16_t transmit_rate_centi_hz = 0u;
};

struct ProtocolOdometry {
  int32_t left = 0;
  int32_t right = 0;
  int32_t velocity_milli = 0;
  uint64_t timestamp_us = 0u;
};

struct ProtocolFaults {
  uint32_t balance = 0u;
  uint32_t master = 0u;
  uint32_t slave = 0u;
  uint32_t feedback_health = 0u;
};

class SerialMessageCodec {
public:
  static size_t encodeCapabilities(const ProtocolCapabilities &value,
                                   uint8_t *payload, size_t capacity);
  static size_t encodeStatus(const ProtocolStatus &value, uint8_t *payload,
                             size_t capacity);
  static size_t encodeImu(const ProtocolImuTelemetry &value, uint8_t *payload,
                          size_t capacity);
  static size_t encodeMotor(const ProtocolMotorTelemetry &value,
                            uint8_t *payload, size_t capacity);
  static size_t encodeOdometry(const ProtocolOdometry &value, uint8_t *payload,
                               size_t capacity);
  static size_t encodeFaults(const ProtocolFaults &value, uint8_t *payload,
                             size_t capacity);
};

class SerialFrameQueue {
public:
  static constexpr size_t kCapacity = 8u;

  bool push(const SerialFrame &frame);
  bool pop(SerialFrame &frame);
  size_t size() const;
  uint32_t droppedFrames() const;

private:
  std::array<SerialFrame, kCapacity> frames_{};
  size_t head_ = 0u;
  size_t size_ = 0u;
  uint32_t dropped_frames_ = 0u;
};

} // namespace gs::balance
