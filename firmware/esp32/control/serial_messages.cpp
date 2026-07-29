/* SPDX-License-Identifier: GPL-3.0-only */
#include "serial_messages.h"

namespace gs::balance {
namespace {

void writeU16(uint8_t *output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8u);
}

void writeI16(uint8_t *output, int16_t value) {
  writeU16(output, static_cast<uint16_t>(value));
}

void writeU32(uint8_t *output, uint32_t value) {
  writeU16(output, static_cast<uint16_t>(value));
  writeU16(output + 2u, static_cast<uint16_t>(value >> 16u));
}

void writeI32(uint8_t *output, int32_t value) {
  writeU32(output, static_cast<uint32_t>(value));
}

void writeU64(uint8_t *output, uint64_t value) {
  writeU32(output, static_cast<uint32_t>(value));
  writeU32(output + 4u, static_cast<uint32_t>(value >> 32u));
}

} // namespace

size_t SerialMessageCodec::encodeCapabilities(const ProtocolCapabilities &value,
                                              uint8_t *payload,
                                              size_t capacity) {
  constexpr size_t kLength = 12u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  payload[0] = kSerialProtocolVersion;
  payload[1] = value.dry_run ? 1u : 0u;
  payload[2] = value.web_enabled ? 1u : 0u;
  payload[3] = value.supported_modes;
  writeU16(&payload[4], value.control_rate_hz);
  writeU16(&payload[6], value.motor_rate_hz);
  writeU16(&payload[8], static_cast<uint16_t>(kSerialMaximumPayloadSize));
  writeU16(&payload[10], value.configuration_keys);
  return kLength;
}

size_t SerialMessageCodec::encodeStatus(const ProtocolStatus &value,
                                        uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 16u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  payload[0] = value.state;
  payload[1] = value.operating_mode;
  payload[2] = value.active_source;
  payload[3] = value.health_flags;
  writeU32(&payload[4], value.faults);
  writeU32(&payload[8], value.loop_overruns);
  writeU32(&payload[12], value.rejected_serial_frames);
  return kLength;
}

size_t SerialMessageCodec::encodeImu(const ProtocolImuTelemetry &value,
                                     uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 44u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  payload[0] = value.address;
  payload[1] = value.calibrated ? 1u : 0u;
  payload[2] = value.valid ? 1u : 0u;
  payload[3] = 0u;
  for (size_t axis = 0u; axis < 3u; ++axis) {
    writeI16(&payload[4u + axis * 2u], value.acceleration_milli_g[axis]);
    writeI16(&payload[10u + axis * 2u], value.gyroscope_centi_dps[axis]);
  }
  writeI16(&payload[16], value.raw_pitch_centi_deg);
  writeI16(&payload[18], value.filtered_pitch_centi_deg);
  writeI16(&payload[20], value.pitch_rate_centi_dps);
  writeU16(&payload[22], value.sample_rate_centi_hz);
  writeU32(&payload[24], value.i2c_errors);
  writeU32(&payload[28], value.missed_samples);
  writeU32(&payload[32], value.sample_age_us);
  writeU16(&payload[36], value.calibration_samples);
  for (size_t axis = 0u; axis < 3u; ++axis) {
    writeI16(&payload[38u + axis * 2u], value.gyro_bias_centi_dps[axis]);
  }
  return kLength;
}

size_t SerialMessageCodec::encodeMotor(const ProtocolMotorTelemetry &value,
                                       uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 46u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeI16(&payload[0], value.calculated_left);
  writeI16(&payload[2], value.calculated_right);
  writeI16(&payload[4], value.applied_left);
  writeI16(&payload[6], value.applied_right);
  writeU16(&payload[8], value.sequence);
  writeU16(&payload[10], value.flags);
  writeU32(&payload[12], value.transmitted_frames);
  writeU32(&payload[16], value.feedback_frames);
  writeU32(&payload[20], value.crc_errors);
  writeU32(&payload[24], value.acknowledgment_timeouts);
  writeU32(&payload[28], value.last_ack_latency_us);
  writeU32(&payload[32], value.maximum_ack_latency_us);
  writeU32(&payload[36], value.last_apply_latency_us);
  writeU32(&payload[40], value.maximum_apply_latency_us);
  writeU16(&payload[44], value.transmit_rate_centi_hz);
  return kLength;
}

size_t SerialMessageCodec::encodeDrive(const ProtocolDriveTelemetry &value,
                                       uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 24u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeI16(&payload[0], value.requested_linear_milli);
  writeI16(&payload[2], value.requested_yaw_milli);
  writeI16(&payload[4], value.mixed_left);
  writeI16(&payload[6], value.mixed_right);
  writeI16(&payload[8], value.commanded_left);
  writeI16(&payload[10], value.commanded_right);
  writeI16(&payload[12], value.applied_left);
  writeI16(&payload[14], value.applied_right);
  writeU32(&payload[16], value.safety_faults);
  payload[20] = value.active_source;
  payload[21] = value.operating_mode;
  payload[22] = value.arm_state;
  payload[23] = value.flags;
  return kLength;
}

size_t SerialMessageCodec::encodeOdometry(const ProtocolOdometry &value,
                                          uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 20u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeI32(&payload[0], value.left);
  writeI32(&payload[4], value.right);
  writeI32(&payload[8], value.velocity_milli);
  writeU64(&payload[12], value.timestamp_us);
  return kLength;
}

size_t SerialMessageCodec::encodeFaults(const ProtocolFaults &value,
                                        uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 16u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeU32(&payload[0], value.balance);
  writeU32(&payload[4], value.master);
  writeU32(&payload[8], value.slave);
  writeU32(&payload[12], value.feedback_health);
  return kLength;
}

size_t
SerialMessageCodec::encodeController(const ProtocolControllerTelemetry &value,
                                     uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 24u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  payload[0] = value.master_state;
  payload[1] = value.slave_state;
  payload[2] = value.status_flags;
  payload[3] = value.motor_status_flags;
  writeU16(&payload[4], value.master_command_age_ms);
  writeU16(&payload[6], value.slave_feedback_age_ms);
  writeU16(&payload[8], value.slave_command_age_ms);
  payload[10] = value.left_hall;
  payload[11] = value.right_hall;
  writeU16(&payload[12], value.left_compare_offset);
  writeU16(&payload[14], value.right_compare_offset);
  writeU16(&payload[16], value.remote_rx_bytes);
  writeU16(&payload[18], value.remote_valid_frames);
  writeU16(&payload[20], value.remote_invalid_frames);
  writeU16(&payload[22], value.remote_framing_errors);
  return kLength;
}

size_t
SerialMessageCodec::encodeResilience(const ProtocolResilienceTelemetry &value,
                                     uint8_t *payload, size_t capacity) {
  constexpr size_t kLength = 48u;
  if (payload == nullptr || capacity < kLength) {
    return 0u;
  }
  writeU16(&payload[0], value.warning_flags);
  payload[2] = value.feedback_crc_streak;
  payload[3] = value.feedback_crc_threshold;
  writeU32(&payload[4], value.feedback_crc_total);
  writeU16(&payload[8], value.left_hall_glitches);
  writeU16(&payload[10], value.right_hall_glitches);
  writeU16(&payload[12], value.slave_feedback_invalid_frames);
  writeU16(&payload[14], value.slave_feedback_framing_errors);
  writeU16(&payload[16], value.slave_command_invalid_frames);
  writeU16(&payload[18], value.slave_command_framing_errors);
  writeU32(&payload[20], value.first_fault.drive_faults);
  writeU32(&payload[24], value.first_fault.master_faults);
  writeU32(&payload[28], value.first_fault.slave_faults);
  payload[32] = value.first_fault.master_state;
  payload[33] = value.first_fault.slave_state;
  payload[34] = value.first_fault.left_hall;
  payload[35] = value.first_fault.right_hall;
  writeI16(&payload[36], value.first_fault.commanded_left);
  writeI16(&payload[38], value.first_fault.commanded_right);
  writeI16(&payload[40], value.first_fault.applied_left);
  writeI16(&payload[42], value.first_fault.applied_right);
  writeU32(&payload[44], value.first_fault.esp32_uptime_ms);
  return kLength;
}

bool SerialFrameQueue::push(const SerialFrame &frame) {
  if (size_ == kCapacity) {
    ++dropped_frames_;
    return false;
  }
  const size_t tail = (head_ + size_) % kCapacity;
  frames_[tail] = frame;
  ++size_;
  return true;
}

bool SerialFrameQueue::pop(SerialFrame &frame) {
  if (size_ == 0u) {
    return false;
  }
  frame = frames_[head_];
  head_ = (head_ + 1u) % kCapacity;
  --size_;
  return true;
}

size_t SerialFrameQueue::size() const { return size_; }

uint32_t SerialFrameQueue::droppedFrames() const { return dropped_frames_; }

} // namespace gs::balance
