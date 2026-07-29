/* SPDX-License-Identifier: GPL-3.0-only */
#include "serial_command_source.h"

namespace gs::balance {
namespace {
constexpr uint64_t kActionLifetimeUs = 500000u;
}

SerialCommandSource::SerialCommandSource(uint32_t parser_timeout_us)
    : parser_(parser_timeout_us) {}

SerialParseResult SerialCommandSource::feed(uint8_t byte, uint64_t now_us) {
  SerialFrame frame{};
  const SerialParseResult result = parser_.feed(byte, now_us, frame);
  if (result == SerialParseResult::kInvalidCrc) {
    ++crc_errors_;
    ++rejected_frames_;
    last_frame_sequence_ = 0u;
    last_frame_type_ = SerialMessageType::kHello;
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kInvalidCrc);
  }
  if (result == SerialParseResult::kTimeout) {
    ++parser_timeouts_;
  }
  if (result == SerialParseResult::kMalformed) {
    ++rejected_frames_;
    last_frame_sequence_ = 0u;
    last_frame_type_ = SerialMessageType::kHello;
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
  }
  if (result != SerialParseResult::kFrame) {
    return result;
  }
  last_frame_sequence_ = frame.sequence;
  last_frame_type_ = frame.type;
  last_frame_ = frame;
  last_error_code_ = 0u;
  if (frame.type == SerialMessageType::kHello && frame.payload_length == 0u) {
    sequence_.reset();
  }
  if (!sequence_.accept(frame.sequence)) {
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kStaleSequence);
    ++rejected_frames_;
    return SerialParseResult::kMalformed;
  }
  if (!acceptFrame(frame, now_us)) {
    if (last_error_code_ == 0u) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kUnsupported);
    }
    ++rejected_frames_;
    return SerialParseResult::kMalformed;
  }
  ++accepted_frames_;
  return result;
}

bool SerialCommandSource::latest(ControlRequest &request) const {
  if (!has_request_) {
    return false;
  }
  request = latest_;
  return true;
}

void SerialCommandSource::disconnect() {
  latest_ = {};
  latest_.source = CommandSource::kSerial;
  latest_.sequence = static_cast<uint16_t>(last_frame_sequence_ + 1u);
  has_request_ = true;
  active_lease_id_ = 0u;
  active_lease_expires_us_ = 0u;
}

uint32_t SerialCommandSource::acceptedFrames() const {
  return accepted_frames_;
}

uint32_t SerialCommandSource::rejectedFrames() const {
  return rejected_frames_;
}

uint32_t SerialCommandSource::crcErrors() const { return crc_errors_; }

uint32_t SerialCommandSource::parserTimeouts() const {
  return parser_timeouts_;
}

uint16_t SerialCommandSource::lastFrameSequence() const {
  return last_frame_sequence_;
}

SerialMessageType SerialCommandSource::lastFrameType() const {
  return last_frame_type_;
}

const SerialFrame &SerialCommandSource::lastFrame() const {
  return last_frame_;
}

uint8_t SerialCommandSource::lastErrorCode() const { return last_error_code_; }

bool SerialCommandSource::acceptFrame(const SerialFrame &frame,
                                      uint64_t now_us) {
  if (frame.type == SerialMessageType::kHello && frame.payload_length == 0u) {
    latest_ = {};
    latest_.source = CommandSource::kSerial;
    latest_.lease_expires_us = now_us + kActionLifetimeUs;
    latest_.sequence = frame.sequence;
    latest_.set_operating_mode = operating_mode_set_;
    latest_.operating_mode = operating_mode_;
    has_request_ = true;
    active_lease_id_ = 0u;
    active_lease_expires_us_ = 0u;
    return true;
  }
  if (frame.type == SerialMessageType::kSetVelocityAndYaw ||
      frame.type == SerialMessageType::kSetLinearVelocity ||
      frame.type == SerialMessageType::kSetYawRate ||
      (frame.type == SerialMessageType::kHeartbeat &&
       frame.payload_length != 0u)) {
    return acceptMovement(frame, now_us);
  }
  if (frame.type == SerialMessageType::kSetDirectMotor) {
    DirectMotorCommand direct{};
    if (!DirectMotorCommandCodec::decode(frame.payload.data(),
                                         frame.payload_length, direct)) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
      return false;
    }
    if (direct.left < -kMaximumTransportTestCommand ||
        direct.left > kMaximumTransportTestCommand ||
        direct.right < -kMaximumTransportTestCommand ||
        direct.right > kMaximumTransportTestCommand) {
      last_error_code_ =
          static_cast<uint8_t>(SerialErrorCode::kInvalidConfiguration);
      return false;
    }
    if (active_lease_id_ != 0u && now_us <= active_lease_expires_us_ &&
        direct.lease_id != active_lease_id_) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kLeaseConflict);
      return false;
    }
    latest_ = {};
    latest_.source = CommandSource::kSerial;
    latest_.direct_motor = true;
    latest_.direct_left = static_cast<float>(direct.left);
    latest_.direct_right = static_cast<float>(direct.right);
    latest_.lease_expires_us =
        now_us + static_cast<uint64_t>(direct.lifetime_ms) * 1000u;
    latest_.lease_id = direct.lease_id;
    latest_.sequence = frame.sequence;
    latest_.set_operating_mode = operating_mode_set_;
    latest_.operating_mode = operating_mode_;
    has_request_ = true;
    active_lease_id_ = direct.lease_id;
    active_lease_expires_us_ = latest_.lease_expires_us;
    return true;
  }
  switch (frame.type) {
  case SerialMessageType::kCapabilities:
  case SerialMessageType::kStatus:
  case SerialMessageType::kImuTelemetry:
  case SerialMessageType::kMotorTelemetry:
  case SerialMessageType::kOdometry:
  case SerialMessageType::kActiveFaults:
  case SerialMessageType::kDriveTelemetry:
  case SerialMessageType::kControllerTelemetry:
  case SerialMessageType::kResilienceTelemetry:
    if (frame.payload_length == 0u) {
      return true;
    }
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
    return false;
  case SerialMessageType::kConfigurationRead:
    if (frame.payload_length == 1u) {
      return true;
    }
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
    return false;
  case SerialMessageType::kConfigurationUpdate:
    if (frame.payload_length == 5u) {
      return true;
    }
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
    return false;
  default:
    break;
  }
  return acceptAction(frame, now_us);
}

bool SerialCommandSource::acceptMovement(const SerialFrame &frame,
                                         uint64_t now_us) {
  MovementCommand movement{};
  if (!MovementCommandCodec::decode(frame.payload.data(), frame.payload_length,
                                    movement)) {
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
    return false;
  }
  if (active_lease_id_ != 0u && now_us <= active_lease_expires_us_ &&
      movement.lease_id != active_lease_id_) {
    last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kLeaseConflict);
    return false;
  }
  const float previous_linear = latest_.linear_velocity;
  const float previous_yaw = latest_.yaw_rate;
  latest_ = {};
  latest_.source = CommandSource::kSerial;
  latest_.linear_velocity =
      frame.type == SerialMessageType::kSetYawRate
          ? previous_linear
          : static_cast<float>(movement.linear_velocity_milli) / 1000.0f;
  latest_.yaw_rate =
      frame.type == SerialMessageType::kSetLinearVelocity
          ? previous_yaw
          : static_cast<float>(movement.yaw_rate_milli) / 1000.0f;
  latest_.lease_expires_us =
      now_us + static_cast<uint64_t>(movement.lifetime_ms) * 1000u;
  latest_.lease_id = movement.lease_id;
  latest_.sequence = frame.sequence;
  latest_.set_operating_mode = operating_mode_set_;
  latest_.operating_mode = operating_mode_;
  has_request_ = true;
  active_lease_id_ = movement.lease_id;
  active_lease_expires_us_ = latest_.lease_expires_us;
  return true;
}

bool SerialCommandSource::acceptAction(const SerialFrame &frame,
                                       uint64_t now_us) {
  latest_ = {};
  latest_.source = CommandSource::kSerial;
  latest_.lease_expires_us = now_us + kActionLifetimeUs;
  latest_.sequence = frame.sequence;
  latest_.set_operating_mode = operating_mode_set_;
  latest_.operating_mode = operating_mode_;
  switch (frame.type) {
  case SerialMessageType::kArm:
    if (frame.payload_length != 0u) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
      return false;
    }
    latest_.arm = true;
    break;
  case SerialMessageType::kDisarm:
    if (frame.payload_length != 0u) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
      return false;
    }
    latest_.disarm = true;
    active_lease_id_ = 0u;
    active_lease_expires_us_ = 0u;
    break;
  case SerialMessageType::kStop:
  case SerialMessageType::kHeartbeat:
    if (frame.payload_length != 0u) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
      return false;
    }
    active_lease_id_ = 0u;
    active_lease_expires_us_ = 0u;
    break;
  case SerialMessageType::kEmergencyStop:
    if (frame.payload_length != 0u) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
      return false;
    }
    latest_.emergency_stop = true;
    latest_.disarm = true;
    active_lease_id_ = 0u;
    active_lease_expires_us_ = 0u;
    break;
  case SerialMessageType::kClearFault:
    if (frame.payload_length != 0u) {
      last_error_code_ = static_cast<uint8_t>(SerialErrorCode::kMalformed);
      return false;
    }
    latest_.clear_fault = true;
    break;
  case SerialMessageType::kSetOperatingMode:
    if (frame.payload_length != 1u || frame.payload[0] > 3u) {
      last_error_code_ =
          static_cast<uint8_t>(SerialErrorCode::kInvalidConfiguration);
      return false;
    }
    operating_mode_ = frame.payload[0];
    operating_mode_set_ = true;
    latest_.set_operating_mode = true;
    latest_.operating_mode = operating_mode_;
    break;
  default:
    return false;
  }
  has_request_ = true;
  return true;
}

} // namespace gs::balance
