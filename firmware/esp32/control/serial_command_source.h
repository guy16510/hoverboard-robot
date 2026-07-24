/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "command_arbiter.h"
#include "interfaces.h"
#include "serial_protocol.h"

#include <cstdint>

namespace gs::balance {

class SerialCommandSource final : public ICommandSource {
public:
  explicit SerialCommandSource(uint32_t parser_timeout_us);

  SerialParseResult feed(uint8_t byte, uint64_t now_us);
  bool latest(ControlRequest &request) const override;
  void disconnect();
  uint32_t acceptedFrames() const;
  uint32_t rejectedFrames() const;
  uint32_t crcErrors() const;
  uint32_t parserTimeouts() const;
  uint16_t lastFrameSequence() const;
  SerialMessageType lastFrameType() const;
  const SerialFrame &lastFrame() const;
  uint8_t lastErrorCode() const;

private:
  bool acceptFrame(const SerialFrame &frame, uint64_t now_us);
  bool acceptMovement(const SerialFrame &frame, uint64_t now_us);
  bool acceptAction(const SerialFrame &frame, uint64_t now_us);

  SerialParser parser_;
  SerialSequence sequence_;
  ControlRequest latest_{};
  bool has_request_ = false;
  uint32_t accepted_frames_ = 0u;
  uint32_t rejected_frames_ = 0u;
  uint32_t crc_errors_ = 0u;
  uint32_t parser_timeouts_ = 0u;
  uint16_t last_frame_sequence_ = 0u;
  SerialMessageType last_frame_type_ = SerialMessageType::kHello;
  SerialFrame last_frame_{};
  uint8_t last_error_code_ = 0u;
  uint8_t operating_mode_ = 0u;
  bool operating_mode_set_ = false;
  uint32_t active_lease_id_ = 0u;
  uint64_t active_lease_expires_us_ = 0u;
};

} // namespace gs::balance
