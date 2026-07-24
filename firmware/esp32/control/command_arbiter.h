/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::balance {

enum class CommandSource : uint8_t {
  kNone,
  kWeb,
  kSerial,
  kLocal,
};

struct ControlRequest {
  CommandSource source = CommandSource::kNone;
  float linear_velocity = 0.0f;
  float yaw_rate = 0.0f;
  float direct_left = 0.0f;
  float direct_right = 0.0f;
  uint64_t lease_expires_us = 0u;
  uint32_t lease_id = 0u;
  uint16_t sequence = 0u;
  bool arm = false;
  bool disarm = false;
  bool emergency_stop = false;
  bool clear_fault = false;
  bool direct_motor = false;
  bool set_operating_mode = false;
  uint8_t operating_mode = 0u;
};

class CommandArbiter {
public:
  void submit(const ControlRequest &request);
  void disconnect(CommandSource source);
  void setLocalDisarm(bool active);
  void setEmergencyStop(bool active);
  ControlRequest resolve(uint64_t now_us);
  CommandSource activeSource() const;

private:
  static bool active(const ControlRequest &request, uint64_t now_us);
  static ControlRequest stopped();
  void clearOwnership();

  ControlRequest serial_{};
  ControlRequest web_{};
  CommandSource active_source_ = CommandSource::kNone;
  bool serial_seen_ = false;
  bool web_seen_ = false;
  bool serial_pending_ = false;
  bool web_pending_ = false;
  bool local_disarm_ = false;
  bool emergency_stop_ = false;
};

} // namespace gs::balance
