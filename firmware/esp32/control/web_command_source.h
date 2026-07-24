/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "command_arbiter.h"
#include "interfaces.h"

#include <cstdint>

namespace gs::balance {

class WebCommandSource final : public ICommandSource {
public:
  void movement(float linear_velocity, float yaw_rate, uint64_t now_us,
                uint32_t lifetime_us = 250000u);
  void arm(uint64_t now_us);
  void disarm(uint64_t now_us);
  void stop(uint64_t now_us);
  void emergencyStop(uint64_t now_us);
  void disconnect();
  bool latest(ControlRequest &request) const override;

private:
  void beginRequest(uint64_t now_us, uint32_t lifetime_us);

  ControlRequest latest_{};
  bool has_request_ = false;
  uint16_t sequence_ = 0u;
};

} // namespace gs::balance
