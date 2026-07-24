/* SPDX-License-Identifier: GPL-3.0-only */
#include "web_command_source.h"

namespace gs::balance {
namespace {
constexpr uint32_t kActionLifetimeUs = 500000u;
}

void WebCommandSource::movement(float linear_velocity, float yaw_rate,
                                uint64_t now_us, uint32_t lifetime_us) {
  beginRequest(now_us, lifetime_us);
  latest_.set_operating_mode = true;
  latest_.operating_mode = 2u;
  latest_.linear_velocity = linear_velocity;
  latest_.yaw_rate = yaw_rate;
}

void WebCommandSource::arm(uint64_t now_us) {
  beginRequest(now_us, kActionLifetimeUs);
  latest_.set_operating_mode = true;
  latest_.operating_mode = 1u;
  latest_.arm = true;
}

void WebCommandSource::disarm(uint64_t now_us) {
  beginRequest(now_us, kActionLifetimeUs);
  latest_.disarm = true;
}

void WebCommandSource::stop(uint64_t now_us) {
  beginRequest(now_us, kActionLifetimeUs);
}

void WebCommandSource::emergencyStop(uint64_t now_us) {
  beginRequest(now_us, kActionLifetimeUs);
  latest_.emergency_stop = true;
  latest_.disarm = true;
}

void WebCommandSource::disconnect() {
  latest_ = {};
  latest_.source = CommandSource::kWeb;
  latest_.sequence = sequence_++;
  has_request_ = true;
}

bool WebCommandSource::latest(ControlRequest &request) const {
  if (!has_request_) {
    return false;
  }
  request = latest_;
  return true;
}

void WebCommandSource::beginRequest(uint64_t now_us, uint32_t lifetime_us) {
  latest_ = {};
  latest_.source = CommandSource::kWeb;
  latest_.lease_expires_us = now_us + lifetime_us;
  latest_.sequence = sequence_++;
  has_request_ = true;
}

} // namespace gs::balance
