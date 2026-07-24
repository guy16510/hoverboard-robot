/* SPDX-License-Identifier: GPL-3.0-only */
#include "motor_transport_metrics.h"

#include <algorithm>

namespace gs::balance {

void MotorTransportMetrics::recordTransmission(uint64_t now_us) {
  if (statistics_.transmitted_frames == 0u) {
    first_transmission_us_ = now_us;
  }
  last_transmission_us_ = now_us;
  ++statistics_.transmitted_frames;
}

void MotorTransportMetrics::beginCommand(uint16_t sequence, uint64_t now_us) {
  if (command_active_ && sequence == active_sequence_) {
    return;
  }
  active_sequence_ = sequence;
  command_started_us_ = now_us;
  command_active_ = true;
  acknowledgment_recorded_ = false;
  application_recorded_ = false;
  ++statistics_.started_commands;
}

void MotorTransportMetrics::observe(uint16_t accepted_esp_sequence,
                                    uint16_t forwarded_slave_sequence,
                                    uint16_t accepted_slave_sequence,
                                    uint64_t now_us) {
  if (!command_active_ || now_us < command_started_us_) {
    return;
  }
  const uint64_t elapsed = now_us - command_started_us_;
  const uint32_t latency =
      elapsed > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed);
  if (!acknowledgment_recorded_ && accepted_esp_sequence == active_sequence_) {
    acknowledgment_recorded_ = true;
    ++statistics_.acknowledged_commands;
    statistics_.last_ack_latency_us = latency;
    statistics_.maximum_ack_latency_us =
        std::max(statistics_.maximum_ack_latency_us, latency);
  }
  if (!application_recorded_ && accepted_esp_sequence == active_sequence_ &&
      forwarded_slave_sequence == active_sequence_ &&
      accepted_slave_sequence == active_sequence_) {
    application_recorded_ = true;
    ++statistics_.applied_commands;
    statistics_.last_apply_latency_us = latency;
    statistics_.maximum_apply_latency_us =
        std::max(statistics_.maximum_apply_latency_us, latency);
  }
}

void MotorTransportMetrics::recordTimeout() { ++statistics_.timeouts; }

MotorTransportStatistics MotorTransportMetrics::statistics() const {
  MotorTransportStatistics result = statistics_;
  if (result.transmitted_frames > 1u &&
      last_transmission_us_ > first_transmission_us_) {
    result.transmit_rate_hz =
        static_cast<float>(result.transmitted_frames - 1u) * 1000000.0f /
        static_cast<float>(last_transmission_us_ - first_transmission_us_);
  }
  return result;
}

} // namespace gs::balance
