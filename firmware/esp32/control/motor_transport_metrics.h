/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::balance {

struct MotorTransportStatistics {
  uint32_t transmitted_frames = 0u;
  uint32_t started_commands = 0u;
  uint32_t acknowledged_commands = 0u;
  uint32_t applied_commands = 0u;
  uint32_t timeouts = 0u;
  float transmit_rate_hz = 0.0f;
  uint32_t last_ack_latency_us = 0u;
  uint32_t maximum_ack_latency_us = 0u;
  uint32_t last_apply_latency_us = 0u;
  uint32_t maximum_apply_latency_us = 0u;
};

class MotorTransportMetrics {
public:
  void recordTransmission(uint64_t now_us);
  void beginCommand(uint16_t sequence, uint64_t now_us);
  void observe(uint16_t accepted_esp_sequence,
               uint16_t forwarded_slave_sequence,
               uint16_t accepted_slave_sequence, uint64_t now_us);
  void recordTimeout();
  MotorTransportStatistics statistics() const;

private:
  MotorTransportStatistics statistics_{};
  uint64_t first_transmission_us_ = 0u;
  uint64_t last_transmission_us_ = 0u;
  uint64_t command_started_us_ = 0u;
  uint16_t active_sequence_ = 0u;
  bool command_active_ = false;
  bool acknowledgment_recorded_ = false;
  bool application_recorded_ = false;
};

} // namespace gs::balance
