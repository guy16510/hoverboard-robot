/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::balance {

struct LoopObservation {
  uint64_t actual_start_us = 0u;
  uint32_t period_us = 0u;
  uint32_t execution_us = 0u;
  uint32_t sensor_sample_age_us = 0u;
  uint32_t motor_command_age_us = 0u;
};

struct LoopStatistics {
  float requested_frequency_hz = 0.0f;
  float average_frequency_hz = 0.0f;
  uint32_t minimum_period_us = 0u;
  uint32_t maximum_period_us = 0u;
  uint32_t worst_jitter_us = 0u;
  uint32_t missed_deadlines = 0u;
  uint32_t maximum_execution_us = 0u;
  uint32_t sensor_sample_age_us = 0u;
  uint32_t motor_command_age_us = 0u;
};

class LoopMetrics {
public:
  explicit LoopMetrics(uint32_t requested_period_us);

  void record(const LoopObservation &observation);
  LoopStatistics statistics() const;
  void reset();
  static uint32_t ageUs(uint64_t now_us, uint64_t timestamp_us);

private:
  uint32_t requested_period_us_;
  LoopStatistics statistics_{};
  uint64_t first_start_us_ = 0u;
  uint64_t last_start_us_ = 0u;
  uint32_t observations_ = 0u;
};

} // namespace gs::balance
