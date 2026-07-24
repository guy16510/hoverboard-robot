/* SPDX-License-Identifier: GPL-3.0-only */
#include "loop_metrics.h"

#include <algorithm>

namespace gs::balance {

LoopMetrics::LoopMetrics(uint32_t requested_period_us)
    : requested_period_us_(requested_period_us) {
  reset();
}

void LoopMetrics::record(const LoopObservation &observation) {
  if (observations_ == 0u) {
    first_start_us_ = observation.actual_start_us;
    statistics_.minimum_period_us = observation.period_us;
  }
  last_start_us_ = observation.actual_start_us;
  ++observations_;
  statistics_.minimum_period_us =
      std::min(statistics_.minimum_period_us, observation.period_us);
  statistics_.maximum_period_us =
      std::max(statistics_.maximum_period_us, observation.period_us);
  const uint32_t jitter = observation.period_us > requested_period_us_
                              ? observation.period_us - requested_period_us_
                              : requested_period_us_ - observation.period_us;
  statistics_.worst_jitter_us = std::max(statistics_.worst_jitter_us, jitter);
  if (observation.period_us > requested_period_us_) {
    ++statistics_.missed_deadlines;
  }
  statistics_.maximum_execution_us =
      std::max(statistics_.maximum_execution_us, observation.execution_us);
  statistics_.sensor_sample_age_us = observation.sensor_sample_age_us;
  statistics_.motor_command_age_us = observation.motor_command_age_us;
}

LoopStatistics LoopMetrics::statistics() const {
  LoopStatistics result = statistics_;
  result.requested_frequency_hz =
      1000000.0f / static_cast<float>(requested_period_us_);
  if (observations_ > 1u && last_start_us_ > first_start_us_) {
    result.average_frequency_hz =
        static_cast<float>(observations_ - 1u) * 1000000.0f /
        static_cast<float>(last_start_us_ - first_start_us_);
  }
  return result;
}

void LoopMetrics::reset() {
  statistics_ = {};
  first_start_us_ = 0u;
  last_start_us_ = 0u;
  observations_ = 0u;
}

uint32_t LoopMetrics::ageUs(uint64_t now_us, uint64_t timestamp_us) {
  if (timestamp_us == 0u) {
    return UINT32_MAX;
  }
  if (timestamp_us >= now_us) {
    return 0u;
  }
  const uint64_t elapsed = now_us - timestamp_us;
  return elapsed > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed);
}

} // namespace gs::balance
