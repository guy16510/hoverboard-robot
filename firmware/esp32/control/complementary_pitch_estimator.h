/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "interfaces.h"

namespace gs::balance {

struct ComplementaryPitchConfig {
  float gyroscope_weight = 0.98f;
  float upright_offset_deg = 0.0f;
  float minimum_dt_seconds = 0.001f;
  float maximum_dt_seconds = 0.02f;
};

class ComplementaryPitchEstimator final : public IAttitudeEstimator {
public:
  explicit ComplementaryPitchEstimator(const ComplementaryPitchConfig &config);

  void reset(const ImuSample &sample) override;
  bool update(const ImuSample &sample) override;
  const PitchEstimate &estimate() const override;

private:
  float accelerometerPitch(const ImuSample &sample) const;

  ComplementaryPitchConfig config_;
  PitchEstimate estimate_{};
  uint64_t last_timestamp_us_ = 0u;
};

} // namespace gs::balance
