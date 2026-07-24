/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "interfaces.h"

namespace gs::balance {

struct PidConfig {
  float proportional = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float integral_limit = 0.0f;
  float output_limit = 0.0f;
};

struct CascadedBalanceConfig {
  PidConfig inner;
  PidConfig outer;
  float derivative_filter_hz = 20.0f;
  float output_limit = 250.0f;
  float slew_per_second = 500.0f;
  float maximum_pitch_reference_deg = 5.0f;
  float yaw_gain = 1.0f;
  float yaw_limit = 50.0f;
  float upright_offset_deg = 0.0f;
  float imu_sign = 1.0f;
  float wheel_sign = 1.0f;

  static CascadedBalanceConfig conservative();
};

float balanceFramePitchDeg(float imu_pitch_deg,
                           const CascadedBalanceConfig &config);

struct BalanceInput {
  float pitch_deg = 0.0f;
  float pitch_rate_dps = 0.0f;
  float wheel_velocity = 0.0f;
  float desired_velocity = 0.0f;
  float desired_yaw_rate = 0.0f;
  float dt_seconds = 0.0f;
};

struct BalanceOutput {
  float left = 0.0f;
  float right = 0.0f;
  float pitch_reference = 0.0f;
  float pitch_proportional = 0.0f;
  float pitch_integral = 0.0f;
  float pitch_derivative = 0.0f;
  float velocity_proportional = 0.0f;
  float velocity_integral = 0.0f;
  float yaw_correction = 0.0f;
  float inner_integral = 0.0f;
  float outer_integral = 0.0f;
  bool saturated = false;
  bool valid = true;
};

class CascadedBalanceController final : public IBalanceController {
public:
  explicit CascadedBalanceController(const CascadedBalanceConfig &config);

  BalanceOutput update(const BalanceInput &input) override;
  void reset(float applied_left, float applied_right) override;
  void clear() override;
  void configure(const CascadedBalanceConfig &config);
  const CascadedBalanceConfig &config() const;

private:
  float filteredPitchRate(float pitch_rate, float dt_seconds);
  float slew(float requested, float previous, float dt_seconds) const;
  static bool validInput(const BalanceInput &input);
  void clearState(float applied_left, float applied_right);

  CascadedBalanceConfig config_;
  float inner_integral_ = 0.0f;
  float outer_integral_ = 0.0f;
  float filtered_pitch_rate_ = 0.0f;
  float previous_left_ = 0.0f;
  float previous_right_ = 0.0f;
  bool hold_previous_once_ = false;
};

} // namespace gs::balance
