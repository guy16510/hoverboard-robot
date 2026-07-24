/* SPDX-License-Identifier: GPL-3.0-only */
#include "balance_controller.h"

#include <algorithm>
#include <cmath>

namespace gs::balance {
namespace {

float limit(float value, float magnitude) {
  return std::clamp(value, -magnitude, magnitude);
}

float activeLimit(float primary, float secondary) {
  if (secondary <= 0.0f) {
    return primary;
  }
  return std::min(primary, secondary);
}

bool drivesFurtherIntoSaturation(float requested, float limited, float error) {
  return (requested > limited && error > 0.0f) ||
         (requested < limited && error < 0.0f);
}

} // namespace

CascadedBalanceConfig CascadedBalanceConfig::conservative() {
  CascadedBalanceConfig config;
  config.inner = {8.0f, 0.8f, 0.25f, 20.0f, 250.0f};
  config.outer = {0.5f, 0.1f, 0.0f, 10.0f, 5.0f};
  return config;
}

float balanceFramePitchDeg(float imu_pitch_deg,
                           const CascadedBalanceConfig &config) {
  return imu_pitch_deg * config.imu_sign - config.upright_offset_deg;
}

CascadedBalanceController::CascadedBalanceController(
    const CascadedBalanceConfig &config)
    : config_(config) {}

BalanceOutput CascadedBalanceController::update(const BalanceInput &input) {
  if (!validInput(input)) {
    reset(0.0f, 0.0f);
    BalanceOutput invalid;
    invalid.valid = false;
    return invalid;
  }
  if (hold_previous_once_) {
    hold_previous_once_ = false;
    BalanceOutput held;
    held.left = previous_left_;
    held.right = previous_right_;
    return held;
  }

  const float velocity_error =
      input.desired_velocity - input.wheel_velocity * config_.wheel_sign;
  const float previous_outer_integral = outer_integral_;
  outer_integral_ =
      limit(previous_outer_integral + velocity_error * input.dt_seconds,
            config_.outer.integral_limit);
  const float velocity_proportional =
      velocity_error * config_.outer.proportional;
  float velocity_integral = outer_integral_ * config_.outer.integral;
  float unrestricted_pitch_reference =
      velocity_proportional + velocity_integral;
  const float pitch_reference_limit = activeLimit(
      config_.maximum_pitch_reference_deg, config_.outer.output_limit);
  float pitch_reference =
      limit(unrestricted_pitch_reference, pitch_reference_limit);
  if (drivesFurtherIntoSaturation(unrestricted_pitch_reference, pitch_reference,
                                  velocity_error)) {
    outer_integral_ = previous_outer_integral;
    velocity_integral = outer_integral_ * config_.outer.integral;
    unrestricted_pitch_reference = velocity_proportional + velocity_integral;
    pitch_reference =
        limit(unrestricted_pitch_reference, pitch_reference_limit);
  }

  const float pitch = balanceFramePitchDeg(input.pitch_deg, config_);
  const float pitch_error = pitch_reference - pitch;
  const float previous_inner_integral = inner_integral_;
  inner_integral_ =
      limit(previous_inner_integral + pitch_error * input.dt_seconds,
            config_.inner.integral_limit);
  const float pitch_proportional = pitch_error * config_.inner.proportional;
  float pitch_integral = inner_integral_ * config_.inner.integral;
  const float pitch_derivative =
      -filteredPitchRate(input.pitch_rate_dps * config_.imu_sign,
                         input.dt_seconds) *
      config_.inner.derivative;

  const float yaw =
      limit(input.desired_yaw_rate * config_.yaw_gain, config_.yaw_limit);
  const float unrestricted_common =
      pitch_proportional + pitch_integral + pitch_derivative;
  const float output_limit =
      activeLimit(config_.output_limit, config_.inner.output_limit);
  const float common_limit = std::max(0.0f, output_limit - std::fabs(yaw));
  float common = limit(unrestricted_common, common_limit);
  if (drivesFurtherIntoSaturation(unrestricted_common, common, pitch_error)) {
    inner_integral_ = previous_inner_integral;
    pitch_integral = inner_integral_ * config_.inner.integral;
    common = limit(pitch_proportional + pitch_integral + pitch_derivative,
                   common_limit);
  }
  const float requested_left = common + yaw;
  const float requested_right = common - yaw;

  BalanceOutput output;
  output.left = slew(requested_left, previous_left_, input.dt_seconds);
  output.right = slew(requested_right, previous_right_, input.dt_seconds);
  output.pitch_reference = pitch_reference;
  output.pitch_proportional = pitch_proportional;
  output.pitch_integral = pitch_integral;
  output.pitch_derivative = pitch_derivative;
  output.velocity_proportional = velocity_proportional;
  output.velocity_integral = velocity_integral;
  output.yaw_correction = yaw;
  output.inner_integral = inner_integral_;
  output.outer_integral = outer_integral_;
  output.saturated = common != unrestricted_common ||
                     output.left != requested_left ||
                     output.right != requested_right;
  previous_left_ = output.left;
  previous_right_ = output.right;
  return output;
}

void CascadedBalanceController::reset(float applied_left, float applied_right) {
  clearState(applied_left, applied_right);
  hold_previous_once_ = true;
}

void CascadedBalanceController::clear() { clearState(0.0f, 0.0f); }

void CascadedBalanceController::clearState(float applied_left,
                                           float applied_right) {
  inner_integral_ = 0.0f;
  outer_integral_ = 0.0f;
  filtered_pitch_rate_ = 0.0f;
  previous_left_ = applied_left;
  previous_right_ = applied_right;
  hold_previous_once_ = false;
}

void CascadedBalanceController::configure(const CascadedBalanceConfig &config) {
  config_ = config;
  reset(0.0f, 0.0f);
}

const CascadedBalanceConfig &CascadedBalanceController::config() const {
  return config_;
}

float CascadedBalanceController::filteredPitchRate(float pitch_rate,
                                                   float dt_seconds) {
  constexpr float kTwoPi = 6.283185307179586f;
  const float coefficient =
      1.0f - std::exp(-kTwoPi * config_.derivative_filter_hz * dt_seconds);
  filtered_pitch_rate_ += coefficient * (pitch_rate - filtered_pitch_rate_);
  return filtered_pitch_rate_;
}

float CascadedBalanceController::slew(float requested, float previous,
                                      float dt_seconds) const {
  const float maximum_step = config_.slew_per_second * dt_seconds;
  return previous + limit(requested - previous, maximum_step);
}

bool CascadedBalanceController::validInput(const BalanceInput &input) {
  return std::isfinite(input.pitch_deg) &&
         std::isfinite(input.pitch_rate_dps) &&
         std::isfinite(input.wheel_velocity) &&
         std::isfinite(input.desired_velocity) &&
         std::isfinite(input.desired_yaw_rate) &&
         std::isfinite(input.dt_seconds) && input.dt_seconds > 0.0f &&
         input.dt_seconds <= 0.1f;
}

} // namespace gs::balance
