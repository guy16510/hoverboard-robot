/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::balance {

enum class BalanceState : uint8_t {
  kBoot,
  kImuCalibrating,
  kDisarmed,
  kArmedBalance,
  kDriving,
  kFallen,
  kFault,
};

struct SafetySnapshot {
  bool imu_healthy = false;
  bool calibrated = false;
  bool approximately_upright = false;
  bool motor_feedback_healthy = false;
  bool motor_transport_healthy = false;
  bool zero_output_acknowledged = false;
  bool loop_healthy = false;
  bool controller_fault = false;
};

struct BalanceStateConfig {
  float fall_angle_deg = 20.0f;
  uint32_t fall_duration_us = 100000u;
};

class BalanceStateMachine {
public:
  explicit BalanceStateMachine(const BalanceStateConfig &config);

  void update(const SafetySnapshot &safety, float pitch_deg, uint64_t now_us);
  bool arm(const SafetySnapshot &safety);
  void disarm();
  void setDriving(bool driving);
  bool clearFault(const SafetySnapshot &safety);
  BalanceState state() const;
  bool outputEnabled() const;

private:
  bool healthyForArming(const SafetySnapshot &safety) const;
  static bool healthyForDiagnostic(const SafetySnapshot &safety);

  BalanceStateConfig config_;
  BalanceState state_ = BalanceState::kBoot;
  uint64_t fall_started_us_ = 0u;
};

} // namespace gs::balance
