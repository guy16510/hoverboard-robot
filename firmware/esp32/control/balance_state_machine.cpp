/* SPDX-License-Identifier: GPL-3.0-only */
#include "balance_state_machine.h"

#include <cmath>

namespace gs::balance {

BalanceStateMachine::BalanceStateMachine(const BalanceStateConfig &config)
    : config_(config) {}

void BalanceStateMachine::update(const SafetySnapshot &safety, float pitch_deg,
                                 uint64_t now_us) {
  if (state_ == BalanceState::kBoot) {
    state_ = BalanceState::kImuCalibrating;
    return;
  }
  if (state_ == BalanceState::kImuCalibrating) {
    if (!safety.imu_healthy) {
      state_ = BalanceState::kFault;
      return;
    }
    if (safety.calibrated) {
      state_ = BalanceState::kDisarmed;
    }
    return;
  }
  if (!safety.imu_healthy || !safety.loop_healthy || safety.controller_fault) {
    state_ = BalanceState::kFault;
    fall_started_us_ = 0u;
    return;
  }
  if ((state_ == BalanceState::kArmedBalance ||
       state_ == BalanceState::kDriving) &&
      (!safety.motor_feedback_healthy || !safety.motor_transport_healthy)) {
    state_ = BalanceState::kFault;
    fall_started_us_ = 0u;
    return;
  }
  if (state_ != BalanceState::kArmedBalance &&
      state_ != BalanceState::kDriving) {
    return;
  }
  if (std::fabs(pitch_deg) <= config_.fall_angle_deg) {
    fall_started_us_ = 0u;
    return;
  }
  if (fall_started_us_ == 0u) {
    fall_started_us_ = now_us;
    return;
  }
  if (now_us - fall_started_us_ >= config_.fall_duration_us) {
    state_ = BalanceState::kFallen;
    fall_started_us_ = 0u;
  }
}

bool BalanceStateMachine::arm(const SafetySnapshot &safety) {
  if (state_ == BalanceState::kFault || !healthyForArming(safety)) {
    return false;
  }
  if (state_ != BalanceState::kDisarmed && state_ != BalanceState::kFallen) {
    return false;
  }
  state_ = BalanceState::kArmedBalance;
  fall_started_us_ = 0u;
  return true;
}

void BalanceStateMachine::disarm() {
  if (state_ != BalanceState::kFault) {
    state_ = BalanceState::kDisarmed;
  }
  fall_started_us_ = 0u;
}

void BalanceStateMachine::setDriving(bool driving) {
  if (driving && state_ == BalanceState::kArmedBalance) {
    state_ = BalanceState::kDriving;
    return;
  }
  if (!driving && state_ == BalanceState::kDriving) {
    state_ = BalanceState::kArmedBalance;
  }
}

bool BalanceStateMachine::clearFault(const SafetySnapshot &safety) {
  if (state_ != BalanceState::kFault || !healthyForDiagnostic(safety)) {
    return false;
  }
  state_ = BalanceState::kDisarmed;
  return true;
}

BalanceState BalanceStateMachine::state() const { return state_; }

bool BalanceStateMachine::outputEnabled() const {
  return state_ == BalanceState::kArmedBalance ||
         state_ == BalanceState::kDriving;
}

bool BalanceStateMachine::healthyForArming(const SafetySnapshot &safety) const {
  return safety.imu_healthy && safety.calibrated &&
         safety.approximately_upright && safety.motor_feedback_healthy &&
         safety.motor_transport_healthy && safety.zero_output_acknowledged &&
         safety.loop_healthy && !safety.controller_fault;
}

bool BalanceStateMachine::healthyForDiagnostic(const SafetySnapshot &safety) {
  return safety.imu_healthy && safety.calibrated && safety.loop_healthy &&
         !safety.controller_fault;
}

} // namespace gs::balance
