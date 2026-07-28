/* SPDX-License-Identifier: GPL-3.0-only */
#include "drive_safety.h"

#include <cmath>

namespace gs::drive {

void DriveSafetyGate::onConnectionEstablished() {
  armed_ = false;
  neutral_observed_ = false;
  latched_faults_ &= ~(kDriveFaultSerialDisconnected | kDriveFaultLeaseExpired);
}

void DriveSafetyGate::onConnectionLost() {
  armed_ = false;
  neutral_observed_ = false;
  latched_faults_ |= kDriveFaultSerialDisconnected;
}

void DriveSafetyGate::setOperatingMode(uint8_t mode) {
  if (operating_mode_ != mode) {
    armed_ = false;
    neutral_observed_ = false;
  }
  operating_mode_ = mode;
  if (mode != kManualDriveMode) {
    latched_faults_ |= kDriveFaultWrongOperatingMode;
  } else {
    latched_faults_ &= ~kDriveFaultWrongOperatingMode;
  }
}

void DriveSafetyGate::observeDemand(float linear_velocity, float yaw_rate) {
  if (nearZero(linear_velocity) && nearZero(yaw_rate)) {
    neutral_observed_ = true;
    latched_faults_ &= ~kDriveFaultNonNeutralArm;
  }
}

bool DriveSafetyGate::requestArm(const DriveSafetyInputs &inputs) {
  const uint32_t current = currentFaults(inputs, true);
  if (operating_mode_ != kManualDriveMode) {
    latched_faults_ |= kDriveFaultWrongOperatingMode;
    armed_ = false;
    return false;
  }
  if (!neutral_observed_) {
    latched_faults_ |= kDriveFaultNonNeutralArm;
    armed_ = false;
    return false;
  }
  if (current != 0u) {
    latched_faults_ |= current;
    armed_ = false;
    return false;
  }
  latched_faults_ &= ~(kDriveFaultNonNeutralArm |
                       kDriveFaultWrongOperatingMode |
                       kDriveFaultZeroNotAcknowledged |
                       kDriveFaultLeaseExpired |
                       kDriveFaultSerialDisconnected);
  if (latched_faults_ != 0u) {
    armed_ = false;
    return false;
  }
  armed_ = true;
  return true;
}

void DriveSafetyGate::disarm(uint32_t reason) {
  armed_ = false;
  latched_faults_ |= reason;
}

void DriveSafetyGate::clearRecoverableFaults(const DriveSafetyInputs &inputs) {
  if (currentFaults(inputs, true) == 0u &&
      operating_mode_ == kManualDriveMode && neutral_observed_) {
    latched_faults_ = 0u;
  }
}

void DriveSafetyGate::evaluate(const DriveSafetyInputs &inputs) {
  const uint32_t current = currentFaults(inputs, false);
  if (current != 0u) {
    latched_faults_ |= current;
    armed_ = false;
  }
  if (armed_ && !inputs.lease_active) {
    latched_faults_ |= kDriveFaultLeaseExpired;
    armed_ = false;
  }
}

bool DriveSafetyGate::armed() const { return armed_; }

bool DriveSafetyGate::neutralObserved() const { return neutral_observed_; }

uint8_t DriveSafetyGate::operatingMode() const { return operating_mode_; }

uint32_t DriveSafetyGate::faults() const { return latched_faults_; }

bool DriveSafetyGate::outputEnabled(const DriveSafetyInputs &inputs) const {
  return armed_ && latched_faults_ == 0u &&
         operating_mode_ == kManualDriveMode &&
         currentFaults(inputs, false) == 0u && inputs.lease_active;
}

uint32_t DriveSafetyGate::currentFaults(const DriveSafetyInputs &inputs,
                                        bool require_zero_ack) {
  uint32_t faults = 0u;
  faults |= inputs.serial_connected ? 0u : kDriveFaultSerialDisconnected;
  faults |= inputs.command_transport_ready ? 0u
                                            : kDriveFaultTransportUnavailable;
  faults |= inputs.feedback_fresh ? 0u : kDriveFaultFeedbackLost;
  faults |= inputs.feedback_runtime_healthy ? 0u
                                             : kDriveFaultControllerUnhealthy;
  faults |= inputs.imu_healthy ? 0u : kDriveFaultImuUnhealthy;
  faults |= inputs.acknowledgment_timed_out
                ? kDriveFaultAcknowledgmentTimeout
                : 0u;
  faults |= inputs.feedback_crc_error ? kDriveFaultFeedbackCrc : 0u;
  faults |= inputs.malformed_command ? kDriveFaultMalformedCommand : 0u;
  faults |= inputs.local_disarm ? kDriveFaultLocalDisarm : 0u;
  faults |= inputs.master_faults != 0u ? kDriveFaultMasterController : 0u;
  faults |= inputs.slave_faults != 0u ? kDriveFaultSlaveController : 0u;
  if (require_zero_ack && !inputs.exact_zero_acknowledged) {
    faults |= kDriveFaultZeroNotAcknowledged;
  }
  if (!std::isfinite(inputs.pitch_deg) || !std::isfinite(inputs.roll_deg) ||
      std::fabs(inputs.pitch_deg) >= kMaximumSafeTiltDeg ||
      std::fabs(inputs.roll_deg) >= kMaximumSafeTiltDeg) {
    faults |= kDriveFaultUnsafeOrientation;
  }
  return faults;
}

bool DriveSafetyGate::nearZero(float value) {
  return std::isfinite(value) && std::fabs(value) <= 0.0001f;
}

} // namespace gs::drive
