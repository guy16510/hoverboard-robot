/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::drive {

constexpr uint8_t kManualDriveMode = 2u;
constexpr float kMaximumSafeTiltDeg = 45.0f;

struct DriveSafetyInputs {
  bool serial_connected = false;
  bool lease_active = false;
  bool command_transport_ready = false;
  bool feedback_fresh = false;
  bool feedback_runtime_healthy = false;
  bool exact_zero_acknowledged = false;
  bool imu_healthy = false;
  bool acknowledgment_timed_out = false;
  bool malformed_command = false;
  bool local_disarm = false;
  uint32_t master_faults = 0u;
  uint32_t slave_faults = 0u;
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
};

enum DriveSafetyFault : uint32_t {
  kDriveFaultSerialDisconnected = 1u << 0u,
  kDriveFaultLeaseExpired = 1u << 1u,
  kDriveFaultTransportUnavailable = 1u << 2u,
  kDriveFaultFeedbackLost = 1u << 3u,
  kDriveFaultControllerUnhealthy = 1u << 4u,
  kDriveFaultZeroNotAcknowledged = 1u << 5u,
  kDriveFaultImuUnhealthy = 1u << 6u,
  kDriveFaultUnsafeOrientation = 1u << 7u,
  kDriveFaultAcknowledgmentTimeout = 1u << 8u,
  kDriveFaultMalformedCommand = 1u << 9u,
  kDriveFaultLocalDisarm = 1u << 10u,
  kDriveFaultMasterController = 1u << 11u,
  kDriveFaultSlaveController = 1u << 12u,
  kDriveFaultNonNeutralArm = 1u << 13u,
  kDriveFaultWrongOperatingMode = 1u << 14u,
};

class DriveSafetyGate {
public:
  void onConnectionEstablished();
  void onConnectionLost();
  void setOperatingMode(uint8_t mode);
  void observeDemand(float linear_velocity, float yaw_rate);
  bool requestArm(const DriveSafetyInputs &inputs);
  void disarm(uint32_t reason = 0u);
  void clearRecoverableFaults(const DriveSafetyInputs &inputs);
  void evaluate(const DriveSafetyInputs &inputs);

  bool armed() const;
  bool neutralObserved() const;
  uint8_t operatingMode() const;
  uint32_t faults() const;
  bool outputEnabled(const DriveSafetyInputs &inputs) const;

  static uint32_t currentFaults(const DriveSafetyInputs &inputs,
                                bool require_zero_ack);

private:
  static bool nearZero(float value);

  uint8_t operating_mode_ = 0u;
  bool armed_ = false;
  bool neutral_observed_ = false;
  uint32_t latched_faults_ = 0u;
};

} // namespace gs::drive
