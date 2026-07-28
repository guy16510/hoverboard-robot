/* SPDX-License-Identifier: GPL-3.0-only */
#include "drive_safety.h"

#include <cassert>
#include <limits>

using namespace gs::drive;

namespace {

DriveSafetyInputs healthy() {
  DriveSafetyInputs inputs;
  inputs.serial_connected = true;
  inputs.lease_active = true;
  inputs.command_transport_ready = true;
  inputs.feedback_fresh = true;
  inputs.feedback_runtime_healthy = true;
  inputs.exact_zero_acknowledged = true;
  inputs.imu_healthy = true;
  return inputs;
}

} // namespace

int main() {
  DriveSafetyGate gate;
  auto inputs = healthy();

  gate.onConnectionEstablished();
  gate.setOperatingMode(kManualDriveMode);
  assert(!gate.requestArm(inputs));
  assert(gate.faults() & kDriveFaultNonNeutralArm);

  gate.observeDemand(0.0f, 0.0f);
  assert(gate.requestArm(inputs));
  assert(gate.armed());
  assert(gate.outputEnabled(inputs));

  inputs.lease_active = false;
  gate.evaluate(inputs);
  assert(!gate.armed());
  assert(gate.faults() & kDriveFaultLeaseExpired);
  assert(!gate.outputEnabled(inputs));

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.faults() == 0u);
  assert(gate.requestArm(inputs));

  inputs.master_faults = 1u;
  gate.evaluate(inputs);
  assert(!gate.armed());
  assert(gate.faults() & kDriveFaultMasterController);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.slave_faults = 1u;
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultSlaveController);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.feedback_fresh = false;
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultFeedbackLost);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.acknowledgment_timed_out = true;
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultAcknowledgmentTimeout);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.feedback_crc_error = true;
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultFeedbackCrc);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.pitch_deg = 46.0f;
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultUnsafeOrientation);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.roll_deg = -46.0f;
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultUnsafeOrientation);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  assert(gate.requestArm(inputs));
  inputs.pitch_deg = std::numeric_limits<float>::quiet_NaN();
  gate.evaluate(inputs);
  assert(gate.faults() & kDriveFaultUnsafeOrientation);

  inputs = healthy();
  gate.clearRecoverableFaults(inputs);
  gate.setOperatingMode(1u);
  assert(!gate.requestArm(inputs));
  assert(gate.faults() & kDriveFaultWrongOperatingMode);

  return 0;
}
