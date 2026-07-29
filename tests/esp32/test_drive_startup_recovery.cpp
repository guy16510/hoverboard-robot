/* SPDX-License-Identifier: GPL-3.0-only */
#include "drive_safety.h"

#include <cassert>
#include <cstdint>

extern "C" {
#include "gs_master.h"
#include "gs_protocol.h"
#include "gs_slave.h"
}

using gs::drive::DriveSafetyGate;
using gs::drive::DriveSafetyInputs;
using gs::drive::kManualDriveMode;

namespace {

gs_master_feedback exchangeCommand(gs_master_coordinator &master,
                                   gs_slave_coordinator &slave,
                                   const gs_esp_command &command,
                                   uint32_t now_ms) {
  uint8_t esp_frame[GS_ESP_COMMAND_SIZE] = {};
  uint8_t slave_command_frame[GS_SLAVE_COMMAND_SIZE] = {};
  uint8_t slave_feedback_frame[GS_SLAVE_FEEDBACK_SIZE] = {};
  uint8_t master_feedback_frame[GS_MASTER_FEEDBACK_SIZE] = {};
  gs_master_feedback feedback{};

  assert(gs_encode_esp_command(esp_frame, &command));
  assert(gs_master_accept_esp_frame(&master, esp_frame, now_ms));
  assert(gs_master_make_slave_frame(&master, slave_command_frame, now_ms + 1u));
  assert(
      gs_slave_accept_master_frame(&slave, slave_command_frame, now_ms + 1u));
  assert(gs_slave_make_feedback(&slave, slave_feedback_frame, now_ms + 2u));
  assert(gs_master_accept_slave_feedback(&master, slave_feedback_frame,
                                         now_ms + 2u));
  assert(gs_master_make_feedback(&master, master_feedback_frame, now_ms + 3u));
  assert(gs_decode_master_feedback(&feedback, master_feedback_frame));
  return feedback;
}

DriveSafetyInputs safetyInputs(const gs_master_feedback &feedback,
                               uint16_t sequence) {
  DriveSafetyInputs inputs;
  inputs.serial_connected = true;
  inputs.lease_active = false;
  inputs.command_transport_ready = true;
  inputs.feedback_fresh = true;
  inputs.feedback_runtime_healthy =
      gs_master_feedback_runtime_healthy(&feedback);
  inputs.exact_zero_acknowledged =
      gs_master_feedback_motion_ready(&feedback, sequence, true);
  inputs.imu_healthy = true;
  inputs.master_faults = feedback.master_faults;
  inputs.slave_faults = feedback.slave_faults;
  inputs.pitch_deg = 0.0f;
  inputs.roll_deg = 0.0f;
  return inputs;
}

DriveSafetyGate neutralManualGate() {
  DriveSafetyGate gate;
  gate.onConnectionEstablished();
  gate.setOperatingMode(kManualDriveMode);
  gate.observeDemand(0.0f, 0.0f);
  return gate;
}

} // namespace

int main() {
  gs_master_coordinator master{};
  gs_slave_coordinator slave{};
  gs_master_init(&master, 0u);
  gs_slave_init(&slave, 0u);

  // Match the healthy lifted-wheel hardware inputs while both controllers are
  // intentionally disabled: valid Hall states, PA4 bypassed, no bridge output,
  // and no controller fault words.
  gs_master_set_runtime_status(&master, true, true);
  gs_master_set_motor_status(&master, 2u, 0u, false);
  gs_slave_set_motor_status(&slave, 2u, 0u, false, true);

  gs_esp_command disabled{};
  disabled.master_flags = GS_COMMAND_DIRECT_LR | GS_COMMAND_DISABLE;
  disabled.slave_flags = GS_COMMAND_DISABLE;
  disabled.sequence = 1u;
  const gs_master_feedback disabled_feedback =
      exchangeCommand(master, slave, disabled, 1u);

  assert(disabled_feedback.master_state == GS_CONTROLLER_DISABLED);
  assert(disabled_feedback.slave_state == GS_CONTROLLER_DISABLED);
  assert((disabled_feedback.status_flags & GS_FEEDBACK_PEER_HEALTHY) == 0u);
  assert(gs_master_feedback_exact_ack(&disabled_feedback, 1u, true));
  assert(!gs_master_feedback_runtime_healthy(&disabled_feedback));
  assert(!gs_master_feedback_motion_ready(&disabled_feedback, 1u, true));
  assert(disabled_feedback.left_applied == 0);
  assert(disabled_feedback.right_applied == 0);

  DriveSafetyGate gate = neutralManualGate();
  DriveSafetyInputs inputs = safetyInputs(disabled_feedback, 1u);

  // This is the observed deadlock state: feedback is fresh and the disabled
  // command is acknowledged, but peer health and motion-ready zero are absent.
  // The production safety gate must permit only the non-energizing READY-zero
  // transition, not ARM or motor output.
  assert(!inputs.feedback_runtime_healthy);
  assert(!inputs.exact_zero_acknowledged);
  assert(gate.zeroEstablishmentAllowed(inputs));
  assert(!gate.requestArm(inputs));
  assert(!gate.armed());
  assert(!gate.outputEnabled(inputs));

  gs_esp_command ready_zero{};
  if (gate.zeroEstablishmentAllowed(inputs)) {
    ready_zero.master_flags = GS_COMMAND_DIRECT_LR;
  } else {
    ready_zero.master_flags = GS_COMMAND_DISABLE;
    ready_zero.slave_flags = GS_COMMAND_DISABLE;
  }
  ready_zero.sequence = 2u;

  assert((ready_zero.master_flags & GS_COMMAND_DIRECT_LR) != 0u);
  assert((ready_zero.master_flags & GS_COMMAND_DISABLE) == 0u);
  assert((ready_zero.slave_flags & GS_COMMAND_DISABLE) == 0u);
  assert(ready_zero.speed == 0);
  assert(ready_zero.steer == 0);

  const gs_master_feedback ready_feedback =
      exchangeCommand(master, slave, ready_zero, 10u);
  assert(ready_feedback.master_state == GS_CONTROLLER_READY);
  assert(ready_feedback.slave_state == GS_CONTROLLER_READY);
  assert((ready_feedback.status_flags & GS_FEEDBACK_PEER_HEALTHY) != 0u);
  assert(gs_master_feedback_exact_ack(&ready_feedback, 2u, true));
  assert(gs_master_feedback_runtime_healthy(&ready_feedback));
  assert(gs_master_feedback_motion_ready(&ready_feedback, 2u, true));
  assert(ready_feedback.left_applied == 0);
  assert(ready_feedback.right_applied == 0);
  assert((ready_feedback.motor_status_flags &
          (GS_MASTER_MOTOR_LEFT_BRIDGE_ENABLED |
           GS_MASTER_MOTOR_RIGHT_BRIDGE_ENABLED)) == 0u);

  inputs = safetyInputs(ready_feedback, 2u);
  inputs.lease_active = true;
  assert(!gate.outputEnabled(inputs));
  assert(gate.requestArm(inputs));
  assert(gate.armed());
  assert(gate.outputEnabled(inputs));

  // The startup exception is deliberately narrow. It must never bypass stale
  // feedback, controller faults, unsafe orientation, or local disarm.
  DriveSafetyGate blocked_gate = neutralManualGate();
  DriveSafetyInputs blocked = safetyInputs(disabled_feedback, 1u);
  blocked.feedback_fresh = false;
  assert(!blocked_gate.zeroEstablishmentAllowed(blocked));
  blocked = safetyInputs(disabled_feedback, 1u);
  blocked.master_faults = 1u;
  assert(!blocked_gate.zeroEstablishmentAllowed(blocked));
  blocked = safetyInputs(disabled_feedback, 1u);
  blocked.local_disarm = true;
  assert(!blocked_gate.zeroEstablishmentAllowed(blocked));
  blocked = safetyInputs(disabled_feedback, 1u);
  blocked.pitch_deg = 45.0f;
  assert(!blocked_gate.zeroEstablishmentAllowed(blocked));

  return 0;
}
