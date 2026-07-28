/* SPDX-License-Identifier: GPL-3.0-only */
#include "controller_fault_clear.h"

namespace gs::balance {

void ControllerFaultClear::request() { pending_ = true; }

void ControllerFaultClear::apply(gs_esp_command &command) const {
  if (!pending_) {
    return;
  }
  command.speed = 0;
  command.steer = 0;
  command.master_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT;
  command.slave_flags = GS_COMMAND_DISABLE | GS_COMMAND_CLEAR_FAULT;
}

bool ControllerFaultClear::observe(const gs_master_feedback &feedback,
                                   uint16_t expected_sequence,
                                   bool command_sent) {
  const bool exact_ack =
      command_sent &&
      feedback.accepted_esp_sequence == expected_sequence &&
      feedback.forwarded_slave_sequence == expected_sequence &&
      feedback.accepted_slave_sequence == expected_sequence;
  if (!pending_ || !exact_ack ||
      feedback.master_faults != 0u || feedback.slave_faults != 0u ||
      (feedback.status_flags & GS_FEEDBACK_CLEAR_PENDING) != 0u) {
    return false;
  }
  pending_ = false;
  return true;
}

bool ControllerFaultClear::pending() const { return pending_; }

} // namespace gs::balance
