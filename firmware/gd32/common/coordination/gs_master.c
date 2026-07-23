/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_master.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "gs_safety.h"
#include "gs_wheel_mix.h"

static void stop_master(gs_master_coordinator *master,
                        gs_controller_state state) {
  master->requested = (gs_wheel_pair){0, 0};
  master->demanded = (gs_wheel_pair){0, 0};
  master->applied = (gs_wheel_pair){0, 0};
  master->state = state;
  master->slave_flags |= GS_COMMAND_DISABLE;
}

static uint32_t first_fault_bit(uint32_t faults) {
  return faults & (uint32_t)(~faults + 1u);
}

static bool command_equal(const gs_esp_command *first,
                          const gs_esp_command *second) {
  return first->speed == second->speed && first->steer == second->steer &&
         first->master_flags == second->master_flags &&
         first->slave_flags == second->slave_flags &&
         first->sequence == second->sequence &&
         first->enable_epoch == second->enable_epoch &&
         first->master_clear_fault_epoch ==
             second->master_clear_fault_epoch &&
         first->slave_clear_fault_epoch == second->slave_clear_fault_epoch;
}

static bool serial_is_newer(uint16_t value, uint16_t previous) {
  const uint16_t delta = (uint16_t)(value - previous);
  return delta != 0u && delta < 0x8000u;
}

static bool slave_state_healthy(uint8_t state) {
  return state == GS_CONTROLLER_READY || state == GS_CONTROLLER_ACTIVE;
}

void gs_master_init(gs_master_coordinator *master, uint32_t now_ms) {
  if (master == NULL) {
    return;
  }
  memset(master, 0, sizeof(*master));
  master->state = GS_CONTROLLER_DISABLED;
  master->last_esp_command_ms = now_ms;
  master->last_new_esp_sequence_ms = now_ms;
  master->last_slave_feedback_ms = now_ms;
  master->slave_flags = GS_COMMAND_DISABLE;
  master->last_clear_result = GS_CLEAR_NONE;
}

void gs_master_latch_fault(gs_master_coordinator *master, uint32_t fault) {
  if (master == NULL || fault == 0u) {
    return;
  }
  const uint32_t new_faults = fault & ~master->faults;
  if (new_faults != 0u) {
    if (master->faults == 0u) {
      master->first_fault = first_fault_bit(new_faults);
    }
    ++master->fault_epoch;
    if (master->fault_epoch == 0u) {
      ++master->fault_epoch;
    }
  }
  master->faults |= fault;
  master->clear_fault_pending = false;
  master->enable_epoch_valid = false;
  master->recovery_required = true;
  master->last_clear_result = GS_CLEAR_NONE;
  stop_master(master, GS_CONTROLLER_FAULTED);
}

bool gs_master_peer_healthy(const gs_master_coordinator *master,
                            uint32_t now_ms) {
  return master != NULL && master->enable_epoch_valid &&
         master->slave_feedback_seen &&
         (uint32_t)(now_ms - master->last_slave_feedback_ms) <=
             GS_SLAVE_TIMEOUT_MS &&
         slave_state_healthy(master->slave_feedback.state) &&
         master->slave_feedback.faults == 0u &&
         master->slave_feedback.enable_epoch == master->enable_epoch;
}

static bool clear_request_valid(gs_master_coordinator *master,
                                const gs_esp_command *command) {
  if ((command->master_flags & GS_COMMAND_CLEAR_FAULT) == 0u) {
    return true;
  }
  if ((command->master_flags & GS_COMMAND_DISABLE) == 0u) {
    master->last_clear_result = GS_CLEAR_REJECT_STATE;
    return false;
  }
  if (command->master_clear_fault_epoch != master->fault_epoch) {
    master->last_clear_result = GS_CLEAR_REJECT_STALE_EPOCH;
    return false;
  }
  if ((master->faults &
       (GS_FAULT_WATCHDOG_LOCKOUT | GS_FAULT_SHUTDOWN)) != 0u) {
    master->last_clear_result = GS_CLEAR_REQUIRES_RESET;
    return false;
  }
  return true;
}

static bool session_request_valid(gs_master_coordinator *master,
                                  const gs_esp_command *command,
                                  gs_wheel_pair requested, uint32_t now_ms) {
  if ((command->master_flags &
       (GS_COMMAND_DISABLE | GS_COMMAND_SHUTDOWN)) != 0u) {
    return true;
  }
  if (master->shutdown || master->faults != 0u || command->enable_epoch == 0u) {
    return false;
  }
  const bool moving = requested.left != 0 || requested.right != 0;
  if (moving) {
    return master->enable_epoch_valid && !master->recovery_required &&
           command->enable_epoch == master->enable_epoch &&
           gs_master_peer_healthy(master, now_ms);
  }
  if (master->enable_epoch_valid) {
    return command->enable_epoch == master->enable_epoch;
  }
  return master->enable_epoch == 0u ||
         serial_is_newer(command->enable_epoch, master->enable_epoch);
}

bool gs_master_accept_esp_frame(gs_master_coordinator *master,
                                const uint8_t frame[GS_ESP_COMMAND_SIZE],
                                uint32_t now_ms) {
  gs_esp_command command;
  if (master == NULL || !gs_decode_esp_command(&command, frame)) {
    if (master != NULL) {
      ++master->invalid_esp_frames;
    }
    return false;
  }

  const gs_wheel_pair requested =
      (command.master_flags & GS_COMMAND_DIRECT_LR) != 0u
          ? gs_direct_wheels(command.speed, command.steer)
          : gs_mix_wheels(command.speed, command.steer);
  if (!clear_request_valid(master, &command) ||
      !session_request_valid(master, &command, requested, now_ms)) {
    ++master->invalid_esp_frames;
    return false;
  }

  if (master->esp_seen && command.sequence == master->last_esp_sequence) {
    if (!command_equal(&command, &master->last_esp_command)) {
      ++master->invalid_esp_frames;
      return false;
    }
    master->last_esp_command_ms = now_ms;
    return true;
  }
  if (master->esp_seen &&
      !serial_is_newer(command.sequence, master->last_esp_sequence)) {
    ++master->invalid_esp_frames;
    return false;
  }
  if (master->esp_seen) {
    const uint16_t delta =
        (uint16_t)(command.sequence - master->last_esp_sequence);
    if (delta > 1u) {
      master->missing_esp_sequences += (uint32_t)(delta - 1u);
    }
  }

  master->last_esp_command = command;
  master->last_esp_sequence = command.sequence;
  master->last_esp_command_ms = now_ms;
  master->last_new_esp_sequence_ms = now_ms;
  master->esp_seen = true;
  ++master->valid_esp_frames;
  master->slave_flags = command.slave_flags;

  if ((command.master_flags & GS_COMMAND_SHUTDOWN) != 0u) {
    master->clear_fault_pending = false;
    master->enable_epoch_valid = false;
    master->shutdown = true;
    master->slave_flags |= GS_COMMAND_SHUTDOWN | GS_COMMAND_DISABLE;
    stop_master(master, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if ((command.master_flags & GS_COMMAND_DISABLE) != 0u) {
    master->enable_epoch_valid = false;
    if ((command.master_flags & GS_COMMAND_CLEAR_FAULT) != 0u) {
      master->clear_fault_pending = true;
      master->last_clear_result = GS_CLEAR_NONE;
    }
    master->slave_flags |= GS_COMMAND_DISABLE;
    stop_master(master, GS_CONTROLLER_DISABLED);
    return true;
  }

  master->clear_fault_pending = false;
  if (!master->enable_epoch_valid) {
    master->enable_epoch = command.enable_epoch;
    master->enable_epoch_valid = true;
  }
  master->requested = requested;
  master->demanded = requested;
  master->state = requested.left == 0 && requested.right == 0
                      ? GS_CONTROLLER_READY
                      : GS_CONTROLLER_ACTIVE;
  return true;
}

void gs_master_tick(gs_master_coordinator *master, uint32_t now_ms) {
  if (master == NULL || !master->esp_seen ||
      (master->state != GS_CONTROLLER_READY &&
       master->state != GS_CONTROLLER_ACTIVE)) {
    return;
  }
  if ((uint32_t)(now_ms - master->last_esp_command_ms) > GS_ESP_TIMEOUT_MS) {
    gs_master_latch_fault(master, GS_FAULT_COMMAND_TIMEOUT);
    return;
  }
  if (master->state != GS_CONTROLLER_ACTIVE) {
    return;
  }
  if (!gs_master_peer_healthy(master, now_ms)) {
    gs_master_latch_fault(master, GS_FAULT_MASTER_LINK_TIMEOUT);
    return;
  }
  if (master->slave_feedback.accepted_sequence !=
          master->last_forwarded_sequence &&
      (uint32_t)(now_ms - master->last_new_esp_sequence_ms) >
          GS_SLAVE_TIMEOUT_MS) {
    gs_master_latch_fault(master, GS_FAULT_MASTER_LINK_TIMEOUT);
  }
}

bool gs_master_make_slave_frame(gs_master_coordinator *master,
                                uint8_t out[GS_SLAVE_COMMAND_SIZE]) {
  if (master == NULL || out == NULL) {
    return false;
  }
  gs_slave_command command = {
      .electrical_command = gs_slave_electrical_command(master->demanded.right),
      .flags = master->slave_flags,
      .sequence = master->last_esp_sequence,
      .enable_epoch = master->last_esp_command.enable_epoch,
      .clear_fault_epoch =
          master->last_esp_command.slave_clear_fault_epoch,
  };
  if (master->state == GS_CONTROLLER_DISABLED ||
      master->state == GS_CONTROLLER_FAULTED ||
      master->state == GS_CONTROLLER_SHUTDOWN) {
    command.electrical_command = 0;
    command.flags |= GS_COMMAND_DISABLE;
  }
  if (!gs_encode_slave_command(out, &command)) {
    return false;
  }
  master->last_forwarded_sequence = command.sequence;
  return true;
}

bool gs_master_accept_slave_feedback(
    gs_master_coordinator *master,
    const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE], uint32_t now_ms) {
  gs_slave_feedback feedback;
  if (master == NULL || !gs_decode_slave_feedback(&feedback, frame)) {
    if (master != NULL) {
      ++master->invalid_slave_feedback_frames;
    }
    return false;
  }

  master->slave_feedback = feedback;
  master->applied.right =
      gs_slave_electrical_command(feedback.applied_electrical);
  master->last_slave_feedback_ms = now_ms;
  master->slave_feedback_seen = true;
  ++master->valid_slave_feedback_frames;

  if (feedback.faults != 0u) {
    gs_master_latch_fault(master, feedback.faults);
    return true;
  }
  if (master->state == GS_CONTROLLER_ACTIVE &&
      (!slave_state_healthy(feedback.state) ||
       feedback.enable_epoch != master->enable_epoch)) {
    gs_master_latch_fault(master, GS_FAULT_MASTER_LINK_TIMEOUT);
    return true;
  }
  if (master->recovery_required && master->enable_epoch_valid &&
      feedback.state == GS_CONTROLLER_READY &&
      feedback.enable_epoch == master->enable_epoch &&
      feedback.accepted_sequence == master->last_forwarded_sequence) {
    master->recovery_required = false;
  }
  return true;
}

bool gs_master_fault_clear_requested(const gs_master_coordinator *master) {
  return master != NULL && master->clear_fault_pending &&
         master->state == GS_CONTROLLER_DISABLED;
}

void gs_master_finish_fault_clear(gs_master_coordinator *master,
                                  gs_clear_result result) {
  if (master == NULL) {
    return;
  }
  if (result == GS_CLEAR_OK) {
    master->faults = 0u;
    master->first_fault = 0u;
    master->recovery_required = true;
    master->enable_epoch_valid = false;
  }
  master->last_clear_result = (uint8_t)result;
  master->clear_fault_pending = false;
  stop_master(master, GS_CONTROLLER_DISABLED);
}

void gs_master_set_runtime_status(gs_master_coordinator *master,
                                  bool pa4_raw_high, bool pa4_bypass) {
  if (master == NULL) {
    return;
  }
  master->runtime_status_flags =
      (uint8_t)((pa4_raw_high ? GS_FEEDBACK_PA4_RAW_HIGH : 0u) |
                (pa4_bypass ? GS_FEEDBACK_PA4_BYPASS : 0u));
}

bool gs_master_make_feedback(const gs_master_coordinator *master,
                             uint8_t out[GS_MASTER_FEEDBACK_SIZE],
                             uint32_t now_ms) {
  if (master == NULL || out == NULL) {
    return false;
  }
  uint8_t flags = master->runtime_status_flags;
  if (gs_master_peer_healthy(master, now_ms)) {
    flags |= GS_FEEDBACK_PEER_HEALTHY;
  }
  if (master->clear_fault_pending) {
    flags |= GS_FEEDBACK_CLEAR_PENDING;
  }
  const gs_master_feedback feedback = {
      .protocol_version = GS_PROTOCOL_VERSION,
      .master_state = (uint8_t)master->state,
      .slave_state = master->slave_feedback.state,
      .status_flags = flags,
      .accepted_esp_sequence = master->esp_seen ? master->last_esp_sequence : 0u,
      .forwarded_slave_sequence = master->last_forwarded_sequence,
      .accepted_slave_sequence = master->slave_feedback.accepted_sequence,
      .master_enable_epoch = master->enable_epoch,
      .slave_enable_epoch = master->slave_feedback.enable_epoch,
      .master_fault_epoch = master->fault_epoch,
      .slave_fault_epoch = master->slave_feedback.fault_epoch,
      .master_clear_result = master->last_clear_result,
      .slave_clear_result = master->slave_feedback.clear_result,
      .left_applied = master->applied.left,
      .right_applied = master->applied.right,
      .left_odometer = master->local_odometer,
      .right_odometer =
          gs_slave_logical_odometer(master->slave_feedback.odometer),
      .master_faults = master->faults,
      .slave_faults = master->slave_feedback.faults,
      .master_first_fault = master->first_fault,
      .slave_first_fault = master->slave_feedback.first_fault,
      .master_command_age_ms =
          master->esp_seen
              ? gs_age_ms_u16(now_ms, master->last_esp_command_ms)
              : UINT16_MAX,
      .slave_feedback_age_ms =
          master->slave_feedback_seen
              ? gs_age_ms_u16(now_ms, master->last_slave_feedback_ms)
              : UINT16_MAX,
      .slave_command_age_ms = master->slave_feedback.command_age_ms,
  };
  return gs_encode_master_feedback(out, &feedback);
}
