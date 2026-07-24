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

static bool command_equal(const gs_esp_command *first,
                          const gs_esp_command *second) {
  return first->speed == second->speed && first->steer == second->steer &&
         first->master_flags == second->master_flags &&
         first->slave_flags == second->slave_flags &&
         first->sequence == second->sequence;
}

static bool sequence_is_newer(uint16_t sequence, uint16_t previous) {
  const uint16_t delta = (uint16_t)(sequence - previous);
  return delta != 0u && delta < 0x8000u;
}

static bool safe_resync_command(const gs_esp_command *command) {
  return command != NULL && command->speed == 0 && command->steer == 0 &&
         (command->master_flags & GS_COMMAND_DISABLE) != 0u &&
         (command->slave_flags & GS_COMMAND_DISABLE) != 0u;
}

static bool safe_resync_allowed(const gs_master_coordinator *master,
                                const gs_esp_command *command,
                                uint32_t now_ms) {
  if (master == NULL || master->shutdown ||
      !safe_resync_command(command)) {
    return false;
  }
  return master->state == GS_CONTROLLER_DISABLED ||
         master->state == GS_CONTROLLER_FAULTED ||
         (uint32_t)(now_ms - master->last_esp_command_ms) > GS_ESP_TIMEOUT_MS;
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
  master->last_slave_feedback_ms = now_ms;
  master->slave_flags = GS_COMMAND_DISABLE;
}

bool gs_master_peer_healthy(const gs_master_coordinator *master,
                            uint32_t now_ms) {
  return master != NULL && master->slave_feedback_seen &&
         (uint32_t)(now_ms - master->last_slave_feedback_ms) <=
             GS_SLAVE_TIMEOUT_MS &&
         slave_state_healthy(master->slave_feedback.state) &&
         master->slave_feedback.faults == 0u;
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

  const bool resync = safe_resync_allowed(master, &command, now_ms);
  if (master->esp_seen && command.sequence == master->last_esp_sequence) {
    if (command_equal(&command, &master->last_esp_command)) {
      master->last_esp_command_ms = now_ms;
      return true;
    }
    if (!resync) {
      ++master->invalid_esp_frames;
      return false;
    }
  }

  if (master->esp_seen && !resync &&
      !sequence_is_newer(command.sequence, master->last_esp_sequence)) {
    ++master->invalid_esp_frames;
    return false;
  }

  const gs_wheel_pair requested =
      (command.master_flags & GS_COMMAND_DIRECT_LR) != 0u
          ? gs_direct_wheels(command.speed, command.steer)
          : gs_mix_wheels(command.speed, command.steer);
  if ((requested.left != 0 || requested.right != 0) &&
      !gs_master_peer_healthy(master, now_ms)) {
    ++master->invalid_esp_frames;
    return false;
  }

  if (master->esp_seen && !resync) {
    const uint16_t delta =
        (uint16_t)(command.sequence - master->last_esp_sequence);
    if (delta > 1u) {
      master->missing_esp_sequences += (uint32_t)(delta - 1u);
    }
  }

  master->last_esp_command = command;
  master->last_esp_sequence = command.sequence;
  master->last_esp_command_ms = now_ms;
  master->esp_seen = true;
  ++master->valid_esp_frames;
  master->slave_flags = command.slave_flags;

  if ((command.master_flags & GS_COMMAND_SHUTDOWN) != 0u) {
    master->clear_fault_pending = false;
    master->shutdown = true;
    master->slave_flags |= GS_COMMAND_SHUTDOWN | GS_COMMAND_DISABLE;
    stop_master(master, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if (master->shutdown) {
    stop_master(master, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if ((command.master_flags & GS_COMMAND_DISABLE) != 0u) {
    if ((command.master_flags & GS_COMMAND_CLEAR_FAULT) != 0u) {
      master->clear_fault_pending = true;
    }
    master->slave_flags |= GS_COMMAND_DISABLE;
    stop_master(master, GS_CONTROLLER_DISABLED);
    return true;
  }

  master->clear_fault_pending = false;
  if (master->faults != 0u) {
    stop_master(master, GS_CONTROLLER_FAULTED);
    return true;
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
    master->faults |= GS_FAULT_COMMAND_TIMEOUT;
    stop_master(master, GS_CONTROLLER_FAULTED);
    return;
  }
  if (master->state != GS_CONTROLLER_ACTIVE) {
    return;
  }
  if (!gs_master_peer_healthy(master, now_ms)) {
    master->faults |= GS_FAULT_MASTER_LINK_TIMEOUT;
    stop_master(master, GS_CONTROLLER_FAULTED);
    return;
  }
  if (master->slave_feedback.accepted_sequence !=
          master->last_forwarded_sequence &&
      (uint32_t)(now_ms - master->last_forwarded_sequence_ms) >
          GS_SLAVE_TIMEOUT_MS) {
    master->faults |= GS_FAULT_MASTER_LINK_TIMEOUT;
    stop_master(master, GS_CONTROLLER_FAULTED);
  }
}

bool gs_master_make_slave_frame(gs_master_coordinator *master,
                                uint8_t out[GS_SLAVE_COMMAND_SIZE],
                                uint32_t now_ms) {
  if (master == NULL || out == NULL) {
    return false;
  }
  gs_slave_command command = {
      .electrical_command = gs_slave_electrical_command(master->demanded.right),
      .flags = master->slave_flags,
      .sequence = master->last_esp_sequence,
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
  if (command.sequence != master->last_forwarded_sequence) {
    master->last_forwarded_sequence_ms = now_ms;
  }
  master->last_forwarded_sequence = command.sequence;
  return true;
}

bool gs_master_accept_slave_feedback(
    gs_master_coordinator *master, const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE],
    uint32_t now_ms) {
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

  if (master->state == GS_CONTROLLER_ACTIVE &&
      (!slave_state_healthy(feedback.state) || feedback.faults != 0u)) {
    master->faults |=
        feedback.faults != 0u ? feedback.faults : GS_FAULT_MASTER_LINK_TIMEOUT;
    stop_master(master, GS_CONTROLLER_FAULTED);
  }
  return true;
}

bool gs_master_fault_clear_requested(const gs_master_coordinator *master) {
  return master != NULL && master->clear_fault_pending &&
         master->state == GS_CONTROLLER_DISABLED;
}

void gs_master_finish_fault_clear(gs_master_coordinator *master, bool success) {
  if (master == NULL) {
    return;
  }
  if (success) {
    master->faults = 0u;
  }
  master->clear_fault_pending = false;
  master->state = GS_CONTROLLER_DISABLED;
  master->requested = (gs_wheel_pair){0, 0};
  master->demanded = (gs_wheel_pair){0, 0};
  master->applied = (gs_wheel_pair){0, 0};
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

void gs_master_note_transport_overflow(gs_master_coordinator *master,
                                       uint8_t sources) {
  if (master == NULL) {
    return;
  }
  const uint8_t transport_mask = GS_FEEDBACK_TRANSPORT_REMOTE_RX_OVERFLOW |
                                 GS_FEEDBACK_TRANSPORT_REMOTE_TX_OVERFLOW |
                                 GS_FEEDBACK_TRANSPORT_LINK_RX_OVERFLOW |
                                 GS_FEEDBACK_TRANSPORT_LINK_TX_OVERFLOW;
  master->transport_status_flags |= (uint8_t)(sources & transport_mask);
}

void gs_master_set_remote_diagnostics(gs_master_coordinator *master,
                                      uint32_t rx_bytes,
                                      uint32_t framing_errors) {
  if (master == NULL) {
    return;
  }
  master->remote_rx_bytes = (uint16_t)rx_bytes;
  master->remote_framing_errors = (uint16_t)framing_errors;
}

void gs_master_set_motor_status(gs_master_coordinator *master, uint8_t hall,
                                uint16_t compare_offset, bool bridge_enabled) {
  if (master == NULL) {
    return;
  }
  master->local_hall = hall;
  master->local_compare_offset = compare_offset;
  master->local_bridge_enabled = bridge_enabled;
}

bool gs_master_make_feedback(const gs_master_coordinator *master,
                             uint8_t out[GS_MASTER_FEEDBACK_SIZE],
                             uint32_t now_ms) {
  if (master == NULL || out == NULL) {
    return false;
  }
  uint8_t flags =
      (uint8_t)(master->runtime_status_flags | master->transport_status_flags);
  if (gs_master_peer_healthy(master, now_ms)) {
    flags |= GS_FEEDBACK_PEER_HEALTHY;
  }
  if (master->clear_fault_pending || (master->slave_feedback.status_flags &
                                      GS_MOTOR_FEEDBACK_CLEAR_PENDING) != 0u) {
    flags |= GS_FEEDBACK_CLEAR_PENDING;
  }
  const gs_master_feedback feedback = {
      .protocol_version = GS_PROTOCOL_VERSION,
      .master_state = (uint8_t)master->state,
      .slave_state = master->slave_feedback.state,
      .status_flags = flags,
      .accepted_esp_sequence =
          master->esp_seen ? master->last_esp_sequence : 0u,
      .forwarded_slave_sequence = master->last_forwarded_sequence,
      .accepted_slave_sequence = master->slave_feedback.accepted_sequence,
      .left_applied = master->applied.left,
      .right_applied = master->applied.right,
      .left_odometer = master->local_odometer,
      .right_odometer =
          gs_slave_logical_odometer(master->slave_feedback.odometer),
      .master_faults = master->faults,
      .slave_faults = master->slave_feedback.faults,
      .master_command_age_ms =
          master->esp_seen ? gs_age_ms_u16(now_ms, master->last_esp_command_ms)
                           : UINT16_MAX,
      .slave_feedback_age_ms =
          master->slave_feedback_seen
              ? gs_age_ms_u16(now_ms, master->last_slave_feedback_ms)
              : UINT16_MAX,
      .slave_command_age_ms = master->slave_feedback.command_age_ms,
      .left_hall = master->local_hall,
      .right_hall = master->slave_feedback.hall,
      .left_compare_offset = master->local_compare_offset,
      .right_compare_offset = master->slave_feedback.compare_offset,
      .motor_status_flags =
          (uint8_t)((master->local_bridge_enabled
                         ? GS_MASTER_MOTOR_LEFT_BRIDGE_ENABLED
                         : 0u) |
                    ((master->slave_feedback.status_flags &
                      GS_MOTOR_FEEDBACK_BRIDGE_ENABLED) != 0u
                         ? GS_MASTER_MOTOR_RIGHT_BRIDGE_ENABLED
                         : 0u) |
                    ((master->slave_feedback.status_flags &
                      GS_MOTOR_FEEDBACK_PA4_RAW_HIGH) != 0u
                         ? GS_MASTER_MOTOR_SLAVE_PA4_RAW_HIGH
                         : 0u)),
      .remote_rx_bytes = master->remote_rx_bytes,
      .remote_valid_frames = (uint16_t)master->valid_esp_frames,
      .remote_invalid_frames = (uint16_t)master->invalid_esp_frames,
      .remote_framing_errors = master->remote_framing_errors,
  };
  return gs_encode_master_feedback(out, &feedback);
}
