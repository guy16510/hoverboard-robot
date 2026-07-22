/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_master.h"

#include <stddef.h>
#include <string.h>

#include "gs_safety.h"
#include "gs_wheel_mix.h"

static void stop_master(gs_master_coordinator *master,
                        gs_controller_state state) {
  master->demanded = (gs_wheel_pair){0, 0};
  master->applied = (gs_wheel_pair){0, 0};
  master->state = state;
  master->slave_flags |= GS_COMMAND_DISABLE;
}

void gs_master_init(gs_master_coordinator *master, uint32_t now_ms) {
  if (master == NULL) {
    return;
  }
  memset(master, 0, sizeof(*master));
  master->state = GS_CONTROLLER_DISABLED;
  master->last_esp_command_ms = now_ms;
  master->slave_flags = GS_COMMAND_DISABLE;
}

bool gs_master_accept_esp_frame(gs_master_coordinator *master,
                                const uint8_t frame[GS_ESP_COMMAND_SIZE],
                                uint32_t now_ms) {
  gs_esp_command command;
  if (master == NULL || !gs_decode_esp_command(&command, frame)) {
    if (master != NULL && master->state != GS_CONTROLLER_DISABLED) {
      master->faults |= GS_FAULT_PROTOCOL;
      stop_master(master, GS_CONTROLLER_FAULTED);
    }
    return false;
  }
  master->last_esp_command_ms = now_ms;
  master->esp_seen = true;
  master->slave_flags = command.slave_flags;

  if ((command.master_flags & GS_COMMAND_SHUTDOWN) != 0u) {
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
    if ((command.master_flags & GS_COMMAND_CLEAR_FAULT) != 0u &&
        (master->faults & GS_FAULT_WATCHDOG_LOCKOUT) == 0u) {
      master->faults = 0u;
    }
    master->slave_flags |= GS_COMMAND_DISABLE;
    stop_master(master, GS_CONTROLLER_DISABLED);
    return true;
  }
  if (master->faults != 0u) {
    stop_master(master, GS_CONTROLLER_FAULTED);
    return true;
  }

  master->demanded = (command.master_flags & GS_COMMAND_DIRECT_LR) != 0u
                         ? gs_direct_wheels(command.speed, command.steer)
                         : gs_mix_wheels(command.speed, command.steer);
  master->state = master->demanded.left == 0 && master->demanded.right == 0
                      ? GS_CONTROLLER_READY
                      : GS_CONTROLLER_ACTIVE;
  return true;
}

void gs_master_tick(gs_master_coordinator *master, uint32_t now_ms) {
  if (master == NULL || !master->esp_seen ||
      master->state == GS_CONTROLLER_DISABLED ||
      master->state == GS_CONTROLLER_SHUTDOWN) {
    return;
  }
  if ((uint32_t)(now_ms - master->last_esp_command_ms) > GS_ESP_TIMEOUT_MS) {
    master->faults |= GS_FAULT_COMMAND_TIMEOUT;
    stop_master(master, GS_CONTROLLER_FAULTED);
  }
}

bool gs_master_make_slave_frame(const gs_master_coordinator *master,
                                uint8_t out[GS_SLAVE_COMMAND_SIZE]) {
  if (master == NULL || out == NULL) {
    return false;
  }
  gs_slave_command command = {
      gs_slave_electrical_command(master->demanded.right), master->slave_flags};
  if (master->state == GS_CONTROLLER_DISABLED ||
      master->state == GS_CONTROLLER_FAULTED ||
      master->state == GS_CONTROLLER_SHUTDOWN) {
    command.electrical_command = 0;
    command.flags |= GS_COMMAND_DISABLE;
  }
  return gs_encode_slave_command(out, &command);
}

bool gs_master_accept_slave_feedback(
    gs_master_coordinator *master,
    const uint8_t frame[GS_SLAVE_FEEDBACK_SIZE]) {
  if (master == NULL ||
      !gs_decode_slave_feedback(&master->slave_feedback, frame)) {
    if (master != NULL && master->state == GS_CONTROLLER_ACTIVE) {
      master->faults |= GS_FAULT_PROTOCOL;
      stop_master(master, GS_CONTROLLER_FAULTED);
    }
    return false;
  }
  return true;
}

bool gs_master_make_feedback(const gs_master_coordinator *master,
                             uint8_t out[GS_MASTER_FEEDBACK_SIZE]) {
  if (master == NULL || out == NULL) {
    return false;
  }
  const gs_master_feedback feedback = {
      (uint8_t)master->state,
      master->slave_feedback.state,
      master->applied.left,
      master->applied.right,
      master->local_odometer,
      gs_slave_logical_odometer(master->slave_feedback.odometer),
      master->faults,
      master->slave_feedback.faults,
  };
  return gs_encode_master_feedback(out, &feedback);
}
