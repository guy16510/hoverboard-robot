/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_slave.h"

#include <stddef.h>
#include <string.h>

#include "gs_safety.h"

static void stop_slave(gs_slave_coordinator *slave, gs_controller_state state) {
  slave->demanded_electrical = 0;
  slave->applied_electrical = 0;
  slave->state = state;
}

void gs_slave_init(gs_slave_coordinator *slave, uint32_t now_ms) {
  if (slave == NULL) {
    return;
  }
  memset(slave, 0, sizeof(*slave));
  slave->state = GS_CONTROLLER_DISABLED;
  slave->last_master_command_ms = now_ms;
}

bool gs_slave_accept_master_frame(gs_slave_coordinator *slave,
                                  const uint8_t frame[GS_SLAVE_COMMAND_SIZE],
                                  uint32_t now_ms) {
  gs_slave_command command;
  if (slave == NULL || !gs_decode_slave_command(&command, frame)) {
    if (slave != NULL && (slave->state == GS_CONTROLLER_READY ||
                          slave->state == GS_CONTROLLER_ACTIVE)) {
      slave->faults |= GS_FAULT_PROTOCOL;
      stop_slave(slave, GS_CONTROLLER_FAULTED);
    }
    return false;
  }
  slave->last_master_command_ms = now_ms;
  slave->master_seen = true;
  if ((command.flags & GS_COMMAND_SHUTDOWN) != 0u) {
    slave->shutdown = true;
    stop_slave(slave, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if (slave->shutdown) {
    stop_slave(slave, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if ((command.flags & GS_COMMAND_DISABLE) != 0u) {
    if ((command.flags & GS_COMMAND_CLEAR_FAULT) != 0u &&
        (slave->faults & GS_FAULT_WATCHDOG_LOCKOUT) == 0u) {
      slave->faults = 0u;
    }
    stop_slave(slave, GS_CONTROLLER_DISABLED);
    return true;
  }
  if (slave->faults != 0u) {
    stop_slave(slave, GS_CONTROLLER_FAULTED);
    return true;
  }
  slave->demanded_electrical = command.electrical_command;
  slave->state = command.electrical_command == 0 ? GS_CONTROLLER_READY
                                                 : GS_CONTROLLER_ACTIVE;
  return true;
}

void gs_slave_tick(gs_slave_coordinator *slave, uint32_t now_ms) {
  if (slave == NULL || !slave->master_seen ||
      (slave->state != GS_CONTROLLER_READY &&
       slave->state != GS_CONTROLLER_ACTIVE)) {
    return;
  }
  if ((uint32_t)(now_ms - slave->last_master_command_ms) >
      GS_SLAVE_TIMEOUT_MS) {
    slave->faults |= GS_FAULT_MASTER_LINK_TIMEOUT;
    stop_slave(slave, GS_CONTROLLER_FAULTED);
  }
}

bool gs_slave_make_feedback(const gs_slave_coordinator *slave,
                            uint8_t out[GS_SLAVE_FEEDBACK_SIZE]) {
  if (slave == NULL || out == NULL) {
    return false;
  }
  const gs_slave_feedback feedback = {
      (uint8_t)slave->state,
      slave->odometer,
      slave->faults,
  };
  return gs_encode_slave_feedback(out, &feedback);
}
