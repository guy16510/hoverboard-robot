/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_slave.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "gs_safety.h"
#include "gs_wheel_mix.h"

static void stop_slave(gs_slave_coordinator *slave, gs_controller_state state) {
  slave->demanded_electrical = 0;
  slave->applied_electrical = 0;
  slave->state = state;
}

static uint32_t first_fault_bit(uint32_t faults) {
  return faults & (uint32_t)(~faults + 1u);
}

static bool command_equal(const gs_slave_command *first,
                          const gs_slave_command *second) {
  return first->electrical_command == second->electrical_command &&
         first->flags == second->flags && first->sequence == second->sequence &&
         first->enable_epoch == second->enable_epoch &&
         first->clear_fault_epoch == second->clear_fault_epoch;
}

static bool serial_is_newer(uint16_t value, uint16_t previous) {
  const uint16_t delta = (uint16_t)(value - previous);
  return delta != 0u && delta < 0x8000u;
}

void gs_slave_init(gs_slave_coordinator *slave, uint32_t now_ms) {
  if (slave == NULL) {
    return;
  }
  memset(slave, 0, sizeof(*slave));
  slave->state = GS_CONTROLLER_DISABLED;
  slave->last_master_command_ms = now_ms;
  slave->last_clear_result = GS_CLEAR_NONE;
}

void gs_slave_latch_fault(gs_slave_coordinator *slave, uint32_t fault) {
  if (slave == NULL || fault == 0u) {
    return;
  }
  const uint32_t new_faults = fault & ~slave->faults;
  if (new_faults != 0u) {
    if (slave->faults == 0u) {
      slave->first_fault = first_fault_bit(new_faults);
    }
    ++slave->fault_epoch;
    if (slave->fault_epoch == 0u) {
      ++slave->fault_epoch;
    }
  }
  slave->faults |= fault;
  slave->clear_fault_pending = false;
  slave->enable_epoch_valid = false;
  slave->recovery_required = true;
  slave->last_clear_result = GS_CLEAR_NONE;
  stop_slave(slave, GS_CONTROLLER_FAULTED);
}

static bool clear_request_valid(gs_slave_coordinator *slave,
                                const gs_slave_command *command) {
  if ((command->flags & GS_COMMAND_CLEAR_FAULT) == 0u) {
    return true;
  }
  if ((command->flags & GS_COMMAND_DISABLE) == 0u) {
    slave->last_clear_result = GS_CLEAR_REJECT_STATE;
    return false;
  }
  if (command->clear_fault_epoch != slave->fault_epoch) {
    slave->last_clear_result = GS_CLEAR_REJECT_STALE_EPOCH;
    return false;
  }
  if ((slave->faults &
       (GS_FAULT_WATCHDOG_LOCKOUT | GS_FAULT_SHUTDOWN)) != 0u) {
    slave->last_clear_result = GS_CLEAR_REQUIRES_RESET;
    return false;
  }
  return true;
}

static bool session_request_valid(gs_slave_coordinator *slave,
                                  const gs_slave_command *command) {
  if ((command->flags & (GS_COMMAND_DISABLE | GS_COMMAND_SHUTDOWN)) != 0u) {
    return true;
  }
  if (slave->shutdown || slave->faults != 0u || command->enable_epoch == 0u) {
    return false;
  }
  if (command->electrical_command != 0) {
    return slave->enable_epoch_valid && !slave->recovery_required &&
           command->enable_epoch == slave->enable_epoch;
  }
  if (slave->enable_epoch_valid) {
    return command->enable_epoch == slave->enable_epoch;
  }
  return slave->enable_epoch == 0u ||
         serial_is_newer(command->enable_epoch, slave->enable_epoch);
}

bool gs_slave_accept_master_frame(gs_slave_coordinator *slave,
                                  const uint8_t frame[GS_SLAVE_COMMAND_SIZE],
                                  uint32_t now_ms) {
  gs_slave_command command;
  if (slave == NULL || !gs_decode_slave_command(&command, frame)) {
    if (slave != NULL) {
      ++slave->invalid_master_frames;
    }
    return false;
  }
  if (!clear_request_valid(slave, &command) ||
      !session_request_valid(slave, &command)) {
    ++slave->invalid_master_frames;
    return false;
  }

  if (slave->master_seen && command.sequence == slave->last_master_sequence) {
    if (!command_equal(&command, &slave->last_master_command)) {
      ++slave->invalid_master_frames;
      return false;
    }
    slave->last_master_command_ms = now_ms;
    return true;
  }
  if (slave->master_seen &&
      !serial_is_newer(command.sequence, slave->last_master_sequence)) {
    ++slave->invalid_master_frames;
    return false;
  }
  if (slave->master_seen) {
    const uint16_t delta =
        (uint16_t)(command.sequence - slave->last_master_sequence);
    if (delta > 1u) {
      slave->missing_master_sequences += (uint32_t)(delta - 1u);
    }
  }

  slave->last_master_command = command;
  slave->last_master_sequence = command.sequence;
  slave->last_master_command_ms = now_ms;
  slave->master_seen = true;
  ++slave->valid_master_frames;

  if ((command.flags & GS_COMMAND_SHUTDOWN) != 0u) {
    slave->clear_fault_pending = false;
    slave->enable_epoch_valid = false;
    slave->shutdown = true;
    stop_slave(slave, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if ((command.flags & GS_COMMAND_DISABLE) != 0u) {
    slave->enable_epoch_valid = false;
    if ((command.flags & GS_COMMAND_CLEAR_FAULT) != 0u) {
      slave->clear_fault_pending = true;
      slave->last_clear_result = GS_CLEAR_NONE;
    }
    stop_slave(slave, GS_CONTROLLER_DISABLED);
    return true;
  }

  slave->clear_fault_pending = false;
  if (!slave->enable_epoch_valid) {
    slave->enable_epoch = command.enable_epoch;
    slave->enable_epoch_valid = true;
    if (command.electrical_command == 0) {
      slave->recovery_required = false;
    }
  }
  slave->demanded_electrical =
      gs_normalize_wheel_command(command.electrical_command);
  slave->state = slave->demanded_electrical == 0 ? GS_CONTROLLER_READY
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
    gs_slave_latch_fault(slave, GS_FAULT_MASTER_LINK_TIMEOUT);
  }
}

bool gs_slave_fault_clear_requested(const gs_slave_coordinator *slave) {
  return slave != NULL && slave->clear_fault_pending &&
         slave->state == GS_CONTROLLER_DISABLED;
}

void gs_slave_finish_fault_clear(gs_slave_coordinator *slave,
                                 gs_clear_result result) {
  if (slave == NULL) {
    return;
  }
  if (result == GS_CLEAR_OK) {
    slave->faults = 0u;
    slave->first_fault = 0u;
    slave->recovery_required = true;
    slave->enable_epoch_valid = false;
  }
  slave->last_clear_result = (uint8_t)result;
  slave->clear_fault_pending = false;
  stop_slave(slave, GS_CONTROLLER_DISABLED);
}

bool gs_slave_make_feedback(const gs_slave_coordinator *slave,
                            uint8_t out[GS_SLAVE_FEEDBACK_SIZE],
                            uint32_t now_ms) {
  if (slave == NULL || out == NULL) {
    return false;
  }
  const gs_slave_feedback feedback = {
      .state = (uint8_t)slave->state,
      .odometer = slave->odometer,
      .faults = slave->faults,
      .first_fault = slave->first_fault,
      .applied_electrical = slave->applied_electrical,
      .accepted_sequence =
          slave->master_seen ? slave->last_master_sequence : 0u,
      .enable_epoch = slave->enable_epoch,
      .fault_epoch = slave->fault_epoch,
      .command_age_ms =
          slave->master_seen
              ? gs_age_ms_u16(now_ms, slave->last_master_command_ms)
              : UINT16_MAX,
      .clear_result = slave->last_clear_result,
  };
  return gs_encode_slave_feedback(out, &feedback);
}
