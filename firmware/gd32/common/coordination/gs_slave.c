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

static bool command_equal(const gs_slave_command *first,
                          const gs_slave_command *second) {
  return first->electrical_command == second->electrical_command &&
         first->flags == second->flags && first->sequence == second->sequence;
}

static bool sequence_is_newer(uint16_t sequence, uint16_t previous) {
  const uint16_t delta = (uint16_t)(sequence - previous);
  return delta != 0u && delta < 0x8000u;
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
    if (slave != NULL) {
      ++slave->invalid_master_frames;
    }
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
      !sequence_is_newer(command.sequence, slave->last_master_sequence)) {
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
    slave->shutdown = true;
    stop_slave(slave, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if (slave->shutdown) {
    stop_slave(slave, GS_CONTROLLER_SHUTDOWN);
    return true;
  }
  if ((command.flags & GS_COMMAND_DISABLE) != 0u) {
    if ((command.flags & GS_COMMAND_CLEAR_FAULT) != 0u) {
      slave->clear_fault_pending = true;
    }
    stop_slave(slave, GS_CONTROLLER_DISABLED);
    return true;
  }

  slave->clear_fault_pending = false;
  if (slave->faults != 0u) {
    stop_slave(slave, GS_CONTROLLER_FAULTED);
    return true;
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
    slave->faults |= GS_FAULT_MASTER_LINK_TIMEOUT;
    stop_slave(slave, GS_CONTROLLER_FAULTED);
  }
}

bool gs_slave_fault_clear_requested(const gs_slave_coordinator *slave) {
  return slave != NULL && slave->clear_fault_pending &&
         slave->state == GS_CONTROLLER_DISABLED;
}

void gs_slave_finish_fault_clear(gs_slave_coordinator *slave, bool success) {
  if (slave == NULL) {
    return;
  }
  if (success) {
    slave->faults = 0u;
  }
  slave->clear_fault_pending = false;
  stop_slave(slave, GS_CONTROLLER_DISABLED);
}

void gs_slave_set_motor_status(gs_slave_coordinator *slave, uint8_t hall,
                               uint16_t compare_offset, bool bridge_enabled,
                               bool pa4_raw_high) {
  if (slave == NULL) {
    return;
  }
  slave->hall = hall;
  slave->compare_offset = compare_offset;
  slave->bridge_enabled = bridge_enabled;
  slave->pa4_raw_high = pa4_raw_high;
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
      .applied_electrical = slave->applied_electrical,
      .accepted_sequence =
          slave->master_seen ? slave->last_master_sequence : 0u,
      .command_age_ms =
          slave->master_seen
              ? gs_age_ms_u16(now_ms, slave->last_master_command_ms)
              : UINT16_MAX,
      .hall = slave->hall,
      .status_flags =
          (uint8_t)((slave->bridge_enabled ? GS_MOTOR_FEEDBACK_BRIDGE_ENABLED
                                           : 0u) |
                    (slave->pa4_raw_high ? GS_MOTOR_FEEDBACK_PA4_RAW_HIGH
                                         : 0u) |
                    (slave->clear_fault_pending
                         ? GS_MOTOR_FEEDBACK_CLEAR_PENDING
                         : 0u)),
      .compare_offset = slave->compare_offset,
  };
  return gs_encode_slave_feedback(out, &feedback);
}
