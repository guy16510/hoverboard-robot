/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_SLAVE_H
#define GS_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

#include "gs_master.h"
#include "gs_protocol.h"

typedef struct {
  gs_controller_state state;
  int16_t demanded_electrical;
  int16_t applied_electrical;
  int32_t odometer;
  gs_slave_command last_master_command;
  uint32_t faults;
  uint32_t first_fault;
  uint32_t last_master_command_ms;
  uint32_t valid_master_frames;
  uint32_t invalid_master_frames;
  uint32_t missing_master_sequences;
  uint16_t last_master_sequence;
  uint16_t enable_epoch;
  uint16_t fault_epoch;
  bool master_seen;
  bool shutdown;
  bool clear_fault_pending;
  bool enable_epoch_valid;
  bool recovery_required;
  uint8_t last_clear_result;
} gs_slave_coordinator;

void gs_slave_init(gs_slave_coordinator *slave, uint32_t now_ms);
void gs_slave_latch_fault(gs_slave_coordinator *slave, uint32_t fault);
bool gs_slave_accept_master_frame(gs_slave_coordinator *slave,
                                  const uint8_t frame[GS_SLAVE_COMMAND_SIZE],
                                  uint32_t now_ms);
void gs_slave_tick(gs_slave_coordinator *slave, uint32_t now_ms);
bool gs_slave_fault_clear_requested(const gs_slave_coordinator *slave);
void gs_slave_finish_fault_clear(gs_slave_coordinator *slave,
                                 gs_clear_result result);
bool gs_slave_make_feedback(const gs_slave_coordinator *slave,
                            uint8_t out[GS_SLAVE_FEEDBACK_SIZE],
                            uint32_t now_ms);

#endif
