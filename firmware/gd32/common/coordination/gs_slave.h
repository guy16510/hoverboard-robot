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
  uint32_t faults;
  uint32_t last_master_command_ms;
  bool master_seen;
  bool shutdown;
} gs_slave_coordinator;

void gs_slave_init(gs_slave_coordinator *slave, uint32_t now_ms);
bool gs_slave_accept_master_frame(gs_slave_coordinator *slave,
                                  const uint8_t frame[GS_SLAVE_COMMAND_SIZE],
                                  uint32_t now_ms);
void gs_slave_tick(gs_slave_coordinator *slave, uint32_t now_ms);
bool gs_slave_make_feedback(const gs_slave_coordinator *slave,
                            uint8_t out[GS_SLAVE_FEEDBACK_SIZE]);

#endif
