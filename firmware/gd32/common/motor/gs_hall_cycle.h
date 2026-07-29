/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_HALL_CYCLE_H
#define GS_HALL_CYCLE_H

#include <stdint.h>

#include "gs_commutation.h"

typedef enum {
  GS_HALL_CYCLE_AUTO = 0,
  GS_HALL_CYCLE_COMMAND_ALIGNED = 1,
  GS_HALL_CYCLE_COMMAND_INVERTED = -1,
} gs_hall_cycle_mode;

typedef struct {
  gs_hall_cycle_mode configured_mode;
  int8_t resolved_polarity;
} gs_hall_cycle_tracker;

void gs_hall_cycle_init(gs_hall_cycle_tracker *tracker,
                        gs_hall_cycle_mode mode);
void gs_hall_cycle_reset(gs_hall_cycle_tracker *tracker);
gs_hall_transition gs_hall_cycle_validate(gs_hall_cycle_tracker *tracker,
                                           uint8_t previous, uint8_t current,
                                           int8_t command_direction,
                                           uint32_t interval_us);
gs_hall_cycle_mode
gs_hall_cycle_resolved_mode(const gs_hall_cycle_tracker *tracker);

#endif
