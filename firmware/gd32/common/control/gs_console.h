/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_CONSOLE_H
#define GS_CONSOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gs_types.h"

enum {
  GS_CONSOLE_MAX_LINE = 63,
  GS_DEFAULT_RAMP_PER_TICK = 10,
};

typedef enum {
  GS_CONSOLE_REJECTED = 0,
  GS_CONSOLE_APPLIED,
  GS_CONSOLE_STATUS,
  GS_CONSOLE_HELP,
  GS_CONSOLE_CLEAR_FAULT,
} gs_console_result;

typedef struct {
  bool enabled;
  bool shutdown;
  gs_drive_request target;
  gs_wheel_pair current;
  uint16_t ramp_per_tick;
} gs_console_state;

void gs_console_init(gs_console_state *state);
gs_console_result gs_console_execute(gs_console_state *state, const char *line,
                                     size_t length);
void gs_console_ramp_tick(gs_console_state *state);

#endif
