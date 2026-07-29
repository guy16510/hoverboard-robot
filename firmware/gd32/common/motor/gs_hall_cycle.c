/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_hall_cycle.h"

#include <stddef.h>

static bool mode_valid(gs_hall_cycle_mode mode) {
  return mode == GS_HALL_CYCLE_AUTO ||
         mode == GS_HALL_CYCLE_COMMAND_ALIGNED ||
         mode == GS_HALL_CYCLE_COMMAND_INVERTED;
}

static int8_t configured_polarity(gs_hall_cycle_mode mode) {
  if (mode == GS_HALL_CYCLE_COMMAND_ALIGNED) {
    return 1;
  }
  if (mode == GS_HALL_CYCLE_COMMAND_INVERTED) {
    return -1;
  }
  return 0;
}

void gs_hall_cycle_init(gs_hall_cycle_tracker *tracker,
                        gs_hall_cycle_mode mode) {
  if (tracker == NULL) {
    return;
  }
  tracker->configured_mode = mode_valid(mode) ? mode : GS_HALL_CYCLE_AUTO;
  tracker->resolved_polarity = configured_polarity(tracker->configured_mode);
}

void gs_hall_cycle_reset(gs_hall_cycle_tracker *tracker) {
  if (tracker == NULL) {
    return;
  }
  tracker->resolved_polarity = configured_polarity(tracker->configured_mode);
}

static gs_hall_transition validate_with_polarity(
    uint8_t previous, uint8_t current, int8_t command_direction,
    int8_t polarity, uint32_t interval_us) {
  return gs_validate_hall_transition(previous, current,
                                     (int8_t)(command_direction * polarity),
                                     interval_us);
}

gs_hall_transition gs_hall_cycle_validate(gs_hall_cycle_tracker *tracker,
                                           uint8_t previous, uint8_t current,
                                           int8_t command_direction,
                                           uint32_t interval_us) {
  if (tracker == NULL) {
    return GS_HALL_INVALID;
  }
  if (tracker->resolved_polarity != 0) {
    return validate_with_polarity(previous, current, command_direction,
                                  tracker->resolved_polarity, interval_us);
  }

  const gs_hall_transition aligned =
      validate_with_polarity(previous, current, command_direction, 1,
                             interval_us);
  if (aligned == GS_HALL_LEGAL) {
    tracker->resolved_polarity = 1;
    return GS_HALL_LEGAL;
  }

  const gs_hall_transition inverted =
      validate_with_polarity(previous, current, command_direction, -1,
                             interval_us);
  if (inverted == GS_HALL_LEGAL) {
    tracker->resolved_polarity = -1;
    return GS_HALL_LEGAL;
  }

  if (aligned == GS_HALL_TOO_FAST || inverted == GS_HALL_TOO_FAST) {
    return GS_HALL_TOO_FAST;
  }
  if (aligned == GS_HALL_INVALID || inverted == GS_HALL_INVALID) {
    return GS_HALL_INVALID;
  }
  if (aligned == GS_HALL_REPEATED || inverted == GS_HALL_REPEATED) {
    return GS_HALL_REPEATED;
  }
  return GS_HALL_ILLEGAL;
}

gs_hall_cycle_mode
gs_hall_cycle_resolved_mode(const gs_hall_cycle_tracker *tracker) {
  if (tracker == NULL || tracker->resolved_polarity == 0) {
    return GS_HALL_CYCLE_AUTO;
  }
  return tracker->resolved_polarity > 0 ? GS_HALL_CYCLE_COMMAND_ALIGNED
                                        : GS_HALL_CYCLE_COMMAND_INVERTED;
}
