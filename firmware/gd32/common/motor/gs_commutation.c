/* SPDX-License-Identifier: GPL-3.0-only
 * Modified from the six-step commutation concepts in RoboDurden upstream and
 * the GAUSSTOP-specific Hall order recorded by the legacy project.
 */
#include "gs_commutation.h"

#include <stddef.h>

static const int8_t forward_map[8] = {-1, 3, 1, 2, 5, 4, 0, -1};
static const int8_t reverse_map[8] = {-1, 1, 5, 0, 3, 2, 4, -1};

static const gs_commutation_vector vectors[6] = {
    {GS_PHASE_Y, GS_PHASE_B, GS_PHASE_G}, {GS_PHASE_Y, GS_PHASE_G, GS_PHASE_B},
    {GS_PHASE_B, GS_PHASE_G, GS_PHASE_Y}, {GS_PHASE_B, GS_PHASE_Y, GS_PHASE_G},
    {GS_PHASE_G, GS_PHASE_Y, GS_PHASE_B}, {GS_PHASE_G, GS_PHASE_B, GS_PHASE_Y},
};

bool gs_commutation_for_hall(uint8_t hall, int8_t direction,
                             gs_commutation_vector *out) {
  if (hall > 7u || out == NULL || direction == 0) {
    return false;
  }
  const int8_t index = direction > 0 ? forward_map[hall] : reverse_map[hall];
  if (index < 0) {
    return false;
  }
  *out = vectors[(size_t)index];
  return true;
}

static uint8_t expected_next(uint8_t previous, int8_t direction) {
  static const uint8_t forward_next[8] = {0, 5, 3, 1, 6, 4, 2, 0};
  static const uint8_t reverse_next[8] = {0, 3, 6, 2, 5, 1, 4, 0};
  return direction > 0 ? forward_next[previous] : reverse_next[previous];
}

gs_hall_transition gs_validate_hall_transition(uint8_t previous,
                                               uint8_t current,
                                               int8_t direction,
                                               uint32_t interval_us) {
  if (previous == 0u || previous == 7u || current == 0u || current == 7u ||
      previous > 7u || current > 7u || direction == 0) {
    return GS_HALL_INVALID;
  }
  if (previous == current) {
    return GS_HALL_REPEATED;
  }
  if (current != expected_next(previous, direction)) {
    return GS_HALL_ILLEGAL;
  }
  if (interval_us < 500u) {
    return GS_HALL_TOO_FAST;
  }
  return GS_HALL_LEGAL;
}
