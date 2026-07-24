/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GAUSSTOP_BRIDGE_PROFILE_H
#define GAUSSTOP_BRIDGE_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gs_commutation.h"

typedef enum {
  GS_BRIDGE_PROFILE_TWO_LEG_FLOATING = 1,
  GS_BRIDGE_PROFILE_PROVEN_THREE_LEG_MIDPOINT = 2,
} gs_bridge_profile_id;

enum {
  GS_BRIDGE_PHASE_COUNT = 3,
  GS_BRIDGE_ALL_PHASES_MASK = (1u << GS_BRIDGE_PHASE_COUNT) - 1u,
};

typedef struct {
  uint16_t compare[GS_BRIDGE_PHASE_COUNT];
  uint8_t enabled_phase_mask;
} gs_bridge_drive_plan;

#if !defined(GS_BRIDGE_PROFILE_ID)
#error "GS_BRIDGE_PROFILE_ID must select an explicit bridge topology"
#endif

#if GS_BRIDGE_PROFILE_ID != 1 && GS_BRIDGE_PROFILE_ID != 2
#error "Unknown GS_BRIDGE_PROFILE_ID"
#endif

static inline gs_bridge_profile_id gs_bridge_profile_current(void) {
  return (gs_bridge_profile_id)GS_BRIDGE_PROFILE_ID;
}

static inline bool
gs_bridge_make_drive_plan(const gs_commutation_vector *vector,
                          uint16_t midpoint, uint16_t compare_offset,
                          uint16_t maximum_compare,
                          gs_bridge_drive_plan *plan) {
  if (vector == NULL || plan == NULL ||
      (unsigned int)vector->source >= GS_BRIDGE_PHASE_COUNT ||
      (unsigned int)vector->sink >= GS_BRIDGE_PHASE_COUNT ||
      (unsigned int)vector->floating >= GS_BRIDGE_PHASE_COUNT ||
      vector->source == vector->sink || vector->source == vector->floating ||
      vector->sink == vector->floating || compare_offset == 0u ||
      compare_offset > maximum_compare || compare_offset > midpoint ||
      (uint32_t)midpoint + compare_offset > UINT16_MAX) {
    return false;
  }
  for (uint8_t phase = 0u; phase < GS_BRIDGE_PHASE_COUNT; ++phase) {
    plan->compare[phase] = midpoint;
  }
  plan->compare[vector->source] = (uint16_t)(midpoint + compare_offset);
  plan->compare[vector->sink] = (uint16_t)(midpoint - compare_offset);
  plan->enabled_phase_mask =
      gs_bridge_profile_current() == GS_BRIDGE_PROFILE_PROVEN_THREE_LEG_MIDPOINT
          ? GS_BRIDGE_ALL_PHASES_MASK
          : (uint8_t)((1u << vector->source) | (1u << vector->sink));
  return true;
}

#endif
