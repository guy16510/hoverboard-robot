/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GAUSSTOP_BRIDGE_PROFILE_H
#define GAUSSTOP_BRIDGE_PROFILE_H

#include <stdint.h>

typedef enum {
  GS_BRIDGE_PROFILE_TWO_LEG_FLOATING = 1,
  GS_BRIDGE_PROFILE_PROVEN_THREE_LEG_MIDPOINT = 2,
} gs_bridge_profile_id;

#if !defined(GS_BRIDGE_PROFILE_ID)
#error "GS_BRIDGE_PROFILE_ID must select an explicit bridge topology"
#endif

#if GS_BRIDGE_PROFILE_ID != 1 && GS_BRIDGE_PROFILE_ID != 2
#error "Unknown GS_BRIDGE_PROFILE_ID"
#endif

static inline gs_bridge_profile_id gs_bridge_profile_current(void) {
  return (gs_bridge_profile_id)GS_BRIDGE_PROFILE_ID;
}

#endif
