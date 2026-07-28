/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_COMMUTATION_H
#define GS_COMMUTATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  GS_PHASE_G = 0,
  GS_PHASE_Y = 1,
  GS_PHASE_B = 2,
} gs_phase;

typedef enum {
  GS_COMMUTATION_PHASE_ADVANCED_REVERSE = 1,
  GS_COMMUTATION_SYMMETRIC_REVERSE = 2,
} gs_commutation_profile;

typedef struct {
  gs_phase source;
  gs_phase sink;
  gs_phase floating;
} gs_commutation_vector;

typedef enum {
  GS_HALL_INVALID = 0,
  GS_HALL_REPEATED,
  GS_HALL_LEGAL,
  GS_HALL_ILLEGAL,
  GS_HALL_TOO_FAST,
} gs_hall_transition;

bool gs_commutation_for_hall(uint8_t hall, int8_t direction,
                             gs_commutation_vector *out);
bool gs_commutation_for_hall_profile(uint8_t hall, int8_t direction,
                                     gs_commutation_profile profile,
                                     gs_commutation_vector *out);
gs_hall_transition gs_validate_hall_transition(uint8_t previous,
                                               uint8_t current,
                                               int8_t direction,
                                               uint32_t interval_us);

#endif
