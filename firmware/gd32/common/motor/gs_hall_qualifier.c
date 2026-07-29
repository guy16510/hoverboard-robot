/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_hall_qualifier.h"

#include <limits.h>
#include <stddef.h>

static bool hall_valid(uint8_t hall) { return hall >= 1u && hall <= 6u; }

static void count_glitch(gs_hall_qualifier *qualifier) {
  if (qualifier->glitch_count != UINT16_MAX) {
    ++qualifier->glitch_count;
  }
}

static bool release_candidate(gs_hall_qualifier *qualifier,
                              gs_qualified_hall_event *event) {
  if (event == NULL) {
    return false;
  }
  event->hall = qualifier->candidate_hall;
  event->timestamp_us = qualifier->candidate_started_us;
  event->interval_us =
      qualifier->candidate_started_us - qualifier->last_qualified_edge_us;
  qualifier->qualified_hall = qualifier->candidate_hall;
  qualifier->last_qualified_edge_us = qualifier->candidate_started_us;
  qualifier->candidate_pending = false;
  return true;
}

void gs_hall_qualifier_init(gs_hall_qualifier *qualifier, uint8_t raw_hall,
                            uint32_t now_us) {
  if (qualifier == NULL) {
    return;
  }
  *qualifier = (gs_hall_qualifier){0};
  qualifier->raw_hall = raw_hall;
  qualifier->qualified_hall = raw_hall;
  qualifier->last_qualified_edge_us = now_us;
  qualifier->initialized = true;
}

bool gs_hall_qualifier_update(gs_hall_qualifier *qualifier, uint8_t raw_hall,
                              uint32_t now_us, gs_qualified_hall_event *event) {
  if (qualifier == NULL) {
    return false;
  }
  if (!qualifier->initialized) {
    gs_hall_qualifier_init(qualifier, raw_hall, now_us);
    return false;
  }

  if (raw_hall != qualifier->raw_hall) {
    if (qualifier->candidate_pending &&
        (uint32_t)(now_us - qualifier->candidate_started_us) >=
            GS_HALL_STABILITY_US) {
      const bool released = release_candidate(qualifier, event);
      qualifier->raw_hall = raw_hall;
      if (raw_hall != qualifier->qualified_hall) {
        qualifier->candidate_hall = raw_hall;
        qualifier->candidate_started_us = now_us;
        qualifier->candidate_pending = true;
      }
      return released;
    }
    if (qualifier->candidate_pending) {
      count_glitch(qualifier);
      qualifier->candidate_pending = false;
    }
    qualifier->raw_hall = raw_hall;
    if (raw_hall != qualifier->qualified_hall) {
      qualifier->candidate_hall = raw_hall;
      qualifier->candidate_started_us = now_us;
      qualifier->candidate_pending = true;
    }
    return false;
  }

  if (!qualifier->candidate_pending ||
      (uint32_t)(now_us - qualifier->candidate_started_us) <
          GS_HALL_STABILITY_US) {
    return false;
  }
  return release_candidate(qualifier, event);
}

uint8_t gs_hall_qualifier_value(const gs_hall_qualifier *qualifier) {
  return qualifier != NULL && qualifier->initialized ? qualifier->qualified_hall
                                                     : 0u;
}

uint16_t gs_hall_qualifier_glitches(const gs_hall_qualifier *qualifier) {
  return qualifier == NULL ? 0u : qualifier->glitch_count;
}

bool gs_hall_qualifier_clear_safe(const gs_hall_qualifier *qualifier,
                                  uint8_t raw_hall) {
  return qualifier != NULL && qualifier->initialized &&
         !qualifier->candidate_pending && hall_valid(raw_hall) &&
         raw_hall == qualifier->raw_hall &&
         raw_hall == qualifier->qualified_hall;
}
