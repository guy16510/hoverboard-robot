/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_HALL_QUALIFIER_H
#define GS_HALL_QUALIFIER_H

#include <stdbool.h>
#include <stdint.h>

enum { GS_HALL_STABILITY_US = 50u };

typedef struct {
  uint8_t hall;
  uint32_t timestamp_us;
  uint32_t interval_us;
} gs_qualified_hall_event;

typedef struct {
  uint8_t raw_hall;
  uint8_t qualified_hall;
  uint8_t candidate_hall;
  uint32_t candidate_started_us;
  uint32_t last_qualified_edge_us;
  uint16_t glitch_count;
  bool initialized;
  bool candidate_pending;
} gs_hall_qualifier;

void gs_hall_qualifier_init(gs_hall_qualifier *qualifier, uint8_t raw_hall,
                            uint32_t now_us);
bool gs_hall_qualifier_update(gs_hall_qualifier *qualifier, uint8_t raw_hall,
                              uint32_t now_us, gs_qualified_hall_event *event);
uint8_t gs_hall_qualifier_value(const gs_hall_qualifier *qualifier);
uint16_t gs_hall_qualifier_glitches(const gs_hall_qualifier *qualifier);
bool gs_hall_qualifier_clear_safe(const gs_hall_qualifier *qualifier,
                                  uint8_t raw_hall);

#endif
