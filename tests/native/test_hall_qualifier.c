/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "gs_hall_qualifier.h"
#include "gs_motor_control.h"

#include <limits.h>

static const uint8_t forward_cycle[] = {2u, 3u, 1u, 5u, 4u, 6u};
static const uint8_t reverse_cycle[] = {2u, 6u, 4u, 5u, 1u, 3u};

static void expect_short_pulses_rejected_from_every_valid_state(void) {
  for (uint8_t qualified = 1u; qualified <= 6u; ++qualified) {
    for (uint32_t width_us = 0u; width_us < GS_HALL_STABILITY_US; ++width_us) {
      gs_hall_qualifier qualifier;
      gs_hall_qualifier_init(&qualifier, qualified, 1000u);
      const uint8_t candidate = qualified == 6u ? 1u : qualified + 1u;
      gs_qualified_hall_event event = {0};

      GS_EXPECT_FALSE(
          gs_hall_qualifier_update(&qualifier, candidate, 1100u, &event));
      GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, qualified,
                                               1100u + width_us, &event));
      GS_EXPECT_EQ(qualified, gs_hall_qualifier_value(&qualifier));
      GS_EXPECT_EQ(1u, gs_hall_qualifier_glitches(&qualifier));
      GS_EXPECT_TRUE(gs_hall_qualifier_clear_safe(&qualifier, qualified));
    }
  }
}

static void expect_cycle_released_once(const uint8_t *cycle,
                                       size_t cycle_length) {
  for (size_t index = 0u; index < cycle_length; ++index) {
    const uint8_t previous = cycle[index];
    const uint8_t next = cycle[(index + 1u) % cycle_length];
    gs_hall_qualifier qualifier;
    gs_hall_qualifier_init(&qualifier, previous, 1000u);
    gs_qualified_hall_event event = {0};

    GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, next, 1500u, &event));
    GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, next, 1549u, &event));
    GS_EXPECT_TRUE(gs_hall_qualifier_update(&qualifier, next, 1550u, &event));
    GS_EXPECT_EQ(next, event.hall);
    GS_EXPECT_EQ(1500u, event.timestamp_us);
    GS_EXPECT_EQ(500u, event.interval_us);
    GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, next, 1600u, &event));
    GS_EXPECT_EQ(0u, gs_hall_qualifier_glitches(&qualifier));
  }
}

static void test_original_edge_timestamp_preserves_too_fast_check(void) {
  gs_hall_qualifier qualifier;
  gs_hall_qualifier_init(&qualifier, 2u, 1000u);
  gs_qualified_hall_event event = {0};

  GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, 3u, 1499u, &event));
  GS_EXPECT_TRUE(gs_hall_qualifier_update(&qualifier, 3u, 1549u, &event));
  GS_EXPECT_EQ(499u, event.interval_us);
  GS_EXPECT_EQ(GS_HALL_TOO_FAST, gs_validate_hall_transition(
                                     2u, event.hall, 1, event.interval_us));

  gs_hall_qualifier_init(&qualifier, 2u, 1000u);
  GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, 3u, 1500u, &event));
  GS_EXPECT_TRUE(gs_hall_qualifier_update(&qualifier, 3u, 1550u, &event));
  GS_EXPECT_EQ(500u, event.interval_us);
  GS_EXPECT_EQ(GS_HALL_LEGAL, gs_validate_hall_transition(2u, event.hall, 1,
                                                          event.interval_us));
}

static void test_invalid_and_skipped_states_are_released_for_faulting(void) {
  const uint8_t candidates[] = {0u, 7u, 4u};
  for (size_t index = 0u; index < sizeof(candidates); ++index) {
    gs_hall_qualifier qualifier;
    gs_hall_qualifier_init(&qualifier, 2u, 100u);
    gs_qualified_hall_event event = {0};
    GS_EXPECT_FALSE(
        gs_hall_qualifier_update(&qualifier, candidates[index], 700u, &event));
    GS_EXPECT_TRUE(
        gs_hall_qualifier_update(&qualifier, candidates[index], 750u, &event));
    GS_EXPECT_EQ(candidates[index], event.hall);
    if (candidates[index] == 0u || candidates[index] == 7u) {
      GS_EXPECT_FALSE(
          gs_hall_qualifier_clear_safe(&qualifier, candidates[index]));
    }
  }
}

static void test_pending_state_blocks_clear_and_wraparound_is_safe(void) {
  gs_hall_qualifier qualifier;
  gs_hall_qualifier_init(&qualifier, 6u, UINT32_MAX - 25u);
  gs_qualified_hall_event event = {0};
  GS_EXPECT_FALSE(
      gs_hall_qualifier_update(&qualifier, 2u, UINT32_MAX - 5u, &event));
  GS_EXPECT_FALSE(gs_hall_qualifier_clear_safe(&qualifier, 2u));
  GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, 2u, 43u, &event));
  GS_EXPECT_TRUE(gs_hall_qualifier_update(&qualifier, 2u, 44u, &event));
  GS_EXPECT_EQ(20u, event.interval_us);
  GS_EXPECT_TRUE(gs_hall_qualifier_clear_safe(&qualifier, 2u));
}

static void test_glitch_counter_saturates(void) {
  gs_hall_qualifier qualifier;
  gs_hall_qualifier_init(&qualifier, 2u, 0u);
  qualifier.glitch_count = UINT16_MAX;
  gs_qualified_hall_event event = {0};
  GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, 3u, 100u, &event));
  GS_EXPECT_FALSE(gs_hall_qualifier_update(&qualifier, 2u, 101u, &event));
  GS_EXPECT_EQ(UINT16_MAX, gs_hall_qualifier_glitches(&qualifier));
}

void gs_test_hall_qualifier(void) {
  expect_short_pulses_rejected_from_every_valid_state();
  expect_cycle_released_once(forward_cycle,
                             sizeof(forward_cycle) / sizeof(forward_cycle[0]));
  expect_cycle_released_once(reverse_cycle,
                             sizeof(reverse_cycle) / sizeof(reverse_cycle[0]));
  test_original_edge_timestamp_preserves_too_fast_check();
  test_invalid_and_skipped_states_are_released_for_faulting();
  test_pending_state_blocks_clear_and_wraparound_is_safe();
  test_glitch_counter_saturates();
}
