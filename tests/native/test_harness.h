/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_TEST_HARNESS_H
#define GS_TEST_HARNESS_H

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

extern unsigned gs_tests_run;
extern unsigned gs_tests_failed;

#define GS_EXPECT_TRUE(value)                                                  \
  do {                                                                         \
    ++gs_tests_run;                                                            \
    if (!(value)) {                                                            \
      ++gs_tests_failed;                                                       \
      fprintf(stderr, "%s:%d expected true: %s\n", __FILE__, __LINE__,         \
              #value);                                                         \
    }                                                                          \
  } while (0)

#define GS_EXPECT_FALSE(value) GS_EXPECT_TRUE(!(value))

#define GS_EXPECT_EQ(expected, actual)                                         \
  do {                                                                         \
    int64_t gs_expected_ = (int64_t)(expected);                                \
    int64_t gs_actual_ = (int64_t)(actual);                                    \
    ++gs_tests_run;                                                            \
    if (gs_expected_ != gs_actual_) {                                          \
      ++gs_tests_failed;                                                       \
      fprintf(stderr, "%s:%d expected %" PRId64 ", got %" PRId64 "\n",         \
              __FILE__, __LINE__, gs_expected_, gs_actual_);                   \
    }                                                                          \
  } while (0)

#define GS_EXPECT_BYTES(expected, actual, count)                               \
  do {                                                                         \
    ++gs_tests_run;                                                            \
    if (memcmp((expected), (actual), (count)) != 0) {                          \
      ++gs_tests_failed;                                                       \
      fprintf(stderr, "%s:%d byte sequence mismatch\n", __FILE__, __LINE__);   \
    }                                                                          \
  } while (0)

#endif
