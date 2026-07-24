/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cmath>
#include <cstdio>

extern unsigned gs_esp32_tests_failed;
extern unsigned gs_esp32_tests_run;

#define GS_ESP32_EXPECT_TRUE(expression)                                       \
  do {                                                                         \
    ++gs_esp32_tests_run;                                                      \
    if (!(expression)) {                                                       \
      ++gs_esp32_tests_failed;                                                 \
      std::fprintf(stderr, "%s:%d expected true: %s\n", __FILE__, __LINE__,    \
                   #expression);                                               \
    }                                                                          \
  } while (false)

#define GS_ESP32_EXPECT_FALSE(expression) GS_ESP32_EXPECT_TRUE(!(expression))

#define GS_ESP32_EXPECT_EQ(expected, actual)                                   \
  do {                                                                         \
    ++gs_esp32_tests_run;                                                      \
    const auto gs_expected = (expected);                                       \
    const auto gs_actual = (actual);                                           \
    if (gs_expected != gs_actual) {                                            \
      ++gs_esp32_tests_failed;                                                 \
      std::fprintf(stderr, "%s:%d expected %lld, got %lld\n", __FILE__,        \
                   __LINE__, static_cast<long long>(gs_expected),              \
                   static_cast<long long>(gs_actual));                         \
    }                                                                          \
  } while (false)

#define GS_ESP32_EXPECT_NEAR(expected, actual, tolerance)                      \
  do {                                                                         \
    ++gs_esp32_tests_run;                                                      \
    const double gs_expected = static_cast<double>(expected);                  \
    const double gs_actual = static_cast<double>(actual);                      \
    if (std::fabs(gs_expected - gs_actual) > static_cast<double>(tolerance)) { \
      ++gs_esp32_tests_failed;                                                 \
      std::fprintf(stderr, "%s:%d expected %.6f +/- %.6f, got %.6f\n",         \
                   __FILE__, __LINE__, gs_expected,                            \
                   static_cast<double>(tolerance), gs_actual);                 \
    }                                                                          \
  } while (false)
