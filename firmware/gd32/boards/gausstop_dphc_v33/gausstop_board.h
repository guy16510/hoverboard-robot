/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Chris Burns
 * GAUSSTOP-specific mapping retained from legacy physical evidence.
 */
#ifndef GAUSSTOP_BOARD_H
#define GAUSSTOP_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "gausstop_bridge_profile.h"
#include "gs_motor_control.h"

#if !defined(GD32F130) || GD32F130 != 1
#error "GAUSSTOP DPHC-V3.3 requires GD32F130"
#endif
#if !defined(TARGET) || TARGET != 1
#error "GAUSSTOP DPHC-V3.3 requires upstream Target 1"
#endif
#if !defined(GS_SYSTEM_CLOCK_HZ) || GS_SYSTEM_CLOCK_HZ != 8000000UL
#error "GAUSSTOP firmware requires the 8 MHz internal IRC target"
#endif
#if !defined(GS_FLASH_BYTES) || GS_FLASH_BYTES != 65536UL
#error "GAUSSTOP GD32F130C8T6 requires exactly 64 KiB flash"
#endif
#if !defined(GS_RAM_BYTES) || GS_RAM_BYTES != 8192UL
#error "GAUSSTOP GD32F130C8T6 requires exactly 8 KiB SRAM"
#endif

#define GS_PORT_A 0u
#define GS_PORT_B 1u
#define GS_PORT_C 2u
#define GS_PIN(port, bit) (((port) << 5) | (bit))

enum {
  GS_PWM_HIGH_G = GS_PIN(GS_PORT_A, 8),
  GS_PWM_HIGH_Y = GS_PIN(GS_PORT_A, 9),
  GS_PWM_HIGH_B = GS_PIN(GS_PORT_A, 10),
  GS_PWM_LOW_G = GS_PIN(GS_PORT_B, 13),
  GS_PWM_LOW_Y = GS_PIN(GS_PORT_B, 14),
  GS_PWM_LOW_B = GS_PIN(GS_PORT_B, 15),
  GS_HALL_A = GS_PIN(GS_PORT_B, 11),
  GS_HALL_B = GS_PIN(GS_PORT_A, 0),
  GS_HALL_C = GS_PIN(GS_PORT_C, 14),
  GS_REQUIRED_LATCH = GS_PIN(GS_PORT_B, 2),
  GS_SHUTDOWN_INPUT = GS_PIN(GS_PORT_A, 4),
  GS_PROTECTION_ADC = GS_PIN(GS_PORT_A, 6),
  GS_BREAK_CANDIDATE = GS_PIN(GS_PORT_B, 12),
  GS_ESP_TX = GS_PIN(GS_PORT_B, 6),
  GS_ESP_RX = GS_PIN(GS_PORT_B, 7),
  GS_LINK_TX = GS_PIN(GS_PORT_A, 2),
  GS_LINK_RX = GS_PIN(GS_PORT_A, 3),
};

#ifndef GS_PB12_BREAK_ENABLED
#define GS_PB12_BREAK_ENABLED 0
#endif

#define GS_DISTINCT(a, b)                                                      \
  _Static_assert((a) != (b), "duplicate safety-critical pin")
#define GS_ASSERT_NOT_BRIDGE(pin)                                              \
  GS_DISTINCT((pin), GS_PWM_HIGH_G);                                           \
  GS_DISTINCT((pin), GS_PWM_HIGH_Y);                                           \
  GS_DISTINCT((pin), GS_PWM_HIGH_B);                                           \
  GS_DISTINCT((pin), GS_PWM_LOW_G);                                            \
  GS_DISTINCT((pin), GS_PWM_LOW_Y);                                            \
  GS_DISTINCT((pin), GS_PWM_LOW_B)

GS_DISTINCT(GS_PWM_HIGH_G, GS_PWM_HIGH_Y);
GS_DISTINCT(GS_PWM_HIGH_G, GS_PWM_HIGH_B);
GS_DISTINCT(GS_PWM_HIGH_G, GS_PWM_LOW_G);
GS_DISTINCT(GS_PWM_HIGH_G, GS_PWM_LOW_Y);
GS_DISTINCT(GS_PWM_HIGH_G, GS_PWM_LOW_B);
GS_DISTINCT(GS_PWM_HIGH_Y, GS_PWM_HIGH_B);
GS_DISTINCT(GS_PWM_HIGH_Y, GS_PWM_LOW_G);
GS_DISTINCT(GS_PWM_HIGH_Y, GS_PWM_LOW_Y);
GS_DISTINCT(GS_PWM_HIGH_Y, GS_PWM_LOW_B);
GS_DISTINCT(GS_PWM_HIGH_B, GS_PWM_LOW_G);
GS_DISTINCT(GS_PWM_HIGH_B, GS_PWM_LOW_Y);
GS_DISTINCT(GS_PWM_HIGH_B, GS_PWM_LOW_B);
GS_DISTINCT(GS_PWM_LOW_G, GS_PWM_LOW_Y);
GS_DISTINCT(GS_PWM_LOW_G, GS_PWM_LOW_B);
GS_DISTINCT(GS_PWM_LOW_Y, GS_PWM_LOW_B);
GS_ASSERT_NOT_BRIDGE(GS_HALL_A);
GS_ASSERT_NOT_BRIDGE(GS_HALL_B);
GS_ASSERT_NOT_BRIDGE(GS_HALL_C);
GS_ASSERT_NOT_BRIDGE(GS_SHUTDOWN_INPUT);
GS_ASSERT_NOT_BRIDGE(GS_PROTECTION_ADC);
GS_ASSERT_NOT_BRIDGE(GS_REQUIRED_LATCH);
_Static_assert(GS_PB12_BREAK_ENABLED == 0,
               "PB12 hardware break is not validated");
_Static_assert(GS_HALL_A == GS_PIN(GS_PORT_B, 11), "HALL_A must be PB11");
_Static_assert(GS_HALL_B == GS_PIN(GS_PORT_A, 0), "HALL_B must be PA0");
_Static_assert(GS_HALL_C == GS_PIN(GS_PORT_C, 14), "HALL_C must be PC14");
_Static_assert(GS_REQUIRED_LATCH == GS_PIN(GS_PORT_B, 2), "latch must be PB2");
_Static_assert(GS_SHUTDOWN_INPUT == GS_PIN(GS_PORT_A, 4),
               "shutdown must be PA4");
_Static_assert(GS_PROTECTION_ADC == GS_PIN(GS_PORT_A, 6),
               "protection ADC must be PA6");

typedef enum {
  GS_UART_REMOTE = 0,
  GS_UART_LINK,
} gs_board_uart;

typedef struct {
  uint8_t hall;
  uint32_t timestamp_us;
  uint32_t interval_us;
} gs_board_hall_event;

typedef struct {
  uint32_t rx_bytes;
  uint32_t tx_bytes;
  uint32_t rx_overflows;
  uint32_t tx_overflows;
  uint32_t framing_errors;
} gs_board_uart_stats;

void gs_board_safe_gpio_init(void);
void gs_board_time_init(void);
void gs_board_operational_init(void);
void gs_board_bridge_off(void *context);
bool gs_board_bridge_apply(void *context, const gs_commutation_vector *vector,
                           uint16_t compare_offset);
gs_bridge_port gs_board_bridge_port(void);
gs_bridge_profile_id gs_board_bridge_profile_id(void);
uint8_t gs_board_read_hall(void);
void gs_board_hall_exti_service(void);
bool gs_board_hall_event_read(gs_board_hall_event *event);
uint32_t gs_board_hall_overflow_count(void);
bool gs_board_shutdown_clear(void);
bool gs_board_adc_read(uint16_t *out);
bool gs_board_watchdog_was_reset(void);
void gs_board_watchdog_start(void);
void gs_board_watchdog_reload(void);
uint32_t gs_board_millis(void);
uint32_t gs_board_micros(void);
void gs_board_uart_init(gs_board_uart uart, bool transmit_enabled);
bool gs_board_uart_read(gs_board_uart uart, uint8_t *byte);
bool gs_board_uart_write(gs_board_uart uart, const uint8_t *bytes,
                         uint32_t length);
void gs_board_uart_get_stats(gs_board_uart uart, gs_board_uart_stats *stats);

#endif
