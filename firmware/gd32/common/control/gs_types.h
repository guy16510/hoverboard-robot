/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_TYPES_H
#define GS_TYPES_H

#include <stdbool.h>
#include <stdint.h>

enum { GS_COMMAND_DEADBAND = 50 };

typedef struct {
  int16_t left;
  int16_t right;
} gs_wheel_pair;

typedef enum {
  GS_DRIVE_MIXED = 0,
  GS_DRIVE_DIRECT_LR,
} gs_drive_mode;

typedef struct {
  gs_drive_mode mode;
  int16_t first;
  int16_t second;
} gs_drive_request;

typedef struct {
  int16_t logical_command;
  uint16_t compare_offset;
  bool bridge_enabled;
} gs_motor_demand;

typedef enum {
  GS_RUNTIME_BRIDGE_ENABLED = 1u << 0,
  GS_RUNTIME_PA4_RAW_HIGH = 1u << 1,
  GS_RUNTIME_PA4_BYPASS = 1u << 2,
  GS_RUNTIME_ADC_VALID = 1u << 3,
  GS_RUNTIME_ADC_READY = 1u << 4,
  GS_RUNTIME_ADC_IN_RANGE = 1u << 5,
  GS_RUNTIME_HALL_VALID = 1u << 6,
} gs_runtime_flag;

typedef struct {
  uint32_t rx_bytes;
  uint32_t tx_bytes;
  uint32_t rx_overflows;
  uint32_t tx_overflows;
  uint32_t framing_errors;
} gs_transport_telemetry;

typedef struct {
  int16_t requested_command;
  int16_t demanded_command;
  int16_t applied_command;
  uint16_t compare_offset;
  uint32_t hall_interval_us;
  uint32_t hall_capture_overflows;
  uint16_t adc_value;
  uint16_t adc_baseline;
  uint8_t motor_state;
  uint8_t flags;
  uint8_t hall;
  uint8_t previous_hall;
  uint8_t adc_samples;
} gs_runtime_telemetry;

#endif
