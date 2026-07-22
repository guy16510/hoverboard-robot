/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_SAFETY_H
#define GS_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

enum {
  GS_PWM_OFFSET_MAX = 80,
  GS_PWM_OFFSET_START = 40,
  GS_COMMAND_DEADBAND = 50,
  GS_ESP_TIMEOUT_MS = 400,
  GS_SLAVE_TIMEOUT_MS = 100,
  GS_STARTUP_TIMEOUT_MS = 700,
  GS_STALL_TIMEOUT_MS = 300,
  GS_MIN_HALL_INTERVAL_US = 500,
  GS_DIRECTION_DWELL_MS = 250,
  GS_CLEAR_SAFE_MS = 250,
  GS_ADC_SAMPLE_PERIOD_MS = 5,
  GS_ADC_CALIBRATION_SAMPLES = 16,
  GS_ADC_CALIBRATION_MAX_SPREAD = 64,
  GS_ADC_LOW_DELTA = 600,
  GS_ADC_HIGH_DELTA = 1400,
};

typedef enum {
  GS_FAULT_NONE = 0,
  GS_FAULT_PROTOCOL = 1u << 0,
  GS_FAULT_COMMAND_TIMEOUT = 1u << 1,
  GS_FAULT_MASTER_LINK_TIMEOUT = 1u << 2,
  GS_FAULT_STARTUP_TIMEOUT = 1u << 3,
  GS_FAULT_STALL = 1u << 4,
  GS_FAULT_HALL_INVALID = 1u << 5,
  GS_FAULT_HALL_SEQUENCE = 1u << 6,
  GS_FAULT_HALL_TOO_FAST = 1u << 7,
  GS_FAULT_SHUTDOWN = 1u << 8,
  GS_FAULT_PROTECTION = 1u << 9,
  GS_FAULT_ADC_CALIBRATION = 1u << 10,
  GS_FAULT_WATCHDOG_LOCKOUT = 1u << 11,
} gs_fault_flag;

typedef struct {
  uint32_t bits;
} gs_fault_set;

typedef struct {
  bool pa4_high;
  bool adc_valid;
  uint16_t adc_value;
  bool hall_valid;
} gs_safety_sample;

typedef enum {
  GS_SAFETY_MASTER = 0,
  GS_SAFETY_SLAVE,
} gs_safety_role;

typedef struct {
  gs_safety_role role;
  gs_fault_set faults;
  uint32_t last_command_ms;
  uint32_t last_hall_ms;
  uint32_t motion_start_ms;
  uint32_t bridge_off_since_ms;
  int32_t adc_total;
  uint16_t adc_min;
  uint16_t adc_max;
  uint16_t adc_baseline;
  uint8_t adc_samples;
  bool adc_ready;
  bool enabled;
  bool demand_active;
  bool hall_seen;
} gs_safety_supervisor;

void gs_safety_init(gs_safety_supervisor *supervisor, gs_safety_role role,
                    bool watchdog_reset, uint32_t now_ms);
void gs_safety_set_enabled(gs_safety_supervisor *supervisor, bool enabled);
void gs_safety_note_command(gs_safety_supervisor *supervisor, uint32_t now_ms);
void gs_safety_note_demand(gs_safety_supervisor *supervisor, bool active,
                           uint32_t now_ms);
void gs_safety_note_hall(gs_safety_supervisor *supervisor, uint32_t now_ms);
void gs_safety_latch(gs_safety_supervisor *supervisor, gs_fault_flag fault);
void gs_safety_sample_adc_off(gs_safety_supervisor *supervisor, uint16_t value);
void gs_safety_evaluate(gs_safety_supervisor *supervisor,
                        const gs_safety_sample *sample, uint32_t now_ms);
bool gs_safety_bridge_allowed(const gs_safety_supervisor *supervisor);
bool gs_safety_clear(gs_safety_supervisor *supervisor,
                     const gs_safety_sample *sample, uint32_t now_ms);

#endif
