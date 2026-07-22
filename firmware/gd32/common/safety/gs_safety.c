/* SPDX-License-Identifier: GPL-3.0-only */
#include "gs_safety.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool elapsed_more_than(uint32_t now, uint32_t then, uint32_t limit) {
  return (uint32_t)(now - then) > limit;
}

void gs_safety_init(gs_safety_supervisor *supervisor, gs_safety_role role,
                    bool watchdog_reset, uint32_t now_ms) {
  if (supervisor == NULL) {
    return;
  }
  memset(supervisor, 0, sizeof(*supervisor));
  supervisor->role = role;
  supervisor->last_command_ms = now_ms;
  supervisor->last_hall_ms = now_ms;
  supervisor->motion_start_ms = now_ms;
  supervisor->bridge_off_since_ms = now_ms;
  supervisor->adc_min = UINT16_MAX;
  if (watchdog_reset) {
    supervisor->faults.bits = GS_FAULT_WATCHDOG_LOCKOUT;
  }
}

void gs_safety_set_enabled(gs_safety_supervisor *supervisor, bool enabled) {
  if (supervisor == NULL) {
    return;
  }
  supervisor->enabled = enabled;
  if (!enabled) {
    supervisor->demand_active = false;
  }
}

void gs_safety_note_command(gs_safety_supervisor *supervisor, uint32_t now_ms) {
  if (supervisor != NULL) {
    supervisor->last_command_ms = now_ms;
  }
}

void gs_safety_note_demand(gs_safety_supervisor *supervisor, bool active,
                           uint32_t now_ms) {
  if (supervisor == NULL) {
    return;
  }
  if (active && !supervisor->demand_active) {
    supervisor->motion_start_ms = now_ms;
    supervisor->hall_seen = false;
  }
  if (!active && supervisor->demand_active) {
    supervisor->bridge_off_since_ms = now_ms;
  }
  supervisor->demand_active = active;
}

void gs_safety_note_hall(gs_safety_supervisor *supervisor, uint32_t now_ms) {
  if (supervisor != NULL) {
    supervisor->last_hall_ms = now_ms;
    supervisor->hall_seen = true;
  }
}

void gs_safety_latch(gs_safety_supervisor *supervisor, gs_fault_flag fault) {
  if (supervisor != NULL) {
    supervisor->faults.bits |= (uint32_t)fault;
  }
}

void gs_safety_sample_adc_off(gs_safety_supervisor *supervisor,
                              uint16_t value) {
  if (supervisor == NULL || supervisor->adc_ready ||
      supervisor->adc_samples >= GS_ADC_CALIBRATION_SAMPLES) {
    return;
  }
  if (value < supervisor->adc_min) {
    supervisor->adc_min = value;
  }
  if (value > supervisor->adc_max) {
    supervisor->adc_max = value;
  }
  supervisor->adc_total += value;
  ++supervisor->adc_samples;
  if (supervisor->adc_samples == GS_ADC_CALIBRATION_SAMPLES) {
    if ((uint16_t)(supervisor->adc_max - supervisor->adc_min) >
        GS_ADC_CALIBRATION_MAX_SPREAD) {
      gs_safety_latch(supervisor, GS_FAULT_ADC_CALIBRATION);
      return;
    }
    supervisor->adc_baseline =
        (uint16_t)(supervisor->adc_total / GS_ADC_CALIBRATION_SAMPLES);
    supervisor->adc_ready = true;
  }
}

static bool adc_in_range(const gs_safety_supervisor *supervisor,
                         uint16_t value) {
  const int32_t measured = value;
  const int32_t baseline = supervisor->adc_baseline;
  return measured >= baseline - GS_ADC_LOW_DELTA &&
         measured <= baseline + GS_ADC_HIGH_DELTA;
}

void gs_safety_evaluate(gs_safety_supervisor *supervisor,
                        const gs_safety_sample *sample, uint32_t now_ms) {
  if (supervisor == NULL || sample == NULL) {
    return;
  }
  if (!sample->pa4_high) {
    gs_safety_latch(supervisor, GS_FAULT_SHUTDOWN);
  }
  if (!sample->hall_valid) {
    gs_safety_latch(supervisor, GS_FAULT_HALL_INVALID);
  }
  if (!sample->adc_valid) {
    gs_safety_latch(supervisor, GS_FAULT_ADC_CALIBRATION);
  } else if (supervisor->adc_ready &&
             !adc_in_range(supervisor, sample->adc_value)) {
    gs_safety_latch(supervisor, GS_FAULT_PROTECTION);
  }
  if (!supervisor->enabled || !supervisor->demand_active) {
    return;
  }
  const uint32_t communication_limit = supervisor->role == GS_SAFETY_MASTER
                                           ? GS_ESP_TIMEOUT_MS
                                           : GS_SLAVE_TIMEOUT_MS;
  if (elapsed_more_than(now_ms, supervisor->last_command_ms,
                        communication_limit)) {
    gs_safety_latch(supervisor, supervisor->role == GS_SAFETY_MASTER
                                    ? GS_FAULT_COMMAND_TIMEOUT
                                    : GS_FAULT_MASTER_LINK_TIMEOUT);
  }
  if (!supervisor->hall_seen &&
      elapsed_more_than(now_ms, supervisor->motion_start_ms,
                        GS_STARTUP_TIMEOUT_MS)) {
    gs_safety_latch(supervisor, GS_FAULT_STARTUP_TIMEOUT);
  }
  if (supervisor->hall_seen &&
      elapsed_more_than(now_ms, supervisor->last_hall_ms,
                        GS_STALL_TIMEOUT_MS)) {
    gs_safety_latch(supervisor, GS_FAULT_STALL);
  }
}

bool gs_safety_bridge_allowed(const gs_safety_supervisor *supervisor) {
  return supervisor != NULL && supervisor->enabled &&
         supervisor->demand_active && supervisor->adc_ready &&
         supervisor->faults.bits == GS_FAULT_NONE;
}

bool gs_safety_clear(gs_safety_supervisor *supervisor,
                     const gs_safety_sample *sample, uint32_t now_ms) {
  if (supervisor == NULL || sample == NULL || supervisor->enabled ||
      supervisor->demand_active || !sample->pa4_high || !sample->adc_valid ||
      !sample->hall_valid || !supervisor->adc_ready ||
      !adc_in_range(supervisor, sample->adc_value) ||
      (supervisor->faults.bits &
       (GS_FAULT_WATCHDOG_LOCKOUT | GS_FAULT_SHUTDOWN)) != 0u ||
      (uint32_t)(now_ms - supervisor->bridge_off_since_ms) < GS_CLEAR_SAFE_MS) {
    return false;
  }
  supervisor->faults.bits = GS_FAULT_NONE;
  supervisor->enabled = false;
  return true;
}
