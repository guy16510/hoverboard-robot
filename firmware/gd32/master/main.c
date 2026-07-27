/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdbool.h>
#include <stdint.h>

#include "gausstop_board.h"
#include "gd32f1x0.h"
#include "gs_frame_parser.h"
#include "gs_master.h"
#include "gs_motor_control.h"
#include "gs_safety.h"
#include "gs_wheel_mix.h"

enum {
  GS_REMOTE_FEEDBACK_FALLBACK_MS = 60,
  GS_REMOTE_FEEDBACK_AFTER_SLAVE_MS = 2,
  GS_REMOTE_FEEDBACK_RETRY_MS = 10,
  GS_REMOTE_FEEDBACK_PERIOD_MS = 100,
};

static gs_master_coordinator master;
static gs_motor_controller motor;
static gs_safety_supervisor safety;
static gs_frame_parser esp_parser;
static gs_frame_parser slave_parser;
static bool feedback_pending;
static uint16_t feedback_sequence;
static uint32_t feedback_due_ms;
static uint32_t next_periodic_feedback_ms;

static void calibrate_protection(void) {
  for (uint8_t sample = 0; sample < GS_ADC_CALIBRATION_SAMPLES; ++sample) {
    const uint32_t wait_until = gs_board_millis() + GS_ADC_SAMPLE_PERIOD_MS;
    while ((int32_t)(gs_board_millis() - wait_until) < 0) {
    }
    uint16_t value = 0;
    if (gs_board_adc_read(&value)) {
      gs_safety_sample_adc_off(&safety, value);
    } else {
      gs_safety_latch(&safety, GS_FAULT_ADC_CALIBRATION);
    }
  }
}

static void force_master_fault(gs_fault_flag fault) {
  gs_safety_latch(&safety, fault);
  master.faults |= (uint32_t)fault;
  master.requested = (gs_wheel_pair){0, 0};
  master.demanded = (gs_wheel_pair){0, 0};
  master.applied = (gs_wheel_pair){0, 0};
  master.state = GS_CONTROLLER_FAULTED;
  gs_motor_force_off(&motor);
}

static void service_esp_rx(void) {
  uint8_t byte = 0;
  uint8_t frame[GS_MAX_FRAME_SIZE];
  while (gs_board_uart_read(GS_UART_REMOTE, &byte)) {
    const uint32_t now_ms = gs_board_millis();
    const gs_parse_result result =
        gs_frame_parser_feed(&esp_parser, byte, now_ms, frame);
    if (result == GS_PARSE_FRAME &&
        gs_master_accept_esp_frame(&master, frame, now_ms)) {
      gs_safety_note_command(&safety, now_ms);
      feedback_pending = true;
      feedback_sequence = master.last_esp_sequence;
      feedback_due_ms = now_ms + GS_REMOTE_FEEDBACK_FALLBACK_MS;
    }
  }
  (void)gs_frame_parser_poll(&esp_parser, gs_board_millis());
}

static void service_slave_rx(void) {
  uint8_t byte = 0;
  uint8_t frame[GS_MAX_FRAME_SIZE];
  while (gs_board_uart_read(GS_UART_LINK, &byte)) {
    const uint32_t now_ms = gs_board_millis();
    const gs_parse_result result =
        gs_frame_parser_feed(&slave_parser, byte, now_ms, frame);
    if (result == GS_PARSE_FRAME &&
        gs_master_accept_slave_feedback(&master, frame, now_ms) &&
        feedback_pending &&
        master.slave_feedback.accepted_sequence == feedback_sequence) {
      const uint32_t acknowledged_due =
          now_ms + GS_REMOTE_FEEDBACK_AFTER_SLAVE_MS;
      if ((int32_t)(acknowledged_due - feedback_due_ms) < 0) {
        feedback_due_ms = acknowledged_due;
      }
    }
  }
  (void)gs_frame_parser_poll(&slave_parser, gs_board_millis());
}

static void service_transport_health(void) {
  static uint32_t remote_rx_overflows;
  static uint32_t remote_tx_overflows;
  static uint32_t link_rx_overflows;
  static uint32_t link_tx_overflows;
  gs_board_uart_stats remote = {0};
  gs_board_uart_stats link = {0};
  gs_board_uart_get_stats(GS_UART_REMOTE, &remote);
  gs_board_uart_get_stats(GS_UART_LINK, &link);
  gs_master_set_remote_diagnostics(&master, remote.rx_bytes,
                                   remote.framing_errors);
  uint8_t overflow_sources = 0u;
  if (remote.rx_overflows != remote_rx_overflows) {
    overflow_sources |= GS_FEEDBACK_TRANSPORT_REMOTE_RX_OVERFLOW;
  }
  if (remote.tx_overflows != remote_tx_overflows) {
    overflow_sources |= GS_FEEDBACK_TRANSPORT_REMOTE_TX_OVERFLOW;
  }
  if (link.rx_overflows != link_rx_overflows) {
    overflow_sources |= GS_FEEDBACK_TRANSPORT_LINK_RX_OVERFLOW;
  }
  if (link.tx_overflows != link_tx_overflows) {
    overflow_sources |= GS_FEEDBACK_TRANSPORT_LINK_TX_OVERFLOW;
  }
  remote_rx_overflows = remote.rx_overflows;
  remote_tx_overflows = remote.tx_overflows;
  link_rx_overflows = link.rx_overflows;
  link_tx_overflows = link.tx_overflows;
  if (overflow_sources != 0u &&
      (master.state == GS_CONTROLLER_READY ||
       master.state == GS_CONTROLLER_ACTIVE)) {
    gs_master_note_transport_overflow(&master, overflow_sources);
    force_master_fault(GS_FAULT_TRANSPORT_OVERFLOW);
  }
}

static bool pa4_bypass_compiled(void) {
#if defined(GS_BYPASS_PA4_SHUTDOWN) && GS_BYPASS_PA4_SHUTDOWN == 1
  return true;
#else
  return false;
#endif
}

static void apply_motor_step(uint8_t hall, bool hall_changed,
                             uint32_t interval_us, bool permitted,
                             uint32_t now_ms) {
  const gs_motor_input input = {hall,      hall_changed,         interval_us,
                                permitted, master.demanded.left, now_ms};
  const gs_motor_output output = gs_motor_step(&motor, &input);
  if (output.faulted) {
    const gs_fault_flag fault =
        output.hall_result == GS_HALL_TOO_FAST
            ? GS_FAULT_HALL_TOO_FAST
            : (output.hall_result == GS_HALL_ILLEGAL ? GS_FAULT_HALL_SEQUENCE
                                                     : GS_FAULT_HALL_INVALID);
    force_master_fault(fault);
  }
  if (output.hall_result == GS_HALL_LEGAL) {
    gs_safety_note_hall(&safety, now_ms);
  }
  master.applied.left = output.demand.logical_command;
  master.local_odometer = motor.odometer;
  gs_master_set_motor_status(&master, hall, output.demand.compare_offset,
                             output.demand.bridge_enabled);
}

static void service_motor(void) {
  static uint32_t last_service_ms;
  static uint32_t last_adc_ms;
  static uint16_t adc_value;
  static bool adc_valid;
  static uint32_t observed_hall_overflows;
  const uint32_t now_ms = gs_board_millis();
  const bool periodic = now_ms != last_service_ms;
  gs_board_hall_event first_event;
  const bool event_available = gs_board_hall_event_read(&first_event);
  if (!periodic && !event_available) {
    return;
  }
  if (periodic) {
    last_service_ms = now_ms;
    if ((uint32_t)(now_ms - last_adc_ms) >= GS_ADC_SAMPLE_PERIOD_MS) {
      adc_valid = gs_board_adc_read(&adc_value);
      last_adc_ms = now_ms;
    }
  }
  const uint32_t hall_overflows = gs_board_hall_overflow_count();
  if (hall_overflows != observed_hall_overflows) {
    observed_hall_overflows = hall_overflows;
    force_master_fault(GS_FAULT_HALL_CAPTURE_OVERFLOW);
  }
  const uint8_t hall = gs_board_read_hall();
  const bool pa4_raw_high = gs_board_shutdown_raw_high();
  gs_master_set_runtime_status(&master, pa4_raw_high, pa4_bypass_compiled());
  gs_safety_set_enabled(&safety, master.state == GS_CONTROLLER_READY ||
                                     master.state == GS_CONTROLLER_ACTIVE);
  gs_safety_note_demand(&safety, gs_motor_bridge_active(&motor), now_ms);
  const gs_safety_sample sample = {gs_board_shutdown_clear(), adc_valid,
                                   adc_value, hall != 0u && hall != 7u};
  if (gs_master_fault_clear_requested(&master) && master.requested.left == 0 &&
      master.requested.right == 0 && master.demanded.left == 0 &&
      master.demanded.right == 0 && master.applied.left == 0 &&
      master.applied.right == 0 && motor.requested_command == 0 &&
      motor.applied_command == 0 && gs_safety_clear(&safety, &sample, now_ms)) {
    gs_motor_clear_fault(&motor, now_ms);
    gs_master_finish_fault_clear(&master, true);
    master.transport_status_flags = 0u;
  }
  gs_safety_evaluate(&safety, &sample, now_ms);
  master.faults |= safety.faults.bits;
  if (safety.faults.bits != 0u) {
    master.requested = (gs_wheel_pair){0, 0};
    master.demanded = (gs_wheel_pair){0, 0};
    master.applied = (gs_wheel_pair){0, 0};
    master.state = GS_CONTROLLER_FAULTED;
  }
  const bool permitted =
      safety.enabled && safety.adc_ready && safety.faults.bits == 0u;
  bool processed_event = false;
  if (event_available) {
    processed_event = true;
    apply_motor_step(first_event.hall, true, first_event.interval_us, permitted,
                     now_ms);
  }
  gs_board_hall_event event;
  while (gs_board_hall_event_read(&event)) {
    processed_event = true;
    apply_motor_step(event.hall, true, event.interval_us, permitted, now_ms);
  }
  if (!processed_event) {
    apply_motor_step(hall, false, 0u, permitted, now_ms);
  }
}

int main(void) {
  const bool watchdog_reset = gs_board_watchdog_was_reset();
  rcu_all_reset_flag_clear();
  gs_board_operational_init();
  gs_master_init(&master, gs_board_millis());
  gs_motor_init(&motor, gs_board_bridge_port(), gs_board_millis());
  gs_safety_init(&safety, GS_SAFETY_MASTER, watchdog_reset, gs_board_millis());
  calibrate_protection();
  const uint8_t command_marker[] = {GS_COMMAND_MARKER};
  const uint8_t slave_feedback_marker[] = {GS_SLAVE_FEEDBACK_MARKER};
  gs_frame_parser_init(&esp_parser, command_marker, 1, GS_ESP_COMMAND_SIZE);
  gs_frame_parser_init(&slave_parser, slave_feedback_marker, 1,
                       GS_SLAVE_FEEDBACK_SIZE);
  gs_board_uart_init(GS_UART_LINK, true);
  gs_board_uart_init(GS_UART_REMOTE, true);
  gs_board_watchdog_start();
  uint32_t next_link_ms = 0;
  next_periodic_feedback_ms = gs_board_millis();
  for (;;) {
    service_esp_rx();
    service_slave_rx();
    service_transport_health();
    gs_master_tick(&master, gs_board_millis());
    service_motor();
    const uint32_t now = gs_board_millis();
    if ((int32_t)(now - next_link_ms) >= 0) {
      uint8_t frame[GS_SLAVE_COMMAND_SIZE];
      next_link_ms = now + 20u;
      if (gs_master_make_slave_frame(&master, frame, now) &&
          !gs_board_uart_write(GS_UART_LINK, frame, sizeof(frame))) {
        force_master_fault(GS_FAULT_TRANSPORT_OVERFLOW);
      }
    }
    const bool acknowledged_feedback_due =
        feedback_pending && (int32_t)(now - feedback_due_ms) >= 0;
    const bool periodic_feedback_due =
        (int32_t)(now - next_periodic_feedback_ms) >= 0;
    if (acknowledged_feedback_due || periodic_feedback_due) {
      uint8_t frame[GS_MASTER_FEEDBACK_SIZE];
      if (!gs_master_make_feedback(&master, frame, now)) {
        feedback_pending = false;
        next_periodic_feedback_ms = now + GS_REMOTE_FEEDBACK_PERIOD_MS;
      } else if (gs_board_uart_write(GS_UART_REMOTE, frame, sizeof(frame))) {
        feedback_pending = false;
        next_periodic_feedback_ms = now + GS_REMOTE_FEEDBACK_PERIOD_MS;
      } else {
        feedback_due_ms = now + GS_REMOTE_FEEDBACK_RETRY_MS;
        next_periodic_feedback_ms = now + GS_REMOTE_FEEDBACK_RETRY_MS;
      }
    }
    gs_board_watchdog_reload();
  }
}
