/* SPDX-License-Identifier: GPL-3.0-only */
#include "gausstop_board.h"

#include <stddef.h>

#include "gd32f1x0.h"

enum {
  GS_PWM_PERIOD = 999,
  GS_PWM_MIDPOINT = 500,
  GS_PWM_DEADTIME = 120,
  GS_HALL_EVENT_BUFFER_SIZE = 16,
  GS_BOARD_UART_COUNT = 2,
  GS_UART_RX_BUFFER_SIZE = 128,
  GS_UART_TX_BUFFER_SIZE = 128,
};

_Static_assert((GS_HALL_EVENT_BUFFER_SIZE & (GS_HALL_EVENT_BUFFER_SIZE - 1u)) ==
                   0u,
               "Hall event buffer size must be a power of two");
_Static_assert((GS_UART_TX_BUFFER_SIZE & (GS_UART_TX_BUFFER_SIZE - 1u)) == 0u,
               "UART TX buffer size must be a power of two");
_Static_assert((GS_UART_RX_BUFFER_SIZE & (GS_UART_RX_BUFFER_SIZE - 1u)) == 0u,
               "UART RX buffer size must be a power of two");

static const uint16_t pwm_channels[3] = {TIMER_CH_0, TIMER_CH_1, TIMER_CH_2};
static volatile uint32_t micros_high;
static volatile uint32_t milliseconds;
static volatile gs_board_hall_event hall_events[GS_HALL_EVENT_BUFFER_SIZE];
static volatile uint8_t hall_event_head;
static volatile uint8_t hall_event_tail;
static volatile uint32_t hall_event_overflows;
static volatile uint8_t hall_last_state;
static volatile uint32_t hall_last_timestamp_us;
static volatile uint8_t uart_tx_buffers[GS_BOARD_UART_COUNT]
                                       [GS_UART_TX_BUFFER_SIZE];
static volatile uint8_t uart_rx_buffers[GS_BOARD_UART_COUNT]
                                       [GS_UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_rx_heads[GS_BOARD_UART_COUNT];
static volatile uint8_t uart_rx_tails[GS_BOARD_UART_COUNT];
static volatile uint8_t uart_tx_heads[GS_BOARD_UART_COUNT];
static volatile uint8_t uart_tx_tails[GS_BOARD_UART_COUNT];
static volatile gs_board_uart_stats uart_stats[GS_BOARD_UART_COUNT];
static bool bridge_active;
static uint8_t bridge_enabled_phase_mask;

void SysTick_Handler(void) { ++milliseconds; }

void TIMER1_IRQHandler(void) {
  if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) != RESET) {
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    micros_high += 0x10000u;
  }
}

static uint8_t channel_for_phase(gs_phase phase) {
  static const uint8_t phase_to_channel[3] = {2, 0, 1};
  return phase_to_channel[(uint8_t)phase];
}

void gs_board_safe_gpio_init(void) {
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  gpio_bit_reset(GPIOB, GPIO_PIN_2);
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_2);
  gpio_bit_set(GPIOB, GPIO_PIN_2);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
  gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
}

void gs_board_time_init(void) {
  milliseconds = 0u;
  micros_high = 0u;
  (void)SysTick_Config(GS_SYSTEM_CLOCK_HZ / 1000u);
  rcu_periph_clock_enable(RCU_TIMER1);
  timer_deinit(TIMER1);
  timer_parameter_struct timer = {0};
  timer.prescaler = 7u;
  timer.alignedmode = TIMER_COUNTER_EDGE;
  timer.counterdirection = TIMER_COUNTER_UP;
  timer.period = 0xFFFFu;
  timer.clockdivision = TIMER_CKDIV_DIV1;
  timer_init(TIMER1, &timer);
  timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
  timer_interrupt_enable(TIMER1, TIMER_INT_UP);
  nvic_irq_enable(TIMER1_IRQn, 2u, 0u);
  timer_enable(TIMER1);
}

static void bridge_gpio_init(void) {
  gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                          GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
                GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_11);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
  gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_14);
}

static void bridge_timer_init(void) {
  rcu_periph_clock_enable(RCU_TIMER0);
  timer_deinit(TIMER0);
  timer_parameter_struct timer = {0};
  timer.prescaler = 0u;
  timer.alignedmode = TIMER_COUNTER_CENTER_BOTH;
  timer.counterdirection = TIMER_COUNTER_UP;
  timer.period = GS_PWM_PERIOD;
  timer.clockdivision = TIMER_CKDIV_DIV1;
  timer_init(TIMER0, &timer);
  timer_auto_reload_shadow_disable(TIMER0);

  timer_oc_parameter_struct output = {0};
  output.ocpolarity = TIMER_OC_POLARITY_HIGH;
  output.ocnpolarity = TIMER_OCN_POLARITY_LOW;
  output.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
  output.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
  for (uint8_t index = 0u; index < 3u; ++index) {
    timer_channel_output_fast_config(TIMER0, pwm_channels[index],
                                     TIMER_OC_FAST_DISABLE);
    timer_channel_output_config(TIMER0, pwm_channels[index], &output);
    timer_channel_output_mode_config(TIMER0, pwm_channels[index],
                                     TIMER_OC_MODE_PWM1);
    timer_channel_output_pulse_value_config(TIMER0, pwm_channels[index],
                                            GS_PWM_MIDPOINT);
    timer_channel_output_shadow_config(TIMER0, pwm_channels[index],
                                       TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_state_config(TIMER0, pwm_channels[index],
                                      TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, pwm_channels[index],
                                                    TIMER_CCXN_DISABLE);
  }

  timer_break_parameter_struct safety = {0};
  safety.runoffstate = TIMER_ROS_STATE_DISABLE;
  safety.ideloffstate = TIMER_IOS_STATE_DISABLE;
  safety.protectmode = TIMER_CCHP_PROT_OFF;
  safety.deadtime = GS_PWM_DEADTIME;
  safety.breakpolarity = TIMER_BREAK_POLARITY_LOW;
  safety.outputautostate = TIMER_OUTAUTO_DISABLE;
  safety.breakstate = TIMER_BREAK_DISABLE;
  timer_break_config(TIMER0, &safety);
  timer_primary_output_config(TIMER0, DISABLE);
  timer_enable(TIMER0);
}

static void adc_init(void) {
  rcu_periph_clock_enable(RCU_ADC);
  rcu_adc_clock_config(RCU_ADCCK_APB2_DIV2);
  adc_deinit();
  adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
  adc_channel_length_config(ADC_REGULAR_CHANNEL, 1u);
  adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
  adc_external_trigger_source_config(ADC_REGULAR_CHANNEL,
                                     ADC_EXTTRIG_REGULAR_NONE);
  adc_enable();
  adc_calibration_enable();
}

uint8_t gs_board_read_hall(void) {
  return (
      uint8_t)(((gpio_input_bit_get(GPIOC, GPIO_PIN_14) == SET ? 1u : 0u)
                << 2) |
               ((gpio_input_bit_get(GPIOA, GPIO_PIN_0) == SET ? 1u : 0u) << 1) |
               (gpio_input_bit_get(GPIOB, GPIO_PIN_11) == SET ? 1u : 0u));
}

uint32_t gs_board_micros(void) {
  uint32_t high_before;
  uint32_t high_after;
  uint16_t current;
  do {
    high_before = micros_high;
    current = (uint16_t)timer_counter_read(TIMER1);
    high_after = micros_high;
  } while (high_before != high_after);
  if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) != RESET &&
      current < 0x8000u) {
    high_before += 0x10000u;
  }
  return high_before | current;
}

static void push_hall_event(uint8_t hall, uint32_t timestamp_us) {
  if (hall == hall_last_state) {
    return;
  }
  const uint8_t next =
      (uint8_t)((hall_event_head + 1u) & (GS_HALL_EVENT_BUFFER_SIZE - 1u));
  if (next == hall_event_tail) {
    ++hall_event_overflows;
    hall_last_state = hall;
    hall_last_timestamp_us = timestamp_us;
    return;
  }
  hall_events[hall_event_head].hall = hall;
  hall_events[hall_event_head].timestamp_us = timestamp_us;
  hall_events[hall_event_head].interval_us =
      timestamp_us - hall_last_timestamp_us;
  hall_event_head = next;
  hall_last_state = hall;
  hall_last_timestamp_us = timestamp_us;
}

static void capture_hall_edge(void) {
  push_hall_event(gs_board_read_hall(), gs_board_micros());
}

static void hall_capture_init(void) {
  hall_event_head = 0u;
  hall_event_tail = 0u;
  hall_event_overflows = 0u;
  hall_last_state = gs_board_read_hall();
  hall_last_timestamp_us = gs_board_micros();
  rcu_periph_clock_enable(RCU_CFGCMP);
  syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN0);
  syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN11);
  syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN14);
  exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
  exti_init(EXTI_11, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
  exti_init(EXTI_14, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
  exti_interrupt_flag_clear(EXTI_0);
  exti_interrupt_flag_clear(EXTI_11);
  exti_interrupt_flag_clear(EXTI_14);
  nvic_irq_enable(EXTI0_1_IRQn, 0u, 0u);
  nvic_irq_enable(EXTI4_15_IRQn, 0u, 0u);
}

void EXTI0_1_IRQHandler(void) {
  if (exti_interrupt_flag_get(EXTI_0) != RESET) {
    exti_interrupt_flag_clear(EXTI_0);
    capture_hall_edge();
  }
}

void gs_board_hall_exti_service(void) {
  bool edge = false;
  if (exti_interrupt_flag_get(EXTI_11) != RESET) {
    exti_interrupt_flag_clear(EXTI_11);
    edge = true;
  }
  if (exti_interrupt_flag_get(EXTI_14) != RESET) {
    exti_interrupt_flag_clear(EXTI_14);
    edge = true;
  }
  if (edge) {
    capture_hall_edge();
  }
}

#if !defined(GS_REMOTE_TRANSPORT_SWD) || GS_REMOTE_TRANSPORT_SWD != 1
void EXTI4_15_IRQHandler(void) { gs_board_hall_exti_service(); }
#endif

bool gs_board_hall_event_read(gs_board_hall_event *event) {
  if (event == NULL || hall_event_tail == hall_event_head) {
    return false;
  }
  const uint8_t tail = hall_event_tail;
  event->hall = hall_events[tail].hall;
  event->timestamp_us = hall_events[tail].timestamp_us;
  event->interval_us = hall_events[tail].interval_us;
  hall_event_tail = (uint8_t)((tail + 1u) & (GS_HALL_EVENT_BUFFER_SIZE - 1u));
  return true;
}

uint32_t gs_board_hall_overflow_count(void) { return hall_event_overflows; }

void gs_board_operational_init(void) {
  gs_board_safe_gpio_init();
  bridge_gpio_init();
  gs_board_time_init();
  hall_capture_init();
  bridge_timer_init();
  gs_board_bridge_off(NULL);
  adc_init();
}

void gs_board_bridge_off(void *context) {
  (void)context;
  timer_primary_output_config(TIMER0, DISABLE);
  for (uint8_t index = 0u; index < 3u; ++index) {
    timer_channel_output_state_config(TIMER0, pwm_channels[index],
                                      TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, pwm_channels[index],
                                                    TIMER_CCXN_DISABLE);
  }
  bridge_active = false;
  bridge_enabled_phase_mask = 0u;
}

static void write_bridge_plan(const gs_bridge_drive_plan *plan) {
  for (uint8_t phase = 0u; phase < GS_BRIDGE_PHASE_COUNT; ++phase) {
    const uint8_t channel = channel_for_phase((gs_phase)phase);
    timer_channel_output_pulse_value_config(TIMER0, pwm_channels[channel],
                                            plan->compare[phase]);
  }
}

static void start_bridge_plan(const gs_bridge_drive_plan *plan) {
  gs_board_bridge_off(NULL);
  for (uint8_t index = 0u; index < GS_BRIDGE_PHASE_COUNT; ++index) {
    timer_channel_output_shadow_config(TIMER0, pwm_channels[index],
                                       TIMER_OC_SHADOW_DISABLE);
  }
  write_bridge_plan(plan);
  for (uint8_t phase = 0u; phase < GS_BRIDGE_PHASE_COUNT; ++phase) {
    const uint8_t channel = channel_for_phase((gs_phase)phase);
    timer_channel_output_shadow_config(TIMER0, pwm_channels[channel],
                                       TIMER_OC_SHADOW_ENABLE);
    if ((plan->enabled_phase_mask & (uint8_t)(1u << phase)) == 0u) {
      continue;
    }
    timer_channel_output_state_config(TIMER0, pwm_channels[channel],
                                      TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(
        TIMER0, pwm_channels[channel], TIMER_CCXN_ENABLE);
  }
  timer_primary_output_config(TIMER0, ENABLE);
  bridge_enabled_phase_mask = plan->enabled_phase_mask;
  bridge_active = true;
}

bool gs_board_bridge_apply(void *context, const gs_commutation_vector *vector,
                           uint16_t compare_offset) {
  (void)context;
  const gs_motor_power_profile *power = gs_motor_power_profile_current();
  gs_bridge_drive_plan plan;
  if (!gs_bridge_make_drive_plan(vector, GS_PWM_MIDPOINT, compare_offset,
                                 power->maximum_compare, &plan) ||
      !gs_board_shutdown_clear()) {
    gs_board_bridge_off(NULL);
    return false;
  }

  if (!bridge_active || bridge_enabled_phase_mask != plan.enabled_phase_mask) {
    start_bridge_plan(&plan);
    return true;
  }
  timer_update_event_disable(TIMER0);
  write_bridge_plan(&plan);
  timer_update_event_enable(TIMER0);
  return true;
}

gs_bridge_port gs_board_bridge_port(void) {
  const gs_bridge_port port = {NULL, gs_board_bridge_off,
                               gs_board_bridge_apply};
  return port;
}

gs_bridge_profile_id gs_board_bridge_profile_id(void) {
  return gs_bridge_profile_current();
}

bool gs_board_shutdown_raw_high(void) {
  return gpio_input_bit_get(GPIOA, GPIO_PIN_4) == SET;
}

bool gs_board_shutdown_clear(void) {
#if defined(GS_BYPASS_PA4_SHUTDOWN) && GS_BYPASS_PA4_SHUTDOWN == 1
  return true;
#else
  return gs_board_shutdown_raw_high();
#endif
}

bool gs_board_adc_read(uint16_t *out) {
  if (out == NULL) {
    return false;
  }
  gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_6);
  adc_regular_channel_config(0u, 6u, ADC_SAMPLETIME_239POINT5);
  adc_flag_clear(ADC_FLAG_EOC);
  adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
  uint32_t timeout = 100000u;
  while (adc_flag_get(ADC_FLAG_EOC) == RESET && timeout != 0u) {
    --timeout;
  }
  if (timeout == 0u) {
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    return false;
  }
  *out = adc_regular_data_read();
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
  return true;
}

bool gs_board_watchdog_was_reset(void) {
  return rcu_flag_get(RCU_FLAG_FWDGTRST) != RESET;
}

void gs_board_watchdog_start(void) {
  fwdgt_config(1250u, FWDGT_PSC_DIV32);
  fwdgt_enable();
}

void gs_board_watchdog_reload(void) { fwdgt_counter_reload(); }
uint32_t gs_board_millis(void) { return milliseconds; }

static uint8_t uart_index(gs_board_uart uart) { return (uint8_t)uart; }
static uint32_t uart_peripheral(gs_board_uart uart) {
  return uart == GS_UART_REMOTE ? USART0 : USART1;
}

static void reset_uart_state(gs_board_uart uart) {
  const uint8_t index = uart_index(uart);
  uart_rx_heads[index] = 0u;
  uart_rx_tails[index] = 0u;
  uart_tx_heads[index] = 0u;
  uart_tx_tails[index] = 0u;
  uart_stats[index] = (gs_board_uart_stats){0};
}

void gs_board_uart_init(gs_board_uart uart, bool transmit_enabled) {
  const bool remote = uart == GS_UART_REMOTE;
  const uint32_t peripheral = uart_peripheral(uart);
  const uint32_t port = remote ? GPIOB : GPIOA;
  const uint32_t pins =
      remote ? GPIO_PIN_6 | GPIO_PIN_7 : GPIO_PIN_2 | GPIO_PIN_3;
  const uint32_t af = remote ? GPIO_AF_0 : GPIO_AF_1;
  reset_uart_state(uart);
  rcu_periph_clock_enable(remote ? RCU_USART0 : RCU_USART1);
  gpio_af_set(port, af, pins);
  gpio_mode_set(port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, pins);
  gpio_output_options_set(port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, pins);
  usart_deinit(peripheral);
  usart_baudrate_set(peripheral, remote ? 19200u : 115200u);
  usart_word_length_set(peripheral, USART_WL_8BIT);
  usart_stop_bit_set(peripheral, USART_STB_1BIT);
  usart_parity_config(peripheral, USART_PM_NONE);
  usart_receive_config(peripheral, USART_RECEIVE_ENABLE);
  usart_transmit_config(peripheral, transmit_enabled ? USART_TRANSMIT_ENABLE
                                                     : USART_TRANSMIT_DISABLE);
  usart_enable(peripheral);
  USART_CTL0(peripheral) &= ~USART_CTL0_TBEIE;
  USART_CTL0(peripheral) |= USART_CTL0_RBNEIE;
  nvic_irq_enable(remote ? USART0_IRQn : USART1_IRQn, 2u, 0u);
}

bool gs_board_uart_read(gs_board_uart uart, uint8_t *byte) {
  const uint8_t index = uart_index(uart);
  if (byte == NULL) {
    return false;
  }
  const uint8_t tail = uart_rx_tails[index];
  if (tail == uart_rx_heads[index]) {
    return false;
  }
  *byte = uart_rx_buffers[index][tail];
  uart_rx_tails[index] = (uint8_t)((tail + 1u) & (GS_UART_RX_BUFFER_SIZE - 1u));
  return true;
}

static void service_uart_rx(gs_board_uart uart) {
  const uint8_t index = uart_index(uart);
  const uint32_t peripheral = uart_peripheral(uart);
  const uint32_t status = USART_STAT(peripheral);
  uint32_t clear = 0u;
  if ((status & USART_STAT_ORERR) != 0u) {
    ++uart_stats[index].rx_overflows;
    clear |= USART_INTC_OREC;
  }
  if ((status & (USART_STAT_FERR | USART_STAT_NERR | USART_STAT_PERR)) != 0u) {
    ++uart_stats[index].framing_errors;
    clear |= USART_INTC_FEC | USART_INTC_NEC | USART_INTC_PEC;
  }
  if (clear != 0u) {
    USART_INTC(peripheral) = clear;
  }
  if ((status & USART_STAT_RBNE) == 0u) {
    return;
  }
  const uint8_t received = (uint8_t)usart_data_receive(peripheral);
  ++uart_stats[index].rx_bytes;
  const uint8_t head = uart_rx_heads[index];
  const uint8_t next = (uint8_t)((head + 1u) & (GS_UART_RX_BUFFER_SIZE - 1u));
  if (next == uart_rx_tails[index]) {
    ++uart_stats[index].rx_overflows;
    return;
  }
  uart_rx_buffers[index][head] = received;
  uart_rx_heads[index] = next;
}

bool gs_board_uart_write(gs_board_uart uart, const uint8_t *bytes,
                         uint32_t length) {
  const uint8_t index = uart_index(uart);
  if (bytes == NULL) {
    return false;
  }
  if (length == 0u) {
    return true;
  }
  const uint8_t head = uart_tx_heads[index];
  const uint8_t tail = uart_tx_tails[index];
  const uint8_t used = (uint8_t)((head - tail) & (GS_UART_TX_BUFFER_SIZE - 1u));
  const uint32_t available = GS_UART_TX_BUFFER_SIZE - 1u - used;
  if (length > available) {
    ++uart_stats[index].tx_overflows;
    return false;
  }
  uint8_t next = head;
  for (uint32_t offset = 0u; offset < length; ++offset) {
    uart_tx_buffers[index][next] = bytes[offset];
    next = (uint8_t)((next + 1u) & (GS_UART_TX_BUFFER_SIZE - 1u));
  }
  uart_tx_heads[index] = next;
  USART_CTL0(uart_peripheral(uart)) |= USART_CTL0_TBEIE;
  return true;
}

static void service_uart_tx(gs_board_uart uart) {
  const uint8_t index = uart_index(uart);
  const uint32_t peripheral = uart_peripheral(uart);
  if ((USART_STAT(peripheral) & USART_STAT_TBE) == 0u) {
    return;
  }
  const uint8_t tail = uart_tx_tails[index];
  if (tail == uart_tx_heads[index]) {
    USART_CTL0(peripheral) &= ~USART_CTL0_TBEIE;
    return;
  }
  usart_data_transmit(peripheral, uart_tx_buffers[index][tail]);
  uart_tx_tails[index] = (uint8_t)((tail + 1u) & (GS_UART_TX_BUFFER_SIZE - 1u));
  ++uart_stats[index].tx_bytes;
}

void USART0_IRQHandler(void) {
  service_uart_rx(GS_UART_REMOTE);
  service_uart_tx(GS_UART_REMOTE);
}
void USART1_IRQHandler(void) {
  service_uart_rx(GS_UART_LINK);
  service_uart_tx(GS_UART_LINK);
}

void gs_board_uart_get_stats(gs_board_uart uart, gs_board_uart_stats *stats) {
  if (stats == NULL) {
    return;
  }
  const uint8_t index = uart_index(uart);
  stats->rx_bytes = uart_stats[index].rx_bytes;
  stats->tx_bytes = uart_stats[index].tx_bytes;
  stats->rx_overflows = uart_stats[index].rx_overflows;
  stats->tx_overflows = uart_stats[index].tx_overflows;
  stats->framing_errors = uart_stats[index].framing_errors;
}
