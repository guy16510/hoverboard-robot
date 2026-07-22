/* SPDX-License-Identifier: GPL-3.0-only
 * Modified from peripheral setup concepts in RoboDurden upstream and retained
 * GAUSSTOP pin/timer behavior from the legacy project.
 */
#include "gausstop_board.h"

#include <stddef.h>

#include "gd32f1x0.h"

enum {
  GS_PWM_PERIOD = 999,
  GS_PWM_MIDPOINT = 500,
  GS_PWM_DEADTIME = 120,
};

static const uint16_t pwm_channels[3] = {TIMER_CH_0, TIMER_CH_1, TIMER_CH_2};
static uint16_t micros_previous;
static uint32_t micros_high;
static volatile uint32_t milliseconds;

void SysTick_Handler(void) { ++milliseconds; }

static uint8_t channel_for_phase(gs_phase phase) {
  /* TIMER channels follow legacy electrical order Y, B, G. */
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
  milliseconds = 0;
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
  for (uint8_t index = 0; index < 3u; ++index) {
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

void gs_board_operational_init(void) {
  gs_board_safe_gpio_init();
  bridge_gpio_init();
  gs_board_time_init();
  bridge_timer_init();
  gs_board_bridge_off(NULL);
  adc_init();
}

void gs_board_bridge_off(void *context) {
  (void)context;
  timer_primary_output_config(TIMER0, DISABLE);
  for (uint8_t index = 0; index < 3u; ++index) {
    timer_channel_output_state_config(TIMER0, pwm_channels[index],
                                      TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, pwm_channels[index],
                                                    TIMER_CCXN_DISABLE);
  }
}

bool gs_board_bridge_apply(void *context, const gs_commutation_vector *vector,
                           uint16_t compare_offset) {
  (void)context;
  if (vector == NULL || compare_offset == 0u || compare_offset > 80u ||
      !gs_board_shutdown_clear()) {
    gs_board_bridge_off(NULL);
    return false;
  }
  const uint8_t source = channel_for_phase(vector->source);
  const uint8_t sink = channel_for_phase(vector->sink);
  const uint8_t floating = channel_for_phase(vector->floating);
  timer_primary_output_config(TIMER0, DISABLE);
  for (uint8_t index = 0; index < 3u; ++index) {
    timer_channel_output_state_config(TIMER0, pwm_channels[index],
                                      TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, pwm_channels[index],
                                                    TIMER_CCXN_DISABLE);
  }
  timer_channel_output_pulse_value_config(TIMER0, pwm_channels[source],
                                          GS_PWM_MIDPOINT + compare_offset);
  timer_channel_output_pulse_value_config(TIMER0, pwm_channels[sink],
                                          GS_PWM_MIDPOINT - compare_offset);
  timer_channel_output_pulse_value_config(TIMER0, pwm_channels[floating],
                                          GS_PWM_MIDPOINT);
  timer_channel_output_state_config(TIMER0, pwm_channels[source],
                                    TIMER_CCX_ENABLE);
  timer_channel_complementary_output_state_config(TIMER0, pwm_channels[source],
                                                  TIMER_CCXN_ENABLE);
  timer_channel_output_state_config(TIMER0, pwm_channels[sink],
                                    TIMER_CCX_ENABLE);
  timer_channel_complementary_output_state_config(TIMER0, pwm_channels[sink],
                                                  TIMER_CCXN_ENABLE);
  timer_primary_output_config(TIMER0, ENABLE);
  return true;
}

gs_bridge_port gs_board_bridge_port(void) {
  const gs_bridge_port port = {NULL, gs_board_bridge_off,
                               gs_board_bridge_apply};
  return port;
}

uint8_t gs_board_read_hall(void) {
  return (
      uint8_t)(((gpio_input_bit_get(GPIOC, GPIO_PIN_14) == SET ? 1u : 0u)
                << 2) |
               ((gpio_input_bit_get(GPIOA, GPIO_PIN_0) == SET ? 1u : 0u) << 1) |
               (gpio_input_bit_get(GPIOB, GPIO_PIN_11) == SET ? 1u : 0u));
}

bool gs_board_shutdown_clear(void) {
  return gpio_input_bit_get(GPIOA, GPIO_PIN_4) == SET;
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

uint32_t gs_board_micros(void) {
  const uint16_t current = (uint16_t)timer_counter_read(TIMER1);
  if (current < micros_previous) {
    micros_high += 0x10000u;
  }
  micros_previous = current;
  return micros_high | current;
}

static uint32_t uart_peripheral(gs_board_uart uart) {
  return uart == GS_UART_REMOTE ? USART0 : USART1;
}

void gs_board_uart_init(gs_board_uart uart, bool transmit_enabled) {
  const bool remote = uart == GS_UART_REMOTE;
  const uint32_t peripheral = uart_peripheral(uart);
  const uint32_t port = remote ? GPIOB : GPIOA;
  const uint32_t pins =
      remote ? GPIO_PIN_6 | GPIO_PIN_7 : GPIO_PIN_2 | GPIO_PIN_3;
  const uint32_t af = remote ? GPIO_AF_0 : GPIO_AF_1;
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
}

bool gs_board_uart_read(gs_board_uart uart, uint8_t *byte) {
  const uint32_t peripheral = uart_peripheral(uart);
  if (byte == NULL || usart_flag_get(peripheral, USART_FLAG_RBNE) == RESET) {
    return false;
  }
  *byte = (uint8_t)usart_data_receive(peripheral);
  return true;
}

bool gs_board_uart_write(gs_board_uart uart, const uint8_t *bytes,
                         uint32_t length) {
  const uint32_t peripheral = uart_peripheral(uart);
  if (bytes == NULL) {
    return false;
  }
  for (uint32_t index = 0; index < length; ++index) {
    uint32_t timeout = 100000u;
    while (usart_flag_get(peripheral, USART_FLAG_TBE) == RESET &&
           timeout != 0u) {
      --timeout;
    }
    if (timeout == 0u) {
      return false;
    }
    usart_data_transmit(peripheral, bytes[index]);
  }
  return true;
}
