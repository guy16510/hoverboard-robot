#pragma once

#include <Arduino.h>
#include <cstdint>

#include "board_config.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

namespace trashbot::sensors {

struct HallSnapshot {
  uint8_t state = 0;
  bool valid = false;
  bool moving = false;
  uint32_t transitions = 0;
  uint32_t invalidStates = 0;
  uint32_t skippedTransitions = 0;
  uint32_t rateMilliHz = 0;
  uint32_t lastTransitionAgeMs = UINT32_MAX;
};

class RightHallSensor {
 public:
  RightHallSensor() = default;
  RightHallSensor(const RightHallSensor &) = delete;
  RightHallSensor &operator=(const RightHallSensor &) = delete;

  void begin() {
    static_assert(board::kRightHallAPin < 32 &&
                      board::kRightHallBPin < 32 &&
                      board::kRightHallCPin < 32,
                  "direct Hall ISR register read requires GPIO pins below 32");

    pinMode(board::kRightHallAPin, INPUT_PULLUP);
    pinMode(board::kRightHallBPin, INPUT_PULLUP);
    pinMode(board::kRightHallCPin, INPUT_PULLUP);

    portENTER_CRITICAL(&mux_);
    state_ = readState();
    transitions_ = 0;
    invalidStates_ = validState(state_) ? 0 : 1;
    skippedTransitions_ = 0;
    portEXIT_CRITICAL(&mux_);

    observedTransitions_ = 0;
    rateWindowTransitions_ = 0;
    rateWindowStartedMs_ = millis();
    lastTransitionMs_ = 0;
    rateMilliHz_ = 0;
    instance_ = this;

    attachInterrupt(digitalPinToInterrupt(board::kRightHallAPin), isrThunk,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(board::kRightHallBPin), isrThunk,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(board::kRightHallCPin), isrThunk,
                    CHANGE);
  }

  void service(uint32_t nowMs) {
    uint32_t transitions = 0;
    portENTER_CRITICAL(&mux_);
    transitions = transitions_;
    portEXIT_CRITICAL(&mux_);

    if (transitions != observedTransitions_) {
      observedTransitions_ = transitions;
      lastTransitionMs_ = nowMs;
    }

    const uint32_t elapsedMs = nowMs - rateWindowStartedMs_;
    if (elapsedMs < board::kHallRateWindowMs) {
      return;
    }

    const uint32_t delta = transitions - rateWindowTransitions_;
    rateMilliHz_ =
        elapsedMs == 0
            ? 0
            : static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000000u) /
                                    elapsedMs);
    rateWindowTransitions_ = transitions;
    rateWindowStartedMs_ = nowMs;
  }

  HallSnapshot snapshot(uint32_t nowMs) const {
    HallSnapshot value;
    portENTER_CRITICAL(&mux_);
    value.state = state_;
    value.transitions = transitions_;
    value.invalidStates = invalidStates_;
    value.skippedTransitions = skippedTransitions_;
    portEXIT_CRITICAL(&mux_);

    value.valid = validState(value.state);
    value.rateMilliHz = rateMilliHz_;
    if (lastTransitionMs_ != 0) {
      value.lastTransitionAgeMs = nowMs - lastTransitionMs_;
      value.moving = value.lastTransitionAgeMs <= board::kHallMovingWindowMs;
    }
    return value;
  }

 private:
  static bool ARDUINO_ISR_ATTR validState(uint8_t state) {
    return state >= 1 && state <= 6;
  }

  static uint8_t ARDUINO_ISR_ATTR readState() {
    const uint32_t levels = REG_READ(GPIO_IN_REG);
    return static_cast<uint8_t>(
        (((levels >> board::kRightHallAPin) & 1u) << 0u) |
        (((levels >> board::kRightHallBPin) & 1u) << 1u) |
        (((levels >> board::kRightHallCPin) & 1u) << 2u));
  }

  static void ARDUINO_ISR_ATTR isrThunk() {
    if (instance_ != nullptr) {
      instance_->captureEdge();
    }
  }

  void ARDUINO_ISR_ATTR captureEdge() {
    const uint8_t next = readState();

    portENTER_CRITICAL_ISR(&mux_);
    const uint8_t previous = state_;
    if (next == previous) {
      portEXIT_CRITICAL_ISR(&mux_);
      return;
    }
    state_ = next;

    if (!validState(next)) {
      ++invalidStates_;
      portEXIT_CRITICAL_ISR(&mux_);
      return;
    }

    if (validState(previous)) {
      const uint8_t changed = static_cast<uint8_t>(previous ^ next);
      if ((changed & static_cast<uint8_t>(changed - 1u)) != 0u) {
        ++skippedTransitions_;
      }
      ++transitions_;
    }
    portEXIT_CRITICAL_ISR(&mux_);
  }

  inline static RightHallSensor *instance_ = nullptr;
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint8_t state_ = 0;
  volatile uint32_t transitions_ = 0;
  volatile uint32_t invalidStates_ = 0;
  volatile uint32_t skippedTransitions_ = 0;
  uint32_t observedTransitions_ = 0;
  uint32_t rateWindowTransitions_ = 0;
  uint32_t rateWindowStartedMs_ = 0;
  uint32_t lastTransitionMs_ = 0;
  uint32_t rateMilliHz_ = 0;
};

}  // namespace trashbot::sensors
