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

template <uint8_t HallAPin, uint8_t HallBPin, uint8_t HallCPin>
class HallSensor {
 public:
  HallSensor() = default;
  HallSensor(const HallSensor &) = delete;
  HallSensor &operator=(const HallSensor &) = delete;

  void begin() {
    static_assert(HallAPin <= 39 && HallBPin <= 39 && HallCPin <= 39,
                  "Hall inputs must be valid ESP32 GPIOs");

    // The controller-connected Hall harness is already pulled up to 5 V.
    // External dividers/level shifters reduce those signals for the ESP32, so
    // enabling an internal pull-up here is both unnecessary and misleading.
    pinMode(HallAPin, INPUT);
    pinMode(HallBPin, INPUT);
    pinMode(HallCPin, INPUT);

    instance_ = this;
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
    started_ = true;

    attachInterrupt(digitalPinToInterrupt(HallAPin), isrThunk, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HallBPin), isrThunk, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HallCPin), isrThunk, CHANGE);
  }

  void service(uint32_t nowMs) {
    if (!started_) {
      return;
    }

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
    if (!started_) {
      return value;
    }

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

  bool started() const { return started_; }

 private:
  static bool ARDUINO_ISR_ATTR validState(uint8_t state) {
    return state >= 1 && state <= 6;
  }

  template <uint8_t Pin>
  static uint8_t ARDUINO_ISR_ATTR readPinLevel() {
    if constexpr (Pin < 32) {
      return static_cast<uint8_t>((REG_READ(GPIO_IN_REG) >> Pin) & 1u);
    }
    return static_cast<uint8_t>(
        (REG_READ(GPIO_IN1_REG) >> static_cast<uint8_t>(Pin - 32)) & 1u);
  }

  static uint8_t ARDUINO_ISR_ATTR readState() {
    return static_cast<uint8_t>((readPinLevel<HallAPin>() << 0u) |
                                (readPinLevel<HallBPin>() << 1u) |
                                (readPinLevel<HallCPin>() << 2u));
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

  inline static HallSensor *instance_ = nullptr;
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
  bool started_ = false;
};

using RightHallSensor = HallSensor<board::kRightHallAPin, board::kRightHallBPin,
                                   board::kRightHallCPin>;
using LeftHallSensor = HallSensor<board::kLeftHallAPin, board::kLeftHallBPin,
                                  board::kLeftHallCPin>;

}  // namespace trashbot::sensors
