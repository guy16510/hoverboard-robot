/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "balance_controller.h"

#include <cstddef>
#include <cstdint>

namespace gs::balance {

enum class BalanceConfigKey : uint8_t {
  kInnerProportional = 0u,
  kInnerIntegral,
  kInnerDerivative,
  kInnerIntegralLimit,
  kOuterProportional,
  kOuterIntegral,
  kOuterDerivative,
  kOuterIntegralLimit,
  kDerivativeFilterHz,
  kOutputLimit,
  kSlewPerSecond,
  kMaximumPitchReference,
  kYawGain,
  kYawLimit,
  kUprightOffset,
  kImuSign,
  kWheelSign,
  kCount,
};

class BalanceConfigurationCodec {
public:
  static constexpr size_t kPayloadSize = 5u;

  static size_t encodeValue(BalanceConfigKey key, float value, uint8_t *payload,
                            size_t capacity);
  static size_t encodeCurrent(BalanceConfigKey key,
                              const CascadedBalanceConfig &config,
                              uint8_t *payload, size_t capacity);
  static bool applyUpdate(const uint8_t *payload, size_t length,
                          CascadedBalanceConfig &config);
  static bool value(BalanceConfigKey key, const CascadedBalanceConfig &config,
                    float &result);
};

} // namespace gs::balance
