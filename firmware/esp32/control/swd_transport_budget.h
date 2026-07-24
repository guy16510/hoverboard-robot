/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstddef>
#include <cstdint>

namespace gs::balance {

class SwdTransportBudget {
public:
  static constexpr uint32_t
  frameDurationUs(uint16_t unit_us, size_t frame_bytes, uint8_t symbol_value) {
    return unit_us * 10u + static_cast<uint32_t>(frame_bytes * 4u) * unit_us *
                               (static_cast<uint32_t>(symbol_value) + 2u);
  }

  static constexpr float maximumRateHz(uint32_t frame_duration_us) {
    return 1000000.0f / static_cast<float>(frame_duration_us);
  }
};

} // namespace gs::balance
