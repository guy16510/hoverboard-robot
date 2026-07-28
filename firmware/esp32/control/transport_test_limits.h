/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::balance {

// The GD32 SWD motor profile reaches full PWM at command 250. Keeping this
// transport ceiling at 100 prevented powered tests from reaching the profile's
// proven startup output.
constexpr int16_t kMaximumTransportTestCommand = 250;

} // namespace gs::balance
