/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

namespace gs::balance::user_config {

// Change this one value after the final IMU case/mount is installed. It is the
// raw IMU pitch, in degrees, that represents the robot's mechanical upright.
constexpr float kUprightMountingOffsetDeg = 20.0f;

// These remain expressed relative to the configured mechanical upright above.
constexpr float kArmingToleranceDeg = 5.0f;
constexpr float kFallAngleDeg = 20.0f;

} // namespace gs::balance::user_config
