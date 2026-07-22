/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef GS_WHEEL_MIX_H
#define GS_WHEEL_MIX_H

#include "gs_types.h"

gs_wheel_pair gs_mix_wheels(int16_t speed, int16_t steer);
gs_wheel_pair gs_direct_wheels(int16_t left, int16_t right);
int16_t gs_slave_electrical_command(int16_t logical_right);
int32_t gs_slave_logical_odometer(int32_t electrical_odometer);

#endif
