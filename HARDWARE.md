# GAUSSTOP DPHC-V3.3 hardware definition

Every statement uses one required evidence classification. These labels describe
the source of evidence, not a new physical result.

## Supported target

| Classification | Property | Value |
|---|---|---|
| **Historically physically verified** | Controller boards | GAUSSTOP DPHC-V3.3-A and DPHC-V3.3-B |
| **Historically physically verified** | Associated hoverboard | Swagtron T882 |
| **Historically physically verified** | MCU marking | GD32F130C8T6, Cortex-M3, LQFP48 |
| **Upstream-defined** | PlatformIO board | `genericGD32F130C8`, Target 1 |
| **Statically validated** | Flash/RAM layout | 64 KiB flash, 8 KiB SRAM |
| **Build validated** | Runtime clock selection | 8 MHz internal IRC, no PLL |

The PlatformIO board description may print a 48 MHz capability. The build
defines `__PIO_DONT_SET_CLOCK_SOURCE__`, `__SYSTEM_CLOCK_8M_IRC8M`, and
`GS_SYSTEM_CLOCK_HZ=8000000`; the pinned SPL initializes `SystemCoreClock` to
that selected value. **Build validated**

This is not an STM32F103 target. The debugger-compatible ID `0x0410` does not
override the identified GD32F130C8T6. **Historically physically verified**

## Dedicated pin map

| Function | Assignment | Classification |
|---|---|---|
| High-side PWM | PA8, PA9, PA10 | **Historically physically verified** |
| Complementary low-side PWM | PB13, PB14, PB15 | **Historically physically verified** |
| HALL_A / bit 0 | PB11 | **Historically physically verified** |
| HALL_B / bit 1 | PA0 | **Historically physically verified** |
| HALL_C / bit 2 | PC14 | **Historically physically verified** |
| Required latch | PB2 output high | **Historically physically verified** |
| Shutdown input | PA4 active low, weak pull-up | **Historically physically verified** behavior; circuit source **Awaiting hardware validation** |
| Relative protection ADC | PA6 | **Historically physically verified** behavior; calibration **Awaiting hardware validation** |
| Timer-break candidate | PB12 input only, break disabled | **Inferred** candidate; electrical role **Awaiting hardware validation** |
| ESP32 transport | USART0 PB6 TX / PB7 RX | **Upstream-defined** intended mapping; connector route **Awaiting hardware validation** |
| Master/slave transport | USART1 PA2 TX / PA3 RX | **Legacy software-tested** architecture |

Upstream Target 1 layout 13 is a nearby reference, not an exact GAUSSTOP
layout. Its Hall mapping is not silently substituted. **Statically validated**

All bridge-pin uniqueness, Hall/output separation, PA4/PA6/PB2 separation,
PB12-disable state, target, clock, and memory constants are compile-time
assertions in `gausstop_board.h`. **Statically validated**

## Hall and bridge configuration

The forward Hall cycle is `010, 011, 001, 101, 100, 110, 010`.
**Historically physically verified**

The shared phase vectors are `Y+B-`, `Y+G-`, `B+G-`, `B+Y-`, `G+Y-`, and
`G+B-`. Motor 2 uses the same implementation with an explicit `-1` slave
orientation; it does not use a separate arbitrary table. **Legacy software-tested**

TIMER0 is center aligned with period 999, midpoint 500, prescaler 0, divider 1,
primary active-high/idle-low, complementary active-low/idle-high, dead-time
register 120, 2 MHz push-pull alternate-function GPIO, fast mode disabled,
and auto-reload shadowing disabled. **Build validated**

At an 8 MHz timer clock, 120 ticks imply approximately 15 microseconds of dead
time. This is a calculation, not a measurement. **Inferred**

The floating phase has both timer channel outputs disabled by the bridge
adapter. TIMER0 primary output and every channel start disabled. **Statically validated**

## Unresolved hardware risks

1. Current master flash may be corrupt or partially erased. **Awaiting hardware validation**
2. The current slave image is unknown. **Awaiting hardware validation**
3. Exposed connector routing to PB6/PB7 is unproven. **Awaiting hardware validation**
4. PA13/SWDIO carried a historic diagnostic beacon; bidirectional production use is unproven. **Historically physically verified** / **Awaiting hardware validation**
5. ST-Link and ESP32 must never drive SWDIO simultaneously. **Inferred**
6. PA4's exact electrical source is unknown. **Awaiting hardware validation**
7. PA6 is uncalibrated. **Awaiting hardware validation**
8. PB12 hardware break is unproven and disabled. **Awaiting hardware validation**
9. Voltage, current, and speed feedback are uncalibrated and omitted. **Legacy software-tested**
10. Sustained loaded operation is unproven. **Awaiting hardware validation**
11. Long-duration thermal behavior is unproven. **Awaiting hardware validation**
12. Battery undervoltage protection is not calibrated. **Awaiting hardware validation**
13. Motor and chassis-forward directions require physical confirmation. **Awaiting hardware validation**

