# SWD-pin ESP32 motor control

This branch targets one ESP32 connected to the MASTER controller through the exposed SWD header.

## Wiring

- ESP32 GPIO17 TX to MASTER PA13 / SWDIO
- ESP32 GPIO35 RX from MASTER PA14 / SWCLK
- ESP32 GND to MASTER GND
- ST-Link signal wires disconnected during runtime
- All signals are 3.3 V logic

## Runtime behavior

- MASTER leaves SWD active for two seconds after reset, then takes over PA13 and PA14.
- ESP32 sends independent signed left and right commands.
- MASTER drives the local wheel and forwards the right-wheel command to SLAVE over PA2/PA3.
- Both controllers start disabled.
- Continuous valid command frames are required.
- Missing commands, CRC errors, controller faults, or link loss disable both motors.
- Initial physical validation uses a conservative command cap. The cap must not be raised until both wheels pass lifted-wheel and low-load testing.

## Future MPU6050 support

The Raspberry Pi supplies high-level velocity and steering targets. The ESP32 owns the future deterministic balance loop and MPU6050 sampling. The GD32 controllers remain responsible for commutation, current/protection checks, command watchdogs, and bridge shutdown.
