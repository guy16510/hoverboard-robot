# Right wheel Hall sensor wiring

The ESP32 reads the right hoverboard wheel's three Hall signals directly and publishes movement telemetry to the Raspberry Pi / Donkeycar protocol.

## ESP32 pins

| Motor Hall lead | ESP32 |
|---|---:|
| Hall A, typically yellow | GPIO19 |
| Hall B, typically green | GPIO21 |
| Hall C, typically blue | GPIO22 |
| Hall ground, typically black | GND |
| Hall supply, typically red | Use the voltage required by the Hall sensors |

The firmware configures GPIO19, GPIO21, and GPIO22 as `INPUT_PULLUP`. No external pull-up resistors are required for open-collector Hall outputs.

## Resistor-free hookup

For a raw hoverboard Hall harness whose three outputs are open collector:

```text
RIGHT MOTOR HALL             ESP32
Yellow / Hall A  ----------> GPIO19
Green  / Hall B  ----------> GPIO21
Blue   / Hall C  ----------> GPIO22
Black  / GND     ----------> GND
Red    / Hall V+ ----------> Hall sensor supply
```

Do not blindly connect a 5 V push-pull Hall signal to an ESP32 input. ESP32 GPIO is a 3.3 V interface. If the motor Hall sensors require 5 V, the resistor-free arrangement is only appropriate when the Hall outputs themselves are open collector and therefore pulled up by the ESP32 to 3.3 V. If the signal wires measure about 5 V while disconnected from the ESP32, level shifting is required unless sacrificing the ESP32 is acceptable.

If the Hall sensors operate correctly from 3.3 V, powering the red Hall lead from ESP32 3V3 is the simplest resistor-free test. If they do not toggle at 3.3 V, use the Hall sensor's required supply voltage and verify the output type before connecting the signal wires.

## Firmware telemetry

The ESP32 sends Hall telemetry every 50 ms using protocol message `0x36`. The payload reports:

- current three-bit Hall state, valid states are 1 through 6
- whether the Hall state is valid
- whether transitions have occurred recently
- total valid Hall transitions since boot
- invalid states, 0 or 7
- skipped transitions where multiple Hall bits changed at once
- transitions per second
- age of the latest transition

This is intentionally transition-based rather than wheel-distance-based. It proves the wheel is moving without assuming a wheel circumference, motor pole count, or Hall sequence orientation that has not been calibrated yet.

## Donkeycar signals

The Raspberry Pi exposes the right wheel data as:

```text
hall/right_state
hall/right_valid
hall/right_moving
hall/right_transitions
hall/right_tps
hall/right_invalid_states
hall/right_skipped_transitions
hall/right_age_s
```

The same values are copied into the dashboard state under `right_hall` and written into each JSON run log.

## Bench check

With motor power off, power the ESP32 and Hall sensors, then rotate the right wheel slowly by hand. `hall/right_state` should cycle only through values 1 through 6, `hall/right_transitions` should increase, and `hall/right_moving` should become true while the wheel is turning.

If the state stays at 0 or 7, or the transition count does not increase, stop and verify Hall supply voltage, ground, and the three signal wires before powering the motor controller.
