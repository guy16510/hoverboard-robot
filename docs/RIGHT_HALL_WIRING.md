# Right wheel Hall sensor wiring

The right motor controller keeps its Hall connector wired normally. The ESP32 only taps the three Hall signal wires through voltage dividers so Donkeycar can observe wheel movement without interfering with controller commutation.

## Confirmed electrical behavior

The controller-connected Hall signal measures about 5 V when HIGH and about 2.5 V on a multimeter while the wheel spins quickly. The 2.5 V moving reading is the meter averaging a digital signal that is switching between about 0 V and 5 V. Treat the Hall outputs as 5 V signals.

Do not connect those 5 V Hall signal wires directly to ESP32 GPIO.

## Right wheel ESP32 pins

| Motor Hall lead | Divider output to ESP32 |
|---|---:|
| Hall A, typically yellow | GPIO19 |
| Hall B, typically green | GPIO21 |
| Hall C, typically blue | GPIO22 |
| Hall ground, typically black | ESP32 GND |
| Hall supply, typically red | Leave connected to motor controller |

The firmware configures GPIO19, GPIO21, and GPIO22 as plain `INPUT`. The motor controller already supplies the Hall pull-ups.

## Divider using only 10k resistors

Use three 10k resistors per Hall signal. One 10k is the upper resistor. Two 10k resistors in series form the 20k lower leg.

```text
existing 5 V Hall signal wire
          |
         10k
          |
          +---------- ESP32 GPIO19 / GPIO21 / GPIO22
          |
         10k
          |
         10k
          |
COMMON GND+---------- ESP32 GND
          |
          +---------- controller Hall GND / motor Hall black
```

The existing Hall signal wire remains connected to the motor controller. The divider is a tap off that wire. Do not cut the controller out of the Hall circuit.

At a 5 V Hall HIGH, 10k over 20k produces about 3.33 V at the GPIO junction.

Build this same divider three times, once for yellow, green, and blue.

## Firmware telemetry

The ESP32 sends right Hall telemetry every 50 ms using protocol message `0x36`. The payload reports:

- current three-bit Hall state, valid states are 1 through 6
- whether the Hall state is valid
- whether transitions have occurred recently
- total valid Hall transitions since boot
- invalid states, 0 or 7
- skipped transitions where multiple Hall bits changed at once
- transitions per second
- age of the latest transition

This remains transition-based until wheel circumference and motor Hall transitions per mechanical revolution are calibrated.

## Donkeycar signals

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

## First physical test

Lift the right wheel off the ground. Leave the left wheel uncommanded. After flashing the new ESP32 firmware and updating the Pi code, run:

```bash
python donkeycar/scripts/validate_right_hall.py --confirm-lifted
```

The test first requires a valid right Hall state, then applies a conservative right-wheel-only forward demand for about 1.5 seconds. It passes only if the right Hall transition count increases, movement is observed, and no new invalid or skipped Hall transitions are recorded. It explicitly commands zero before disconnecting, and disconnect sends STOP and DISARM as an additional safety layer.

If the right Hall state is 0 or 7 before motion, the test refuses to move the wheel. Check the common ground and all three divider taps first.
