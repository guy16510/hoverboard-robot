# ESP32 production pin map

This is the complete GPIO contract for the classic ESP32 DevKit used by the WinXu drivetrain. The map is intentionally centralized in `firmware/esp32/board_config.h` and protected by compile-time uniqueness checks plus Pi-side contract tests.

| Function | GPIO | Direction | Current status |
|---|---:|---|---|
| Left throttle DAC | 25 | output | active |
| Right throttle DAC | 26 | output | active |
| Left reverse | 27 | output | active |
| Right reverse | 14 | output | active |
| Left brake | 33 | output | active |
| Right brake | 32 | output | active |
| Right Hall A | 19 | input | active |
| Right Hall B | 21 | input | active |
| Right Hall C | 22 | input | active |
| Left Hall A | 16 | input | reserved, disabled until wired |
| Left Hall B | 17 | input | reserved, disabled until wired |
| Left Hall C | 36 | input-only | reserved, disabled until wired |
| Front ultrasonic TRIG | 5 | output | active |
| Front ultrasonic ECHO | 34 | input-only | active |
| Left ultrasonic TRIG | 15 | output | active |
| Left ultrasonic ECHO | 35 | input-only | active |
| Right ultrasonic TRIG | 18 | output | active |
| Right ultrasonic ECHO | 39 | input-only | active |
| MPU6050 SDA | 4 | bidirectional | reserved |
| MPU6050 SCL | 23 | output/open-drain | reserved |
| Arm servo signal | 13 | output | reserved |

## Design rules

- GPIO25 and GPIO26 stay dedicated to DAC throttle output.
- GPIO34, GPIO35, GPIO36, and GPIO39 are input-only and are never assigned to an output function.
- GPIO6 through GPIO11 are not used because they are tied to ESP32 flash on common modules.
- GPIO5 and GPIO15 are strapping pins. They are used only as ultrasonic TRIG outputs connected to high-impedance module inputs. Do not add external hardware that drives those pins during boot.
- Right and left Hall signals are treated as controller-connected 5 V signals and must be reduced to 3.3 V before reaching the ESP32.
- Left Hall interrupt capture is disabled in firmware until the left Hall taps are physically wired. This prevents floating inputs from creating an interrupt storm.
- MPU6050 and servo pins are reserved now to prevent later feature work from colliding with drivetrain pins. Reserving a pin does not mean that feature is active yet.

## Hall protocol

- `0x36` is right-wheel Hall telemetry.
- `0x37` is left-wheel Hall telemetry.
- Donkeycar exposes independent right and left Hall states. Left remains invalid/stale while `kLeftHallEnabled` is false.

## Current bring-up gate

Only validate the right wheel first:

```bash
python donkeycar/scripts/validate_right_hall.py --confirm-lifted
```

This test algebraically commands left wheel demand to zero and right wheel demand forward at a conservative level, while validating only the right Hall channel.
