# Remote hardware validation handoff

Date: 2026-07-24

## Current hardware state

- ESP32: `/dev/cu.usbserial-0001`, MAC `28:56:2f:4b:73:5c`
- MPU address: `0x68`
- MPU identity: `0x70` (MPU-6500-compatible register map)
- MASTER and SLAVE firmware were not modified or reflashed.
- Installed ESP32 image: `esp32_stage6_lifted`
- Installed image SHA-256:
  `bc2605a8f6eaf82f04bebbaeea619e263f7dca1605385a6cee0afc65c4f4f849`
- The installed image is active, lifted-wheel-only validation firmware. Do not
  use it for a ground-balancing test.

## Physical results

- IMU detected and calibrated at 400/400 samples.
- IMU sampling held approximately 200 Hz with zero growing I2C errors.
- Stage 5 direct transport exercised left, right, equal forward/reverse, and
  differential commands at a maximum absolute command of 50.
- Stage 5 finished disarmed at zero output with zero controller faults, CRC
  errors, or acknowledgment timeouts.
- Stage 6 armed in balance mode with wheels lifted.
- Both wheels received bounded correction; observed applied range included
  left `-41` and right `-33`.
- The 20-degree fall detector transitioned to `FALLEN`, removed output, and the
  run finished `DISARMED` at zero.
- Stage 6 recorded 229 feedback frames with zero controller faults, CRC errors,
  or acknowledgment timeouts.

## Configuration

The future enclosure mounting angle is centralized in:

`firmware/esp32/control/balance_user_config.h`

Change `kUprightMountingOffsetDeg` after installing the final case. The current
global provisional value is `20.0` degrees. Hardware validation used a RAM-only
offset of `-16.3` degrees for the present loose sensor position.

## Evidence locations

- Stage 3 diagnostics:
  `/Users/admin/Desktop/hoverboard-stage3-diagnostics`
- Stage 5 transport:
  `/Users/admin/Desktop/hoverboard-stage5-evidence`
- Stage 6 lifted-wheel correction:
  `/Users/admin/Desktop/hoverboard-stage6-evidence`

Evidence archives have adjacent SHA-256 sidecars. Evidence is intentionally not
committed.

## Continue on another machine

Import the Git bundle, check out `codex/remote-hardware-validation`, and read
this file plus the goal objective before continuing. Copy the three evidence
directories separately if the next machine needs the raw hardware record.

Do not open a pull request, rewrite history, flash MASTER/SLAVE, increase the
motor power profile, or attempt ground balancing without new explicit
authorization.
