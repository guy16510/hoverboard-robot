# Authorized hardware validation procedure

Do not execute this document on the current computer. It was prepared without
controller access, ST-Link, USB devices, serial inspection, electrical
measurement, flashing, or motor operation. Every result below remains
**Awaiting hardware validation**.

Proceed only on an authorized, electrically suitable hardware computer with
the wheels unloaded unless a later stage explicitly says otherwise. Stop after
any mismatch; do not use uncontrolled retry loops.

## Stage 1: Inspect artifacts

1. Check each `manifest.txt` hash against its BIN/ELF.
2. Confirm target names and the `GAUSSTOP_DPHC-V3.3_GD32F130C8T6` board.
3. Confirm GD32 images are within 64 KiB and static RAM within 7,104 bytes.
4. Confirm release flags and that no accidental debug image is selected.
5. Select `gausstop_safe_recovery` first, not master or slave operational code.

## Stage 2: Recover master SWD access

Power connections:

```text
ST-Link GND   -> controller GND
ST-Link SWCLK -> controller CLK
ST-Link SWDIO -> controller DIO
ST-Link NRST  -> controller reset
ST-Link 3.3 V -> disconnected
ST-Link 5 V   -> disconnected
ESP32         -> disconnected
```

Use connect-under-reset at 100 kHz or lower. Read exactly `0x10000` bytes from
`0x08000000` before writing. Preserve the original with date, board identity,
byte length, and SHA-256. Read it a second time and require exact-length,
byte-for-byte, and SHA-256 agreement. Stop after one failed attempt and diagnose
connections; do not create an uncontrolled reconnect loop.

## Stage 3: Verify slave flash

Repeat the exact 64 KiB preservation read, second read, byte comparison, and
SHA-256 procedure for the slave. Do not assume legacy firmware remains present.

## Stage 4: Safe recovery image

Flash only `gausstop_safe_recovery`. Verify exact-length byte-for-byte readback
and SHA-256 against the artifact. Verify repeated reset and debugger attachment.
The image's fixed SRAM heartbeat may be inspected; no motor or communication
peripheral should be expected from this target.

## Stage 5: Bridge-disabled ESP32 communication

Use `gausstop_communication_diagnostic`, and prove bridge outputs remain
disabled throughout. Separately validate:

- MCU PB6/PB7 electrical activity
- exposed connector activity
- ESP32 receive on GPIO35
- ESP32 transmit on GPIO17
- CRC-valid command reception
- CRC-valid feedback reception
- 100 ms parser and 400 ms command timeout behavior

Do not infer connector routing from MCU-pin activity. The historic external
PB6/PB7 route was not proven.

## Stage 6: Bridge-disabled master/slave communication

With diagnostic firmware and one ESP32, prove PA2/PA3 commands and slave
feedback in both directions while both bridges remain disabled. Confirm only
zero/disabled slave commands are emitted.

## Stage 7: Single master motor

Install operational master firmware only after Stages 1--6 pass. Lift the wheel
and use low bounded demand with PWM ceiling 100. Verify Hall ordering, initial
commutation selection, startup, coast stop, command timeout, PA4/PA6/fault
behavior, and electrical/mechanical direction. Restore recovery firmware after
an unexplained result.

## Stage 8: Slave motor through master

Use the same ESP32. Do not connect or introduce a second ESP32. Verify slave
operation only through master commands, including the 100 ms link timeout,
mirrored electrical command, Hall count, fault feedback, and coast stop.

## Stage 9: Dual-motor unloaded operation

Lift both wheels. Verify correct wheel direction, mirrored mounting,
coordinated chassis-forward intent, reverse, pivot and asymmetric turns,
command timeout, slave timeout, disable, shutdown, and emergency stop behavior.

## Stage 10: Loaded validation

Plan and record, but do not pre-claim: sustained loaded operation, thermal
monitoring, battery sag, undervoltage behavior, PA6/current calibration,
transition-rate/speed calibration, chassis-forward direction, repeated reversal,
and long-duration fault/timeout behavior.

## Exact first authorized-hardware action

Before connecting an ST-Link or ESP32, run Stage 1 against the
`gausstop_safe_recovery` manifest and verify its target name, release flags,
byte size, and SHA-256. Then connect only the Stage 2 SWD wires with both ST-Link
power pins and the ESP32 disconnected.
