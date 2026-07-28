# Hardware Flash Checklist

Use this checklist only with both drive wheels lifted clear of the ground and an immediate battery disconnect available.

## Required images

Build all three production targets from the same checked-out commit:

```bash
python3 -m platformio run -e gausstop_slave
python3 -m platformio run -e gausstop_master_swd
python3 -m platformio run -c platformio-drive.ini -e esp32_drive_coordinator
```

The current branch changes the slave commutation profile and the ESP32 drive coordinator. Flashing only the GD32 pair leaves the Pi-to-ESP32 drive protocol on the old image. Flashing only the ESP32 leaves the corrected slave forward path absent.

## Flash order

1. Stop the Raspberry Pi drive service and disconnect USB serial from the ESP32.
2. Back up and identify each GD32 before writing. Do not assume which board is MASTER or SLAVE.
3. Flash and read back `gausstop_slave` to the confirmed SLAVE controller.
4. Flash and read back `gausstop_master_swd` to the confirmed MASTER controller.
5. Remove the ST-Link completely from the MASTER SWD header.
6. Flash `esp32_drive_coordinator` to the ESP32, preserving NVS unless recovery requires otherwise.
7. Power-cycle the complete controller stack.

## Required validation before motion

- Confirm both GD32 firmware readbacks match their built binaries.
- Confirm the ESP32 application readback matches `firmware.bin`.
- Confirm MASTER and SLAVE feedback is fresh, CRC errors and acknowledgment timeouts stay at zero, and both fault words are zero.
- Confirm exact zero output is acknowledged by the ESP32, MASTER, and SLAVE before ARM.
- Confirm both bridges are disabled at zero.
- Run bounded lifted-wheel tests independently: left forward/reverse, right forward/reverse, then both wheels.
- Abort immediately on an invalid Hall state, stale feedback, controller fault, unexpected bridge enable, missing odometry, or direction mismatch.
- End every attempt with exact zero, STOP, DISARM, and service restoration only after the direct test owns no serial port.

Do not perform a floor test until right-forward produces valid Hall odometry without `GS_FAULT_STARTUP_TIMEOUT` and all four independent direction tests pass repeatedly.
