# Hardware Flash Checklist

Use this checklist only with both drive wheels lifted clear of the ground and an immediate battery disconnect available.

## Use one immutable bundle

Do not mix locally built files or artifacts from different workflow runs. Download the successful `final-flash-bundle-<commit>` artifact produced by the **Final three-controller flash bundle** workflow.

Before flashing:

1. Confirm the artifact name contains the exact branch HEAD commit.
2. Open `MANIFEST.txt` and confirm the same commit is recorded there.
3. Run `sha256sum -c SHA256SUMS` from inside the extracted bundle. On macOS, use `shasum -a 256 -c SHA256SUMS` if GNU `sha256sum` is unavailable.
4. Confirm the manifest reports protocol version `3`, command marker `0x32`, and SLAVE feedback marker `0x33`.

Protocol epoch 3 deliberately prevents mixed firmware from communicating. If any one of ESP32, MASTER, or SLAVE is from an older build, command or feedback frames are rejected and motion remains disabled.

## Required images

The one artifact must contain:

```text
gausstop_slave/firmware.bin
gausstop_master_swd/firmware.bin
esp32_drive_coordinator/bootloader.bin
esp32_drive_coordinator/partitions.bin
esp32_drive_coordinator/firmware.bin
```

The current branch changes the SLAVE commutation profile, ESP32 startup handshake, Pi startup verification, and the complete southbound protocol epoch. All three controllers must be flashed from this same artifact.

## Flash order

1. Stop the Raspberry Pi drive service and disconnect USB serial from the ESP32.
2. Back up and identify each GD32 before writing. Do not assume which board is MASTER or SLAVE.
3. Flash and read back `gausstop_slave/firmware.bin` to the confirmed SLAVE controller.
4. Flash and read back `gausstop_master_swd/firmware.bin` to the confirmed MASTER controller.
5. Remove the ST-Link completely from the MASTER SWD header.
6. Flash the ESP32 bootloader, partition table, and application from `esp32_drive_coordinator`, preserving NVS unless recovery requires otherwise.
7. Read back the ESP32 application region and compare it with `firmware.bin`.
8. Power-cycle the complete controller stack. Do not rely on software reset alone.

## Required validation before motion

- Confirm both GD32 firmware readbacks match their bundled binaries byte-for-byte.
- Confirm the ESP32 application readback matches the bundled `firmware.bin`.
- Confirm the Pi receives compatible production capabilities, not a dry-run firmware response.
- Confirm MASTER and SLAVE feedback is fresh, protocol-compatible, and continuously increasing.
- Confirm CRC errors, malformed-frame counts, transport overflows, and acknowledgment timeouts stay at zero.
- Confirm both controller fault words and the ESP32 safety fault word are zero.
- Confirm the disabled startup state automatically advances to an exact READY zero acknowledgment.
- Confirm ARM is acknowledged and then independently confirmed by a fresh armed-at-zero status response.
- Confirm applied output remains `[0,0]` and both bridges remain disabled throughout startup and ARM-at-zero confirmation.
- Leave the stack armed at zero for at least 10 seconds and require no fault, timeout, stale feedback, bridge enable, or output drift.

## Lifted-wheel motion sequence

Run bounded tests independently:

1. Left forward, then left reverse.
2. Right forward, then right reverse.
3. Both forward, then both reverse.
4. Repeat the complete sequence at least three times.

Start below full command. Increase to `±250` only after Hall transitions, odometry direction, applied output, and acknowledgments remain coherent. Abort immediately on an invalid Hall state, stale feedback, controller fault, unexpected bridge enable, missing odometry, direction mismatch, CRC error, timeout, or transport overflow.

End every attempt with exact zero, STOP, and DISARM. Restore the Raspberry Pi service only after the direct test has released the serial port.

Do not perform a floor test until right-forward produces valid Hall odometry without `GS_FAULT_STARTUP_TIMEOUT`, all four independent direction tests pass repeatedly, and the ten-second armed-zero soak remains clean.
