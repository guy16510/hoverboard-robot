# ESP32 balance wiring and diagnostic checkpoint

The balance coordinator preserves the proven motor-controller wiring:

```text
ESP32 GPIO17 -> MASTER command input (PA13 / SWDIO pulse transport)
ESP32 GPIO35 <- MASTER feedback output (PA14 / SWCLK UART)
ESP32 GND    -> MASTER GND
```

The MPU6050 uses:

```text
ESP32 GPIO21 -> MPU6050 SDA
ESP32 GPIO22 -> MPU6050 SCL
ESP32 3.3 V  -> MPU6050 VCC
ESP32 GND    -> MPU6050 GND
```

Use the ESP32 USB connection for the Raspberry Pi protocol. Do not place Pi
UART traffic on GPIO17, GPIO35, GPIO21, or GPIO22.

## Grouped Stage 3 physical steps

Perform this group only after agreeing to the physical checkpoint:

1. Remove motor power. Leave both MASTER and SLAVE disabled. Disconnect ST-Link
   SWDIO and SWCLK so it cannot contend with GPIO17/35.
2. Connect MPU6050 VCC to ESP32 3.3 V, GND to GND, SDA to GPIO21, and SCL to
   GPIO22. Do not use 5 V.
3. Connect only ESP32 USB power. Identify the new serial device without
   guessing; on macOS compare `/dev/cu.*` before and after connection, and on
   Linux compare `/dev/serial/by-id/*`.
4. Flash only the locally built `esp32_balance_coordinator` image. Do not
   reflash MASTER or SLAVE for this checkpoint.
5. Open 115200-baud telemetry. Keep the chassis stationary until
   `calibrated:1`; do not touch it during the 400-sample gyro calibration.
6. Record sensor address (`104` / `0x68` or `105` / `0x69`), WHO_AM_I success
   (`0x68` for MPU-6050 or `0x70` for register-compatible MPU-6500 silicon),
   accepted calibration samples, raw and bias-corrected gyro vectors, calculated
   gyro-bias vector, sample rate, loop
   rate, min/max period, worst jitter, I2C errors, and missed samples for at
   least 30 seconds.
7. Tilt forward slowly, return upright, then tilt backward slowly. Record the
   sign and approximate magnitude of raw and filtered pitch. Do not assume the
   sign is correct until chassis-forward is identified.
8. Leave the chassis stationary again and confirm pitch rate returns near zero,
   bias does not drift, and I2C errors do not grow.
9. Disconnect USB power. Motors remain unpowered throughout this stage.

Expected safety evidence:

- firmware banner says `dry_run=1`;
- state progresses through `IMU_CALIBRATING` to `DISARMED` only with a healthy
  MPU;
- calculated left/right terms may be nonzero, but applied motor commands remain
  disabled and zero;
- an MPU timeout leads to `FAULT`;
- no ground-balancing attempt is made.

When the hardware is on another computer, use
[`remote-balance-evidence.md`](remote-balance-evidence.md) to capture the raw
serial bytes, decoded telemetry, operator action timestamps, firmware hash, and
source state as one checksummed archive.
