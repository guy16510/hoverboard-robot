# Codex Goal

Use branch `codex/swd-esp32-dual-motor-control`.

Flash and verify the committed SLAVE, MASTER, and ESP32 images, then validate bidirectional ESP32-to-MASTER communication and low-command independent movement of both secured, lifted wheels.

Runtime wiring:

```text
ESP32 GPIO17 TX -> MASTER PA13 / SWDIO
ESP32 GPIO35 RX <- MASTER PA14 / SWCLK
ESP32 GND       -> MASTER GND
```

Use the committed binaries under `dist/` and the existing repository flashing tools. Flash SLAVE first, MASTER second, and ESP32 last. Exact-readback-verify both GD32 images. Never connect ST-Link power. Disconnect ST-Link SWDIO and SWCLK before runtime.

After flashing, require increasing transmit and receive counters, low feedback age, zero CRC errors, and zero MASTER and SLAVE faults. With both wheels lifted and secured, use `tools/drive_esp32.py` for brief low-command tests of the left wheel, right wheel, both wheels, and reverse directions. Confirm stale commands stop and disable motion automatically.

Proceed through software work automatically. Group physical instructions whenever safe. Ask only when I must change wiring, move ST-Link, hold the power button, apply power, identify the ESP32 port, or observe wheel behavior.

Do not perform a ground test or increase command magnitude beyond the existing conservative bench limit. Finish with both motors disabled and report verification hashes, communication evidence, wheel-test results, timeout behavior, and faults.