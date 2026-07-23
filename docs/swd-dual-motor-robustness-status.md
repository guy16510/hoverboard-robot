# SWD Dual-Motor Robustness Status

This branch implements the first safety gates needed before another energized motion test. It does not claim the dual-wheel path is production-ready.

## Implemented

- Protocol v2 uses new wire markers and rejects protocol v1 frames.
- ESP32 commands carry a monotonic sequence number.
- MASTER feedback reports the sequence accepted by MASTER, forwarded to SLAVE, and accepted by SLAVE.
- Nonzero demand is held at zero until both controllers acknowledge a healthy `READY,READY` session.
- MASTER faults and disables both demands if SLAVE feedback is stale, faulted, shut down, unexpectedly disabled, or fails to acknowledge the forwarded command.
- Commands below the 50-unit deadband normalize to zero before coordinator, safety, motor, and SLAVE-link evaluation.
- Duplicate sequences are idempotent only when their payload is identical. Older or mutated duplicate frames are rejected.
- Fault clear is coordinated through live PA4, PA6, and Hall samples. Safety clears first, then motor startup/Hall history and the coordinator fault word clear.
- Runtime status includes sequence acknowledgments, requested/ramped/applied demand, controller states, faults, link ages, physical PA4 level, PA4 bypass state, and transport counters available at each layer.
- `tools/drive_esp32.py` requires zero-demand acknowledgment before motion, reads output asynchronously, stops on stale/faulted/unacknowledged control, and writes timestamped JSONL evidence.
- CI is read-only, publishes immutable artifacts with a manifest and hashes, checks clean-build reproducibility, and refuses committed `dist/` firmware.
- The PA4 bypass is identified as a bench-only artifact in telemetry, CI output, artifact path, and manifest.

## Still blocked before claiming robust motor operation

- SWD software UART transmission remains synchronous and masks receive interrupts while sending feedback.
- The normal GD32 UART writer remains polling-based.
- Hall transitions are still acquired by the 1 ms service loop rather than GPIO interrupt or timer capture.
- Bridge topology and PWM authority still require bounded electrical measurement and lifted-wheel validation on MASTER and SLAVE independently.
- The final images produced from this source have not been flashed, read back, or motion-tested.
- Firmware source commit/build identity is present in the CI manifest, but it is not yet embedded as a full commit string in each MCU image.

Do not perform a ground test. The next hardware milestone is a readback-verified protocol-v2 flash followed by a zero-demand `READY,READY` acknowledgment test.
