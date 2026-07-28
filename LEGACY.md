# Legacy evidence inventory

The read-only legacy tree was inspected at
`/Users/burnschris/Git/tc-automation-main`. It is evidence, not a build
dependency. No captures, binaries, history, or unrelated automation were
imported.

## Relevant files reviewed

- `protocol/gen2_serial.c`, `protocol/gen2_serial.h`, `protocol/gausstop_protocol.h`
- `controller/gausstop_control.c/.h`, `gausstop_runtime.c/.h`,
  `gausstop_endpoint.c/.h`, `gausstop_speed.c/.h`
- `firmware/gausstop-controller/src/main_gen2.c`, `main_v2.c`,
  `main_uart_diag.c`, shared controller modules, and `platformio.ini`
- `firmware/safe-bringup/src/main.c` and `platformio.ini`
- `firmware/gausstop-closedloop-proof/src/main.c` and split include files
- `esp32/gausstop-controller-client/src/main.cpp`, `passive_uart_probe.cpp`,
  `gausstop_client.c/.h`, `gausstop_dual.c/.h`, `shared_gen2_serial.c`, and
  `platformio.ini`
- C tests for protocol, controller hardening, state machine, runtime,
  endpoint, speed, client workflow, and dual-client behavior
- JavaScript tests for Hall sequences, UART diagnostics, protection,
  closed-loop behavior, and motor-2 mirroring
- `docs/gausstop-pinmap-evidence.md`, `gausstop-target1-bringup.md`,
  `gausstop-upstream-layout-adaptation.md`, `gausstop-dphc-v3.3-b.md`,
  `esp32-controller-protocol.md`, and `protocol-discovery.md`
- `hardware/gausstop-layout-comparison.json`

## Behavior retained

| Legacy source | Retained behavior | Reason |
|---|---|---|
| `main_gen2.c` and pin-map docs | PA8/9/10, PB13/14/15, PB11/PA0/PC14, PB2, PA4, PA6 | GAUSSTOP-specific, **Historically physically verified** evidence |
| `main_gen2.c`, Hall tests, and physical captures | Shared measured Hall cycles; first-wheel phase-advanced reverse table; second-wheel reverse remained unproven | Preserves proven first-wheel behavior without presenting its reverse phase advance as proven for motor 2 |
| `main_gen2.c` | One ESP32 -> master -> slave topology and slave orientation inversion | Proven integration design; treated as **Legacy software-tested** here |
| `gen2_serial.c` and protocol tests | CRC-16/0x1021 behavior and start markers | Interoperability and **Legacy software-tested** vectors |
| controller safety modules/tests | zero startup, bounded ramps, timeout, direction dwell, latched faults | Required safety behavior absent or less explicit upstream |
| `safe-bringup/main.c` | fixed SRAM heartbeat concept | Debugger-observable recovery without communications |
| `passive_uart_probe.cpp` | receive-only probe role | Prevents accidental transmission during routing investigation |

All retained behavior was reimplemented behind clean interfaces and receives
new native tests. No raw implementation file was copied.

## Rejected material

- packed-struct serialization and compiler-layout-dependent frames
- generic experimental protocol variants and floating-point telemetry
- zero-valued current, voltage, or speed presented as measurements
- two-ESP32 control paths and direct ESP32-to-slave addressing
- Target 2 and STM32F103 assumptions
- PA13/SWDIO bit-banging or production transport
- arbitrary motor-2 commutation tables instead of explicit orientation
- Arduino `String` command parsing and implicit enable-on-drive behavior
- capture logs, readbacks, firmware binaries, map files, and USB transcripts
- abandoned UART, gate, open-loop, Hall-recorder, and protection experiments
- unrelated trashcan automation code and source-text-only assertions

Historical findings remain historical. They are not new physical validation
claims from this computer.
