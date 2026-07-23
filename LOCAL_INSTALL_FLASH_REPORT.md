# Local Install, Flash, and Validation Report

Status: **in progress; no hardware has been flashed and no motor has been moved**

## Host and repository

- Host: macOS 26.5.1 (Build 25F80), arm64
- Repository: `https://github.com/guy16510/hoverboard-robot.git`
- Default branch: `main`
- Commit: `27549025643332d2285f4153e99282edf75f2e08`
- Commit date: `2026-07-22T16:14:46-04:00`
- Initial checkout state: clean and synchronized with `origin/main`
- Build commands modify the repository's tracked `dist/` outputs, so generated manifests report `git_dirty=true` after the first generated artifact is replaced.

## Installed dependencies

- Python: 3.14.6
- PlatformIO Core: 6.1.19
- Arm GNU Toolchain: 14.2.1 (`arm-none-eabi-gcc`)
- Xtensa ESP32 GCC: 8.4.0, esp-2021r2-patch5
- Arduino-ESP32: pinned commit `5e19e086c43d0fa5e5a596497ff8f11a0a43f6c2` (2.0.17)
- Community GD32 platform: pinned commit `b80521168db1afc6e59cab59ce0685014e0a8111`
- GD32 SPL: pinned commit `8849cb2af35f16431734d9e9c101de648f54f061`
- esptool: 4.11.0
- SCons: 4.8.1
- clang-format: 22.1.8
- Sanitizer host compiler: Homebrew clang 22.1.8
- OpenOCD: 0.12.0

Pinned installation commands:

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
./tools/bootstrap-tools.sh
```

Apple clang 17.0.0 on this macOS release hangs before `main` in ASan's
`InitializeShadowMemory` even for a one-line smoke program. The unchanged test
suite was therefore executed with `CC=/opt/homebrew/opt/llvm/bin/clang` after a
successful ASan/UBSan smoke test. The first Homebrew extraction ran out of disk
space; only that failed extraction residue and the bootstrap script's temporary
download directories were deleted, freeing 5.1 GiB, before retrying.

## Software validation

Commands:

```sh
CC=/opt/homebrew/opt/llvm/bin/clang ./tools/test-all.sh
./tools/build-all.sh
./tools/static-checks.sh
```

Results:

- Strict native tests: 270 assertions passed
- ASan/UBSan native tests: 270 assertions passed
- PlatformIO `native_tests`: success
- Built native executable: 270 assertions passed
- Firmware builds: all six environments succeeded
- Image verification: all six environments succeeded
- Static checks: completed successfully
- Python dependency check: no broken requirements

## Firmware artifacts

| Target / artifact | Bytes | SHA-256 |
|---|---:|---|
| `gausstop_safe_recovery/firmware.bin` | 1,124 | `54514e6cee5f65edaed32949e5022f7b1e093c330a873e0bcde5fa0bc731a6a6` |
| `gausstop_safe_recovery/firmware.elf` | 10,472 | `8657ed938b41781448a7f3ccc73f76b08e492e9cb55164003c04f61206bc56a3` |
| `gausstop_communication_diagnostic/firmware.bin` | 3,800 | `40562ec888795efc70bbb69a8cd56a2948bb71fb597158e3f11dbd28b25139ff` |
| `gausstop_communication_diagnostic/firmware.elf` | 15,716 | `430de71d5041803c84c93f466f34bb7f3563b46fd3b58f1f6941785dc176db7a` |
| `gausstop_master/firmware.bin` | 9,424 | `e8472d082f0ba3d13931b79577982fb818ce2977947eb22a743848f5f396ba2b` |
| `gausstop_master/firmware.elf` | 27,016 | `92249be28d8150d962516688cf6eb1423b00f0ad053fb66be3a0637ef2241099` |
| `gausstop_slave/firmware.bin` | 8,476 | `e6d5b235757018413d51112b8a7e019ce3c66129c75c3a1579b1de60e156ca53` |
| `gausstop_slave/firmware.elf` | 25,340 | `f04e29a67bed3dff58b28ba1d646545fd364888c6249cdaf72d00808fa1fb07d` |
| `esp32_passive_probe/bootloader.bin` | 17,536 | `3d234a7471f67b013686dabd4dee7c1fa915c9928463616a94bc9297acf1abf8` |
| `esp32_passive_probe/partitions.bin` | 3,072 | `148b959cbff1c38aa8e1d5c0ba9d612c54997b945e56a63f41223eef650653a1` |
| `esp32_passive_probe/firmware.bin` | 268,560 | `a612d0a1eb10012d6c4c577827f228a775714f78ee64312906697c4e331ae8ed` |
| `esp32_passive_probe/firmware.elf` | 6,003,864 | `37457b05c0fb4771751a3935b783d0f9215f45d0ac9a7204cdb245d0b3e3a722` |
| `esp32_coordinator/bootloader.bin` | 17,536 | `3d234a7471f67b013686dabd4dee7c1fa915c9928463616a94bc9297acf1abf8` |
| `esp32_coordinator/partitions.bin` | 3,072 | `148b959cbff1c38aa8e1d5c0ba9d612c54997b945e56a63f41223eef650653a1` |
| `esp32_coordinator/firmware.bin` | 271,248 | `ac57929b1705b0c4ef18da33cee808a09e415ecd45ef2e424f2995adaa3e02e4` |
| `esp32_coordinator/firmware.elf` | 6,036,848 | `09569e8f9ec08e84ce77e6f2a85b34d06a68af61646d9ba3b39650bf67543900` |

All artifact paths are under `/Users/admin/Desktop/dev/hoverboard-robot/dist/`.

## Hardware detection and OpenOCD configuration

Initial enumeration found no ST-Link, ESP32, or USB UART device. After the user
confirmed `stlink usb only confirmed`, macOS IORegistry detected:

- Product: `STM32 STLink`
- Manufacturer: `STMicroelectronics`
- VID:PID: `0483:3748` (decimal 1155:14152)
- USB location: `0x01140000`
- USB address: 3
- USB speed: 12 Mbit/s
- Serial number: **not supplied**; both serial-number properties are empty

No new `/dev/cu.*` or `/dev/tty.*` device was created, as expected for this
debug-only ST-Link/V2-class device. No ESP32 or USB UART is currently detected.
Only macOS Bluetooth/debug serial endpoints are present; none will be guessed
or used.

Candidate installed OpenOCD configurations, pending target detection:

- Interface: `/opt/homebrew/opt/open-ocd/share/openocd/scripts/interface/stlink.cfg`
- Target compatibility candidate: `/opt/homebrew/opt/open-ocd/share/openocd/scripts/target/stm32f1x.cfg`
- Required overrides: SWD transport, connect-under-reset, adapter speed 100 kHz,
  8 KiB work area, and explicit 64 KiB flash size.

The STM32-compatible debug ID alone will not be accepted as proof of an
STM32F103. Detection must be compatible with the marked GD32F130C8T6 and 64 KiB
flash before any read or write.

### BOARD A first read-only detection attempt

User confirmations:

- `board a three wire swd confirmed`
- `board a normal power confirmed`

BOARD A exposes GND, DIO/SWDIO, and CLK/SWCLK but no NRST pad. The hoverboard's
manual button controls normal board power and was not connected to ST-Link
NRST. Manufacturer documentation identifies MCU NRST as LQFP48 pin 7, but no
direct MCU-pin connection was attempted.

Log: `hardware_logs/board-a-readonly-detect-20260722T214550Z.log`

Relevant result:

```text
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.225246
Error: init mode failed (unable to connect to the target)
```

No target halt, flash probe, memory read, reset, erase, or write occurred. The
installed `stm32f1x.cfg` reset its default adapter speed to 1000 kHz after the
earlier 100 kHz setting. Any authorized retry must place `adapter speed 100`
after the target configuration is loaded. The procedure stopped after this
single failed attempt pending power removal and physical wiring verification.

After confirmations `board a power removed after detection failure`,
`board a three wire swd rechecked`, and
`board a repowered for 100khz retry`, one corrected retry was made.

Log: `hardware_logs/board-a-readonly-detect-retry-20260722T215009Z.log`

```text
Info : clock speed 100 kHz
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.226824
Error: init mode failed (unable to connect to the target)
```

The retry again failed before target initialization. No target halt, flash
probe, reset, memory read, erase, or write occurred. Adapter speed and abnormal
target voltage are ruled out. The checkpoint is stopped; likely remaining
causes include unavailable NRST/connect-under-reset, SWD disabled or repurposed
by the running image, or lack of electrical continuity from the exposed pads to
MCU PA13/PA14.

A further read-only attempt used OpenOCD's direct ST-Link DAP driver at exactly
100 kHz after the user reported repowering BOARD A and reconnecting ST-Link.

Log: `hardware_logs/board-a-readonly-dapdirect-20260722T215224Z.log`

```text
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.224704
Error: init mode failed (unable to connect to the target)
```

This also failed before target initialization. No reset, target halt, flash
probe, memory read, erase, or write occurred. Both OpenOCD ST-Link drivers have
now failed at 100 kHz with normal target voltage. Further blind retries are not
authorized; progress requires NRST/connect-under-reset access or board-level
continuity diagnosis of GND, PA13/SWDIO, and PA14/SWCLK.

The user then reported that this controller had previously been flashed through
the exposed three-wire header and confirmed `board a ready for held-button
probe`. While the user continuously held the normal power button, a read-only
100 kHz HLA SWD probe succeeded.

Log: `hardware_logs/board-a-held-button-readonly-20260722T215552Z.log`

```text
Info : clock speed 100 kHz
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.215243
Info : [gd32f130.cpu] Cortex-M3 r2p1 processor detected
[gd32f130.cpu] halted due to debug-request, current mode: Handler HardFault
xPSR: 0x01000003 pc: 0xfffffffe msp: 0xffffffd8
Info : device id = 0x13030410
Info : flash size = 64 KiB
```

This proves compatible Cortex-M3 debug access and the configured/probed 64 KiB
flash geometry. The existing firmware was already in HardFault with invalid PC
and MSP state; this is consistent with the observed failure to maintain the
board's power latch. No flash memory was read, erased, or written in this probe.

### BOARD A original-flash backup

With user confirmation `board a held for original flash backup`, the complete
flash range `0x08000000..0x0800FFFF` was read twice in one read-only OpenOCD
session at 100 kHz without reset control.

- UTC timestamp: `2026-07-22T21:58:21Z`
- Directory: `hardware_backups/board-a/20260722T215821Z/`
- Read 1: `board-a-original-read1.bin`, 65,536 bytes
- Read 2: `board-a-original-read2.bin`, 65,536 bytes
- Byte comparison: identical
- SHA-256, both reads: `e7fbfbafb646f5bef46f925fd0d3287039c6a1b0faa312568812e079a2e888c8`
- OpenOCD log: `openocd-read.log`
- Metadata: `metadata.txt`
- Physical identifier: BOARD A; role was unassigned at backup time and was
  subsequently user-designated MASTER before any write
- ST-Link serial: not supplied (empty USB descriptor)
- Detected target: Cortex-M3 r2p1, device ID `0x13030410`, 64 KiB flash,
  target voltage 3.215243 V

No backup file was overwritten and no flash write or erase occurred.

The user subsequently designated BOARD A as **MASTER** with the exact response
`board a is master`; backup metadata was updated accordingly.

Because this physical board exposes no NRST pad, the repository helper now has
an explicit `GAUSSTOP_RESET_MODE=none` path. It retains the original default
connect-under-reset behavior and only selects normal halt when the environment
variable is deliberately set to `none`. Both paths force 100 kHz after target
configuration and retain OpenOCD program verification, exact-length readback,
`cmp`, SHA-256 output, and overwrite refusal. The explicit target configuration
is `tools/hardware/openocd-gd32f130c8.cfg` with 64 KiB flash and 8 KiB SRAM.

### MASTER safe-recovery flash

After user confirmation `master held for safe recovery flash`, the repository
helper was executed with its explicit no-NRST mode:

```sh
printf 'FLASH AND VERIFY GAUSSTOP\n' |
  GAUSSTOP_RESET_MODE=none ./tools/hardware/flash-and-verify.sh \
  /opt/homebrew/opt/open-ocd/share/openocd/scripts/interface/stlink.cfg \
  tools/hardware/openocd-gd32f130c8.cfg \
  dist/gausstop_safe_recovery/firmware.bin
```

Log: `hardware_logs/master-safe-recovery-flash-20260722T220131Z.log`

- SWD speed: 100 kHz
- Target voltage: 3.215243 V
- Device ID: `0x13030410`
- Flash geometry: 64 KiB
- Artifact size: 1,124 bytes
- OpenOCD programming: finished
- OpenOCD verification: `Verified OK`
- Exact readback comparison: passed
- Artifact SHA-256: `54514e6cee5f65edaed32949e5022f7b1e093c330a873e0bcde5fa0bc731a6a6`
- Readback SHA-256: `54514e6cee5f65edaed32949e5022f7b1e093c330a873e0bcde5fa0bc731a6a6`
- Readback: `dist/gausstop_safe_recovery/firmware.bin.readback.bin`

OpenOCD reported page-granularity erase extension through `0x080007ff`; the
complete pre-write 64 KiB image is preserved in the verified BOARD A backup.

### MASTER recovery power-cycle validation

The user pressed and released the normal power button once and reported that
MASTER remained latched on, the prior normal hum returned, and the button did
not need to be held. A 100 kHz post-cycle probe showed valid execution state:

```text
pc: 0x080002a2
msp: 0x20001ff0
HEARTBEAT=0x47535243,0x1,0x130c8c6,0x44504843,0x7a1200,0x53
```

Decoded heartbeat: `GSRC`, format 1, GD32F130C8T6 target, DPHC board,
8,000,000 Hz clock, advancing sequence 0x53. Logs:

- `hardware_logs/master-recovery-post-powercycle-20260722T220331Z.log`
- `hardware_logs/master-recovery-heartbeat-20260722T220346Z.log`

### MASTER communication-diagnostic flash failure

The required repository helper was then invoked for
`gausstop_communication_diagnostic`. Log:
`hardware_logs/master-communication-diagnostic-flash-20260722T220400Z.log`.

```text
** Programming Started **
Info : device id = 0x13030410
Info : flash size = 64 KiB
Error: failed erasing sectors 0 to 3
** Programming Failed **
```

The checkpoint stopped immediately. No successful programming, verification,
readback, comparison, or hash occurred. The post-failure contents are unknown;
safe recovery is not assumed intact. The verified original 64 KiB backup is
unchanged.

### MASTER post-failure read and diagnostic retry

At the user's request to proceed, a complete read-only post-failure image was
captured before another write:

- Path: `hardware_readbacks/master-post-failure/20260722T220538Z/master-post-diagnostic-failure.bin`
- Size: 65,536 bytes
- SHA-256: `3e6746d06100d3d5188b87fa5e9c87f397ddb519140e4300744f92622d101097`
- Safe-recovery prefix: exact match for all 1,124 artifact bytes
- Remaining bytes through the first four 1 KiB pages: erased (`0xFF`)

This proved the failed erase had not corrupted the active safe-recovery image.
After the power cycle, one controlled retry used the same repository helper and
configuration. Log:
`hardware_logs/master-communication-diagnostic-flash-retry-20260722T220650Z.log`.

- Artifact size: 3,800 bytes
- OpenOCD programming: finished
- OpenOCD verification: `Verified OK`
- Exact readback comparison: passed
- Artifact SHA-256: `40562ec888795efc70bbb69a8cd56a2948bb71fb597158e3f11dbd28b25139ff`
- Readback SHA-256: `40562ec888795efc70bbb69a8cd56a2948bb71fb597158e3f11dbd28b25139ff`
- Readback: `dist/gausstop_communication_diagnostic/firmware.bin.readback.bin`

MASTER now contains the bridge-disabled communication diagnostic; operational
motor firmware has not been installed.

### ESP32 USB detection

After user confirmation `esp32 usb only confirmed`, the newly appearing serial
device was mapped by both device-node creation time and macOS IORegistry:

- Serial port: `/dev/cu.usbserial-0001`
- TTY peer: `/dev/tty.usbserial-0001`
- USB bridge: Silicon Labs CP2102 USB to UART Bridge Controller
- VID:PID: `10c4:ea60`
- USB serial: `0001`
- USB location: `0x01130000`
- USB address: 3

No controller UART wiring was attached during detection.

### ESP32 coordinator installation

The user authorized the automatic serial reset/upload with
`esp32 coordinator reset and upload authorized`. Exact command:

```sh
PLATFORMIO_CORE_DIR="$PWD/.platformio" \
PLATFORMIO_SETTING_ENABLE_TELEMETRY=no \
.venv/bin/pio run -e esp32_coordinator -t upload \
  --upload-port /dev/cu.usbserial-0001
```

Log: `hardware_logs/esp32-coordinator-upload-20260722T221031Z.log`

- Detected chip: ESP32-D0WD-V3 revision 3.1
- Crystal: 40 MHz
- MAC: `28:56:2f:4b:47:f0`
- Upload baud: 460800
- Bootloader write at `0x00001000`: hash verified
- Partition table write at `0x00008000`: hash verified
- Boot application metadata at `0x0000e000`: hash verified
- Coordinator application, 271,248 bytes at `0x00010000`: hash verified
- Final RTS hard reset: completed
- PlatformIO result: SUCCESS

No controller was connected to the ESP32 during upload.

### MASTER bridge-disabled UART diagnostic

User-confirmed wiring:

```text
ESP32 GND       -> MASTER GND
ESP32 GPIO17 TX -> MASTER PB7 / USART0 RX
MASTER PB6 TX   -> ESP32 GPIO35 RX
```

MASTER battery power was applied only after all three wires were confirmed.
ST-Link, its power pins, ESP32 power rails, SLAVE, and PA2/PA3 remained
disconnected. The coordinator console was opened at 115200 and only `status`
was sent; no enable or motion command was issued.

First status:

```text
enabled=0 shutdown=0 mode=drive target=0,0 ramped=0,0 ramp=10 feedback_age_ms=338 tx=7 rx=0 crc_errors=0 states=0,0 odometers=0,0 faults=0x00000000,0x00000000
```

Longer observation status:

```text
enabled=0 shutdown=0 mode=drive target=0,0 ramped=0,0 ramp=10 feedback_age_ms=1842 tx=37 rx=0 crc_errors=0 states=0,0 odometers=0,0 faults=0x00000000,0x00000000
```

USB console operation at 115200 and coordinator transmission are confirmed,
but MASTER feedback reception failed (`rx=0`, `crc_errors=0`). The exposed
PB6/PB7 route, wire mapping, or signal continuity is therefore not validated.
Opening the CP2102 serial port produced an ESP32 `POWERON_RESET` boot log despite
requesting inactive DTR/RTS; this unexpected reset did not alter the installed
image. The communication checkpoint is failed and operational MASTER firmware
must not be installed yet.

## Physical mapping, backups, flashing, and validation

Pending. No controller has been identified as master or slave. No original
flash backup exists yet. No write, readback verification, UART validation,
power cycle, safety-command test, or motor test has been performed.

Recovery commands will be added only after both immutable 64 KiB backups have
been created, independently reread, compared, and hashed.
