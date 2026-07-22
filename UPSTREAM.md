# Upstream provenance

This repository is a clean implementation informed by two pinned GPLv3
upstreams. Builds are self-contained and never fetch source code.

| Purpose | Repository | Commit |
|---|---|---|
| Primary GD32 implementation | `RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32` | `fa089568a523d8a43887bf8925b0ddb077ebb6ae` |
| Board-layout reference | `RoboDurden/Hoverboard-Firmware-Hack-Gen2.x` | `eefe281b25086c55ff35b0322dd1de2c31965652` |

Pinned build dependencies:

| Tool | Pin |
|---|---|
| PlatformIO Core | `6.1.19` |
| Community GD32 PlatformIO platform | `b80521168db1afc6e59cab59ce0685014e0a8111` |
| GD32 SPL package | `8849cb2af35f16431734d9e9c101de648f54f061` |
| Espressif 32 PlatformIO platform | `6.13.0` |
| clang-format | `22.1.8` |

## Source strategy

The primary upstream is authoritative for GD32F130 peripheral setup,
six-step BLDC control, ADC/timer initialization, serial handling, and
master/slave concepts. The following upstream files were reviewed:

- `HoverBoardGigaDevice/Inc/defines/defines_2-1-13.h`
- `HoverBoardGigaDevice/Src/setup.c`
- `HoverBoardGigaDevice/Src/bldcBC.c`
- `HoverBoardGigaDevice/Src/bldc.c`
- `HoverBoardGigaDevice/Src/comms.c`
- `HoverBoardGigaDevice/Src/commsMasterSlave.c`
- `HoverBoardGigaDevice/Src/remoteUart.c`

The new modules preserve upstream copyright and license obligations while
using explicit GAUSSTOP mappings, bounded parsers, explicit serialization,
and testable dependency boundaries. No upstream source file is copied
verbatim. Relevant implementation files are classified `Modified upstream`
where they adapt those concepts; new protocol and ESP32 files use their
corresponding classifications.

The companion layout repository is documentation evidence only. Layout 13
was the closest historic reference, but is not claimed to match GAUSSTOP:
the **Historically physically verified** Hall mapping differs.

`tools/audit-upstream.sh` optionally downloads the exact commits into an
ignored cache for human comparison. It is never called by builds, tests, or
CI and requires explicit invocation and network access.

