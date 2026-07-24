# Hardware validation attempt — 2026-07-23

## Scope

- Repository: `guy16510/hoverboard-robot`
- Branch: `codex/swd-dual-motor-recovery`
- Commit: `6fdc809860f65db66af531fb94043fd147bf95da`
- ESP32 serial port: `/dev/cu.usbserial-0001`
- Required transport: `pulse-rmt-v1`

## Images and readback results

All three canonical artifacts came from the successful GitHub Actions build for
the exact commit above. Each flashed region was dumped at its exact image length
and compared byte-for-byte.

| Target | SHA-256 | Result |
| --- | --- | --- |
| `gausstop_slave` | `19e793123bb7c756aad1c400d6dd3e6ec992d708563bbde128895b3011cb4f3a` | Programmed once; OpenOCD verify passed; readback matched |
| `bench_master_pa4_bypass` | `027835fb9c26fd3f297a82e5117f7a59f73c1f4d62ed8f264986f42e6d679def` | Programmed once; OpenOCD verify passed; readback matched |
| `esp32_swd_coordinator` application | `6f7bca55a62a3ff40b908a992472f855ca30be6255bb3bbe49063f946d008fde` | Programmed once at `0x10000`; 280,240-byte readback matched |

The existing ESP32 bootloader had a valid image checksum/hash, and its partition
table parsed successfully with `app0` at `0x10000`; both were preserved.

## Communication tests

Two zero-demand tests were run with `tools/drive_esp32.py`. No nonzero motor
command was sent.

The initial gate began before feedback takeover had settled. It showed:

- `command_transport="pulse-rmt-v1"`
- ESP32 TX `23 -> 32`
- ESP32 RX `0 -> 1`
- ESP32 CRC errors `11 -> 16`
- MASTER remote parser bytes `0 -> 13321`
- MASTER valid frames `0 -> 2`
- MASTER framing errors `0 -> 3`
- `transport_overflows=[0,0,1,0]`
- `peer_healthy=0`
- Both bridges, compares, applied commands, and odometers remained zero

The settled gate showed:

- ESP32 TX `23 -> 32`
- ESP32 RX `3 -> 6`
- ESP32 CRC errors `11 -> 16`
- MASTER remote parser bytes `21747 -> 21890`
- MASTER valid frames `4 -> 6`
- MASTER framing errors remained `6`
- MASTER fault `8192` (`GS_FAULT_TRANSPORT_OVERFLOW`)
- `transport_overflows=[0,0,1,0]`, identifying link RX overflow
- `peer_healthy=0`
- Hall states `[2,4]`, both valid
- PA4 bypass active and SLAVE PA4 high
- Both bridges, compares, applied commands, and odometers remained zero

A subsequent 30-snapshot capture opened serial with DTR/RTS inactive to avoid
resetting the ESP32. The failure persisted independently of serial reset:

- ESP32 TX `14 -> 109`
- ESP32 RX `2 -> 26`
- ESP32 CRC errors `5 -> 44`
- Feedback age repeatedly exceeded 250 ms, reaching 668 ms
- MASTER remote parser bytes `25762 -> 26807`
- MASTER valid-frame count remained stuck at `2`
- `peer_healthy` remained false
- MASTER remained faulted with `8192`

## Code path and outcome

`firmware/gd32/master/main.c::service_transport_health()` observes the
`GS_UART_LINK` RX overflow, records
`GS_FEEDBACK_TRANSPORT_LINK_RX_OVERFLOW`, and calls
`force_master_fault(GS_FAULT_TRANSPORT_OVERFLOW)`. The exact firmware therefore
rejects motion by design while communication is corrupt and peer health is
false.

The communication gate did not pass, so left/right/both motion tests were not
run. Every test path sent `stop` and `disable`; final telemetry confirmed
requested, ramped, applied, compare, and bridge values were all zero.

