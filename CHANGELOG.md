# Changelog

## Unreleased

- Created a clean, local-only GAUSSTOP one-ESP32 dual-motor firmware project.
- Pinned both public upstream commits, the GD32/ESP32 platforms, SPL, compilers,
  PlatformIO, and formatting tools.
- Added separate protocol, parser, wheel-mixing, commutation, motor-state,
  safety, master/slave coordination, transport, and console modules with
  fixed storage and explicit ownership.
- Added master, slave, bridge-disabled diagnostic, safe-recovery, ESP32
  coordinator, passive-probe, and native-test PlatformIO targets.
- Added explicit GAUSSTOP DPHC-V3.3 pin assertions and a 64 KiB flash / 8 KiB
  SRAM linker layout with a 7 KiB static-RAM ceiling.
- Added 270 host assertions, ASan/UBSan runs, role-symbol checks, formatting,
  licensing, provenance, source-classification, memory, and hardware-isolation
  checks.
- Generated BIN/ELF/map artifacts and manifests locally. Two clean builds from
  the same commit produced identical hashes for every firmware, ESP32
  bootloader, and ESP32 partition binary.
- Added staged authorized-hardware procedures without performing flashing,
  serial inspection, USB enumeration, electrical measurement, or motor tests.
