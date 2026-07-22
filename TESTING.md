# Build and test guide

No command in the automatic build/test path enumerates USB, opens serial ports,
uploads, flashes, erases, or starts a debugger. Hardware helpers are separate,
manual-only scripts and are never called by CI. **Statically validated**

## Bootstrap

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
./tools/bootstrap-tools.sh
```

The bootstrap selects one of two checksum-pinned host bundles: macOS arm64 or
Linux x86_64. It installs Arm GNU 14.2.1, Espressif Xtensa 8.4.0
2021r2-patch5, SCons 4.8.1, and the offline esptool 4.11 image converter in
ignored local directories. PlatformIO, GD32 platform/SPL, ESP32 platform,
Arduino-ESP32, and clang-format pins are in `requirements-dev.txt`,
`platformio.ini`, and `UPSTREAM.md`. **Build validated**

## Commands

```sh
./tools/test-all.sh
./tools/build-all.sh
./tools/static-checks.sh
```

`test-all.sh` runs strict native C tests, ASan/UBSan, PlatformIO native build,
and the resulting program. `build-all.sh` builds six firmware roles, verifies
limits and expected artifacts, and creates ignored `dist/` manifests.
`static-checks.sh` checks format, source classification, SPDX/license coverage,
hardware isolation, shell syntax, pin/break declarations, role source, and ELF
symbols. **Statically validated**

The ESP32 build invokes esptool only with offline image-conversion operations
to create bootloader/application images from build inputs. No serial port or
device command is supplied. **Build validated**

## Reproducibility

`SOURCE_DATE_EPOCH` is set from the current Git commit. A final check rebuilds
from clean generated directories twice and compares firmware BIN hashes. Tool
caches, `.pio/`, `.venv/`, `.toolchains/`, and `dist/` are ignored. Generated
manifests record dirty state so uncommitted-source artifacts are evident.

Remote GitHub Actions have not been run for this local-only repository. Workflow
syntax and every underlying command are validated locally. **Statically validated**

