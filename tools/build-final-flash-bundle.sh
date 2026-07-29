#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"

pio="${PIO:-$repo_dir/.venv/bin/pio}"
bundle="${BUNDLE_DIR:-$repo_dir/final-flash-bundle}"
source_sha="${SOURCE_SHA:-$(git rev-parse HEAD)}"
staging="$(mktemp -d "${TMPDIR:-/tmp}/hoverboard-final-flash.XXXXXX")"
trap 'rm -rf "$staging"' EXIT

export PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-$repo_dir/.platformio}"
export PLATFORMIO_SETTING_ENABLE_TELEMETRY=no

require_file() {
  local path="$1"
  if [[ ! -s "$path" ]]; then
    echo "required build artifact missing or empty: $path" >&2
    exit 1
  fi
}

copy_first_build() {
  local source="$1"
  local destination="$2"
  require_file "$source"
  mkdir -p "$(dirname "$destination")"
  cp "$source" "$destination"
}

compare_rebuild() {
  local expected="$1"
  local actual="$2"
  require_file "$expected"
  require_file "$actual"
  cmp "$expected" "$actual"
}

rm -rf "$bundle"
mkdir -p \
  "$staging/gausstop_slave" \
  "$staging/gausstop_master_swd" \
  "$staging/esp32_drive_coordinator"

# Build and stage each target immediately. PlatformIO may remove build folders
# when switching between platformio.ini and platformio-drive.ini.
rm -rf .pio/build/gausstop_slave
"$pio" run -e gausstop_slave 2>&1 | tee gausstop-slave-build.log
copy_first_build .pio/build/gausstop_slave/firmware.bin \
  "$staging/gausstop_slave/firmware.bin"
copy_first_build .pio/build/gausstop_slave/firmware.elf \
  "$staging/gausstop_slave/firmware.elf"

rm -rf .pio/build/gausstop_master_swd
"$pio" run -e gausstop_master_swd 2>&1 | tee gausstop-master-swd-build.log
copy_first_build .pio/build/gausstop_master_swd/firmware.bin \
  "$staging/gausstop_master_swd/firmware.bin"
copy_first_build .pio/build/gausstop_master_swd/firmware.elf \
  "$staging/gausstop_master_swd/firmware.elf"

rm -rf .pio/build/esp32_drive_coordinator
"$pio" run -c platformio-drive.ini -e esp32_drive_coordinator \
  2>&1 | tee esp32-drive-build.log
for file in firmware.bin firmware.elf bootloader.bin partitions.bin boot_app0.bin; do
  copy_first_build ".pio/build/esp32_drive_coordinator/$file" \
    "$staging/esp32_drive_coordinator/$file"
done

# Rebuild every flashable image from an empty target directory and require
# byte-identical output before publishing the first build.
rm -rf .pio/build/gausstop_slave
"$pio" run -e gausstop_slave
compare_rebuild "$staging/gausstop_slave/firmware.bin" \
  .pio/build/gausstop_slave/firmware.bin

rm -rf .pio/build/gausstop_master_swd
"$pio" run -e gausstop_master_swd
compare_rebuild "$staging/gausstop_master_swd/firmware.bin" \
  .pio/build/gausstop_master_swd/firmware.bin

rm -rf .pio/build/esp32_drive_coordinator
"$pio" run -c platformio-drive.ini -e esp32_drive_coordinator
for file in firmware.bin bootloader.bin partitions.bin boot_app0.bin; do
  compare_rebuild "$staging/esp32_drive_coordinator/$file" \
    ".pio/build/esp32_drive_coordinator/$file"
done

mkdir -p "$bundle"
cp -R "$staging/gausstop_slave" "$bundle/"
cp -R "$staging/gausstop_master_swd" "$bundle/"
cp -R "$staging/esp32_drive_coordinator" "$bundle/"

cat > "$bundle/MANIFEST.txt" <<EOF
repository=guy16510/hoverboard-robot
commit=$source_sha
protocol_version=4
command_marker=0x42
slave_feedback_marker=0x43

These three firmware directories are one indivisible matched set.
Mixed protocol epochs fail closed and cannot arm.
Every flashable binary was built twice from clean target directories and compared byte-for-byte.
EOF

cat > "$bundle/esp32_drive_coordinator/FLASH_LAYOUT.txt" <<'EOF'
ESP32 flash offsets for the esp32dev production image:
0x1000  bootloader.bin
0x8000  partitions.bin
0xE000  boot_app0.bin
0x10000 firmware.bin
Preserve NVS unless an explicit recovery procedure requires erasing it.
EOF

(
  cd "$bundle"
  if command -v sha256sum >/dev/null 2>&1; then
    find . -type f ! -name SHA256SUMS -print0 \
      | sort -z \
      | xargs -0 sha256sum > SHA256SUMS
    sha256sum -c SHA256SUMS
  else
    find . -type f ! -name SHA256SUMS -print0 \
      | sort -z \
      | xargs -0 shasum -a 256 > SHA256SUMS
    shasum -a 256 -c SHA256SUMS
  fi
)

echo "Final matched flash bundle created at $bundle"
