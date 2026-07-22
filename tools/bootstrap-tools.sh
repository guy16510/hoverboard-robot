#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
tool_dir="$repo_dir/.toolchains"
pio_package_dir="$repo_dir/.platformio/packages"
mkdir -p "$tool_dir" "$pio_package_dir"

if [[ ! -x "$repo_dir/.venv/bin/pio" ]]; then
  echo "missing .venv; run: python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt" >&2
  exit 1
fi

if [[ ! -f "$pio_package_dir/tool-scons/.piopm" ]]; then
  mkdir -p "$pio_package_dir/tool-scons"
  cp "$repo_dir/tools/tool-packages/tool-scons/package.json" "$pio_package_dir/tool-scons/package.json"
  cp "$repo_dir/tools/tool-packages/tool-scons/scons.py" "$pio_package_dir/tool-scons/scons.py"
  cp "$repo_dir/tools/tool-packages/tool-scons/.piopm" "$pio_package_dir/tool-scons/.piopm"
fi

if [[ ! -f "$tool_dir/esptoolpy/package.json" ]]; then
  mkdir -p "$tool_dir/esptoolpy"
  cp "$repo_dir/tools/tool-packages/esptoolpy/package.json" "$tool_dir/esptoolpy/package.json"
  cp "$repo_dir/tools/tool-packages/esptoolpy/esptool.py" "$tool_dir/esptoolpy/esptool.py"
fi

host_os="$(uname -s)"
host_arch="$(uname -m)"
case "$host_os/$host_arch" in
  Darwin/arm64)
    arm_url="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
    arm_sha="c7c78ffab9bebfce91d99d3c24da6bf4b81c01e16cf551eb2ff9f25b9e0a3818"
    arm_root="arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi"
    xtensa_url="https://github.com/espressif/crosstool-NG/releases/download/esp-2021r2-patch5/xtensa-esp32-elf-gcc8_4_0-esp-2021r2-patch5-macos-arm64.tar.gz"
    xtensa_sha="b14189772d70a96813895fff7731d0f2fec0c825cfc02e002d6d91a0cc4b6b1d"
    ;;
  Linux/x86_64)
    arm_url="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz"
    arm_sha="62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823"
    arm_root="arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi"
    xtensa_url="https://github.com/espressif/crosstool-NG/releases/download/esp-2021r2-patch5/xtensa-esp32-elf-gcc8_4_0-esp-2021r2-patch5-linux-amd64.tar.gz"
    xtensa_sha="8ef14e0409c2011b41e504a30f70d3e35287313a795d1f2462ad2cd0e2052d37"
    ;;
  *)
    echo "unsupported build host: $host_os/$host_arch" >&2
    exit 1
    ;;
esac

verify_sha() {
  local expected="$1"
  local archive="$2"
  local actual
  if command -v sha256sum >/dev/null 2>&1; then
    actual="$(sha256sum "$archive" | awk '{print $1}')"
  else
    actual="$(shasum -a 256 "$archive" | awk '{print $1}')"
  fi
  [[ "$actual" == "$expected" ]] || {
    echo "checksum mismatch for $archive" >&2
    exit 1
  }
}

download_archive() {
  local url="$1"
  local destination="$2"
  curl --fail --location --retry 3 "$url" --output "$destination"
}

if [[ ! -x "$tool_dir/arm-none-eabi/bin/arm-none-eabi-gcc" ]]; then
  temp_dir="$(mktemp -d)"
  archive="$temp_dir/arm.tar.xz"
  download_archive "$arm_url" "$archive"
  verify_sha "$arm_sha" "$archive"
  tar -xJf "$archive" -C "$temp_dir"
  mv "$temp_dir/$arm_root" "$tool_dir/arm-none-eabi"
  cp "$repo_dir/tools/tool-packages/arm-package.json" "$tool_dir/arm-none-eabi/package.json"
fi

if [[ ! -x "$tool_dir/xtensa-esp32/bin/xtensa-esp32-elf-gcc" ]]; then
  temp_dir="$(mktemp -d)"
  archive="$temp_dir/xtensa.tar.gz"
  download_archive "$xtensa_url" "$archive"
  verify_sha "$xtensa_sha" "$archive"
  tar -xzf "$archive" -C "$temp_dir"
  mv "$temp_dir/xtensa-esp32-elf" "$tool_dir/xtensa-esp32"
  cp "$repo_dir/tools/tool-packages/xtensa-package.json" "$tool_dir/xtensa-esp32/package.json"
fi

echo "pinned build tools are ready"

