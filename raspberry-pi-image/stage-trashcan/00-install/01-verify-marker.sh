#!/bin/bash -e

set -euo pipefail

marker="${ROOTFS_DIR}/etc/trashcan-robot-image-validated"
build_info="${ROOTFS_DIR}/etc/trashcan-robot-build-info"

if [[ ! -f "$marker" ]]; then
  echo "Trashcan robot image validation marker was not created by the chroot validation step" >&2
  exit 1
fi
if [[ ! -f "$build_info" ]]; then
  echo "Trashcan robot image build information was not created" >&2
  exit 1
fi

grep -Fxq 'architecture=arm64' "$build_info" || {
  echo "Trashcan robot image did not validate as arm64" >&2
  exit 1
}
grep -Fxq 'release=bookworm' "$build_info" || {
  echo "Trashcan robot image did not validate as Bookworm" >&2
  exit 1
}
grep -Fxq 'first_boot_userconfig=disabled' "$build_info" || {
  echo "First-boot user configuration was not disabled" >&2
  exit 1
}
grep -Fxq 'ssh=enabled' "$build_info" || {
  echo "SSH was not validated as enabled" >&2
  exit 1
}

printf 'Validated trashcan robot image marker: %s\n' "$marker"
printf 'Validated trashcan robot image facts:\n'
cat "$build_info"
