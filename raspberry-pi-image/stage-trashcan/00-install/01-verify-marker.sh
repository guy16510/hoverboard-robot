#!/bin/bash -e

set -euo pipefail

marker="${ROOTFS_DIR}/etc/trashcan-robot-image-validated"
if [[ ! -f "$marker" ]]; then
  echo "Trashcan robot image validation marker was not created by the chroot validation step" >&2
  exit 1
fi

printf 'Validated trashcan robot image marker: %s\n' "$marker"
