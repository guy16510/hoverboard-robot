#!/bin/bash -e
set -euo pipefail

source_profile="${STAGE_DIR}/01-network/files/trashcan-wifi.nmconnection"
target_profile="${ROOTFS_DIR}/etc/NetworkManager/system-connections/trashcan-wifi.nmconnection"

if [[ ! -f "$source_profile" ]]; then
  printf 'Wi-Fi profile not supplied; image remains unprovisioned\n'
  exit 0
fi

install -D -o root -g root -m 0600 "$source_profile" "$target_profile"
printf 'Installed NetworkManager Wi-Fi profile\n'
