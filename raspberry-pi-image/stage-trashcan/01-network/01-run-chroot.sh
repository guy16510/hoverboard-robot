#!/bin/bash -e
set -euo pipefail

profile=/etc/NetworkManager/system-connections/trashcan-wifi.nmconnection
marker=/etc/trashcan-wifi-provisioned
rm -f "$marker"

if [[ ! -f "$profile" ]]; then
  printf 'Wi-Fi profile not present; first boot will require provisioning\n'
  exit 0
fi

[[ "$(stat -c '%a' "$profile")" == "600" ]] || {
  echo "Wi-Fi profile must have mode 0600" >&2
  exit 1
}
[[ "$(stat -c '%U:%G' "$profile")" == "root:root" ]] || {
  echo "Wi-Fi profile must be owned by root:root" >&2
  exit 1
}

grep -Fxq 'type=wifi' "$profile"
grep -Fxq 'autoconnect=true' "$profile"
grep -Fxq 'key-mgmt=wpa-psk' "$profile"
grep -Eq '^ssid=.+$' "$profile"
grep -Eq '^psk=.{8,}$' "$profile"

systemctl enable NetworkManager.service
systemctl is-enabled --quiet NetworkManager.service

touch "$marker"
printf 'NetworkManager Wi-Fi profile validated\n'
