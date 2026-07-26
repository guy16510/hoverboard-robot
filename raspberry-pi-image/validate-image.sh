#!/usr/bin/env bash
set -euo pipefail

IMAGE="${1:-}"
[[ -n "$IMAGE" && -f "$IMAGE" ]] || {
  echo "Usage: sudo $0 path/to/trashcan-robot.img.xz" >&2
  exit 2
}

for tool in xz losetup lsblk mount umount mountpoint chroot awk grep find stat readlink; do
  command -v "$tool" >/dev/null 2>&1 || {
    echo "Missing required image validation tool: $tool" >&2
    exit 1
  }
done

work_dir="$(mktemp -d)"
raw_image="$work_dir/image.img"
root_mount="$work_dir/root"
loop_device=''
mkdir -p "$root_mount"

cleanup() {
  local status=$?
  set +e
  if mountpoint -q "$root_mount"; then
    umount "$root_mount"
  fi
  if [[ -n "$loop_device" ]]; then
    losetup -d "$loop_device"
  fi
  rm -rf "$work_dir"
  exit "$status"
}
trap cleanup EXIT INT TERM

xz --test "$IMAGE"
xz -dc -- "$IMAGE" > "$raw_image"
loop_device="$(losetup --find --show --partscan --read-only "$raw_image")"

root_partition=''
for _ in {1..20}; do
  root_partition="$(lsblk -lnpo NAME,FSTYPE "$loop_device" | awk '$2 == "ext4" { print $1; exit }')"
  [[ -n "$root_partition" ]] && break
  sleep 1
done
[[ -n "$root_partition" ]] || {
  echo "Could not locate the ext4 root partition in $IMAGE" >&2
  lsblk "$loop_device" >&2 || true
  exit 1
}
mount -o ro "$root_partition" "$root_mount"

marker="$root_mount/etc/trashcan-robot-image-validated"
build_info="$root_mount/etc/trashcan-robot-build-info"
app=/opt/trashcan-robot/donkeycar
mounted_app="$root_mount$app"
service="$root_mount/etc/systemd/system/trashcan-donkeycar.service"
enabled_link="$root_mount/etc/systemd/system/multi-user.target.wants/trashcan-donkeycar.service"

[[ -f "$marker" && -f "$build_info" ]] || {
  echo "Exported image is missing its validation marker or build information" >&2
  exit 1
}
for fact in \
  'architecture=arm64' \
  'release=bookworm' \
  'robot_user=trashbot' \
  'robot_group=trashbot' \
  'robot_working_directory=/opt/trashcan-robot/donkeycar' \
  'robot_service=enabled' \
  'robot_service_contract=validated' \
  'python_smoke=passed' \
  'application_tests=passed' \
  'systemd_verify=passed' \
  'dependency_check=passed' \
  'repository_owner=trashbot:trashbot'; do
  grep -Fxq "$fact" "$build_info" || {
    echo "Exported image is missing validated fact: $fact" >&2
    exit 1
  }
done

[[ -x "$mounted_app/.venv/bin/python" ]] || {
  echo "Exported image has no usable Donkeycar virtual environment" >&2
  exit 1
}
bash "$mounted_app/scripts/validate-service-unit.sh" \
  "$service" trashbot trashbot "$app"
cmp -s "$mounted_app/systemd/trashcan-donkeycar.service" "$service" || {
  echo "Exported service unit differs from the canonical unit" >&2
  exit 1
}

[[ -L "$enabled_link" ]] || {
  echo "Exported image has the robot service disabled" >&2
  exit 1
}
link_target="$(readlink "$enabled_link")"
if [[ "$link_target" != '/etc/systemd/system/trashcan-donkeycar.service' && \
      "$link_target" != '../trashcan-donkeycar.service' ]]; then
  echo "Unexpected exported service enablement target: $link_target" >&2
  exit 1
fi

trashbot_uid="$(awk -F: '$1 == "trashbot" { print $3 }' "$root_mount/etc/passwd")"
trashbot_gid="$(awk -F: '$1 == "trashbot" { print $3 }' "$root_mount/etc/group")"
[[ -n "$trashbot_uid" && -n "$trashbot_gid" ]] || {
  echo "Exported image is missing the trashbot user or group" >&2
  exit 1
}
bad_owner="$(find "$root_mount/opt/trashcan-robot" \( ! -uid "$trashbot_uid" -o ! -gid "$trashbot_gid" \) -print -quit)"
if [[ -n "$bad_owner" ]]; then
  echo "Exported repository ownership is incorrect at $bad_owner" >&2
  stat -c '%u:%g %n' "$bad_owner" >&2 || true
  exit 1
fi

chroot "$root_mount" /usr/bin/env PYTHONPATH="$app" \
  "$app/.venv/bin/python" - <<'PY'
import flask
import psutil
import serial
import yaml
import donkeycar
from trashcan_robot.config import load_config
from trashcan_robot.protocol import crc16_ccitt_false
cfg = load_config('/opt/trashcan-robot/donkeycar/config/robot.yaml')
assert cfg.serial.baud == 115200
assert crc16_ccitt_false(b'123456789') == 0x29B1
print('Exported image Python smoke test passed')
PY
chroot "$root_mount" "$app/.venv/bin/python" -m pip check

if grep -R -Fq '/opt/hoverboard-robot' "$root_mount/etc/systemd/system"; then
  echo "Exported image contains a forbidden legacy service path" >&2
  exit 1
fi

printf 'Exported Raspberry Pi image contract validated: %s\n' "$IMAGE"
