#!/usr/bin/env bash
set -euo pipefail

IMAGE="${1:-}"
[[ -n "$IMAGE" && -f "$IMAGE" ]] || {
  echo "Usage: sudo $0 path/to/trashcan-robot.img.xz" >&2
  exit 2
}

for tool in xz losetup lsblk mount umount mountpoint chroot awk grep find stat readlink cmp; do
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
trap cleanup EXIT

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
repository=/opt/trashcan-robot
app="$repository/donkeycar"
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
  'nodejs=installed' \
  'nodejs_minimum_major=18' \
  'nodejs_smoke=passed' \
  'npm=installed' \
  'node_protocol_tests=passed' \
  'robot_user=trashbot' \
  'robot_group=trashbot' \
  'robot_working_directory=/opt/trashcan-robot/donkeycar' \
  'robot_service=enabled' \
  'robot_service_contract=validated' \
  'python_smoke=passed' \
  'opencv_apriltag=passed' \
  'backup_camera_config=validated' \
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
bad_owner="$(find "$root_mount$repository" \( ! -uid "$trashbot_uid" -o ! -gid "$trashbot_gid" \) -print -quit)"
if [[ -n "$bad_owner" ]]; then
  echo "Exported repository ownership is incorrect at $bad_owner" >&2
  stat -c '%u:%g %n' "$bad_owner" >&2 || true
  exit 1
fi

node_version="$(chroot "$root_mount" /usr/bin/node --version)"
node_major="${node_version#v}"
node_major="${node_major%%.*}"
if ! [[ "$node_major" =~ ^[0-9]+$ ]] || (( node_major < 18 )); then
  echo "Exported image requires Node.js 18 or newer, got $node_version" >&2
  exit 1
fi
npm_version="$(chroot "$root_mount" /usr/bin/npm --version)"
chroot "$root_mount" /usr/bin/node --input-type=module - <<'JS'
import assert from 'node:assert/strict';
import {
  MessageType,
  crc16,
  encodeFrame,
} from 'file:///opt/trashcan-robot/tools/pi-client/protocol.mjs';

assert.equal(crc16(Buffer.from('123456789')), 0x29b1);
const frame = encodeFrame({ type: MessageType.DISARM, sequence: 7 });
assert.equal(frame[0], 0xa5);
assert.equal(frame[1], 0x5a);
console.log('Exported image Node.js smoke test passed');
JS
printf 'Exported image Node.js validated: node %s, npm %s\n' "$node_version" "$npm_version"

chroot "$root_mount" /usr/bin/env \
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$app" \
  "$app/.venv/bin/python" - <<'PY'
import cv2
import flask
import psutil
import serial
import yaml
import donkeycar
from inspect import signature
from pathlib import Path
from donkeycar.parts.tub_v2 import TubWriter
from trashcan_robot.config import load_config
from trashcan_robot.pipeline import create_tub_writer
from trashcan_robot.protocol import crc16_ccitt_false

if not hasattr(cv2, 'aruco'):
    raise SystemExit('Exported image is missing cv2.aruco')
if not hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
    raise SystemExit('Exported image is missing AprilTag 36h11 support')
cfg = load_config('/opt/trashcan-robot/donkeycar/config/robot.yaml')
assert cfg.serial.baud == 115200
assert cfg.raw['backup_camera']['enabled'] is True
assert cfg.raw['backup_camera']['family'] == '36h11'
assert crc16_ccitt_false(b'123456789') == 0x29B1
assert 'base_path' in signature(TubWriter).parameters

class TubWriterContract:
    def __init__(self, *, base_path, inputs, types):
        self.base_path = base_path

writer = create_tub_writer(
    TubWriterContract,
    Path('/tmp/tub-contract'),
    ['cam/image_array'],
    ['image_array'],
)
assert writer.base_path == '/tmp/tub-contract'
print('Exported image OpenCV AprilTag smoke test passed')
print('Exported image Python smoke test passed')
PY
chroot "$root_mount" /usr/bin/env PYTHONDONTWRITEBYTECODE=1 \
  "$app/.venv/bin/python" -m pip check

if grep -R -Fq '/opt/hoverboard-robot' "$root_mount/etc/systemd/system"; then
  echo "Exported image contains a forbidden legacy service path" >&2
  exit 1
fi

printf 'Exported Raspberry Pi image contract validated: %s\n' "$IMAGE"
