#!/bin/bash -e

set -euo pipefail

APP=/opt/trashcan-robot/donkeycar
VENV="$APP/.venv"

architecture="$(dpkg --print-architecture)"
if [[ "$architecture" != "arm64" ]]; then
  echo "Raspberry Pi image must be arm64, got $architecture" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "bookworm" ]]; then
  echo "Raspberry Pi image must be Bookworm, got ${VERSION_CODENAME:-unknown}" >&2
  exit 1
fi

id pi >/dev/null 2>&1 || {
  echo "Expected preconfigured first user 'pi' is missing" >&2
  exit 1
}

systemctl is-enabled --quiet ssh.service || {
  echo "SSH must be enabled in the generated image" >&2
  exit 1
}

userconfig_state="$(systemctl is-enabled userconfig.service 2>/dev/null || true)"
case "$userconfig_state" in
  enabled|enabled-runtime|linked|linked-runtime|alias)
    echo "userconfig.service must not run on this unattended image, state is $userconfig_state" >&2
    exit 1
    ;;
esac

printf 'TRASHCAN_IMAGE_ARCHITECTURE=%s\n' "$architecture"
printf 'TRASHCAN_IMAGE_RELEASE=%s\n' "$VERSION_CODENAME"
printf 'TRASHCAN_FIRST_BOOT_USERCONFIG=disabled\n'
printf 'TRASHCAN_SSH=enabled\n'

getent group trashbot >/dev/null || groupadd --system trashbot
id trashbot >/dev/null 2>&1 || useradd --system --create-home --gid trashbot --shell /usr/sbin/nologin trashbot
for group in dialout video render gpio i2c input; do
  getent group "$group" >/dev/null && usermod -aG "$group" trashbot || true
done

python3 - <<'PY'
import sys
if sys.version_info[:2] != (3, 11):
    raise SystemExit(f"Raspberry Pi OS Bookworm Python 3.11 required, got {sys.version}")
PY

python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/python" -m pip install --upgrade 'pip<25' wheel setuptools
"$VENV/bin/pip" install --only-binary=:all: -r "$APP/requirements.txt"

install -d -o trashbot -g trashbot -m 0755 "$APP/data/tubs" "$APP/data/logs" "$APP/models"
chown -R trashbot:trashbot /opt/trashcan-robot

sed -i \
  -e 's|User=pi|User=trashbot|' \
  -e 's|Group=pi|Group=trashbot|' \
  -e 's|SupplementaryGroups=dialout video|SupplementaryGroups=dialout video render gpio i2c input|' \
  -e 's|/opt/hoverboard-robot/donkeycar|/opt/trashcan-robot/donkeycar|g' \
  /etc/systemd/system/trashcan-donkeycar.service
systemctl enable trashcan-donkeycar.service

PYTHONPATH="$APP" "$VENV/bin/python" - <<'PY'
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
print('Raspberry Pi image Python smoke test passed')
PY

PYTHONPATH="$APP" "$VENV/bin/python" -m pytest -q "$APP/tests"
systemd-analyze verify /etc/systemd/system/trashcan-donkeycar.service

cat > /etc/trashcan-robot-build-info <<EOF
architecture=$architecture
release=$VERSION_CODENAME
first_user=pi
first_boot_userconfig=disabled
ssh=enabled
cloud_init=disabled
robot_service=enabled
EOF

touch /etc/trashcan-robot-image-validated
