#!/bin/bash -e

set -euo pipefail

APP=/opt/trashcan-robot/donkeycar
VENV="$APP/.venv"

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

touch /etc/trashcan-robot-image-validated
