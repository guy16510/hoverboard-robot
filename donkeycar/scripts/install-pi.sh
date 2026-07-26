#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
APP="$ROOT/donkeycar"
VENV="${VENV:-$APP/.venv}"

sudo apt-get update
sudo apt-get install -y python3-venv python3-dev build-essential libcamera-dev python3-picamera2 joystick
python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/python" -m pip install --upgrade pip wheel
"$VENV/bin/pip" install -r "$APP/requirements.txt"
mkdir -p "$APP/data/tubs" "$APP/data/logs" "$APP/models"
printf 'Installed. Activate with: source %s/bin/activate\n' "$VENV"
printf 'Add your user to dialout if needed: sudo usermod -aG dialout $USER\n'
