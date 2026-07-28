#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
APP="$ROOT/donkeycar"
VENV="${VENV:-$APP/.venv}"
PYTHON_BIN="${PYTHON_BIN:-python3.11}"

sudo apt-get update
sudo apt-get install -y \
  python3.11 python3.11-venv python3.11-dev \
  build-essential libcamera-dev python3-picamera2 joystick \
  nodejs npm

command -v "$PYTHON_BIN" >/dev/null 2>&1 || {
  echo "Donkeycar 5.3 requires Python 3.11. Set PYTHON_BIN to a Python 3.11 executable." >&2
  exit 1
}

"$PYTHON_BIN" - <<'PY'
import sys
if sys.version_info[:2] != (3, 11):
    raise SystemExit(f"Python 3.11 required, got {sys.version.split()[0]}")
PY

command -v node >/dev/null 2>&1 || {
  echo "Node.js installation failed" >&2
  exit 1
}
command -v npm >/dev/null 2>&1 || {
  echo "npm installation failed" >&2
  exit 1
}
node_version="$(node --version)"
node_major="${node_version#v}"
node_major="${node_major%%.*}"
if ! [[ "$node_major" =~ ^[0-9]+$ ]] || (( node_major < 18 )); then
  echo "Node.js 18 or newer is required, got $node_version" >&2
  exit 1
fi
node --test "$ROOT/tools/pi-client/test/"*.test.mjs

"$PYTHON_BIN" -m venv --system-site-packages "$VENV"
"$VENV/bin/python" -m pip install --upgrade pip wheel
"$VENV/bin/pip" install -r "$APP/requirements.txt"
mkdir -p "$APP/data/tubs" "$APP/data/logs" "$APP/models"
printf 'Installed. Activate with: source %s/bin/activate\n' "$VENV"
printf 'Node.js %s and npm %s installed.\n' "$node_version" "$(npm --version)"
printf 'Add your user to dialout if needed: sudo usermod -aG dialout %s\n' "$USER"
