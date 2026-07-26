#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
APP="$ROOT/donkeycar"
SERVICE_SOURCE="$APP/systemd/trashcan-donkeycar.service"
SERVICE_TARGET="/etc/systemd/system/trashcan-donkeycar.service"
RUN_USER="${SUDO_USER:-$USER}"
RUN_GROUP="$(id -gn "$RUN_USER")"

[[ -x "$APP/.venv/bin/python" ]] || {
  echo "Missing $APP/.venv. Run scripts/install-pi.sh first." >&2
  exit 1
}

sed \
  -e "s|User=pi|User=$RUN_USER|" \
  -e "s|Group=pi|Group=$RUN_GROUP|" \
  -e "s|/opt/hoverboard-robot/donkeycar|$APP|g" \
  "$SERVICE_SOURCE" | sudo tee "$SERVICE_TARGET" >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable trashcan-donkeycar.service
printf 'Installed %s. Run preflight before starting it.\n' "$SERVICE_TARGET"
printf 'Start with: sudo systemctl start trashcan-donkeycar.service\n'
