#!/bin/bash -e

set -euo pipefail

APP=/opt/trashcan-robot/donkeycar
VENV="$APP/.venv"
SERVICE_SOURCE="$APP/systemd/trashcan-donkeycar.service"
SERVICE_TARGET=/etc/systemd/system/trashcan-donkeycar.service
MARKER=/etc/trashcan-robot-image-validated
BUILD_INFO=/etc/trashcan-robot-build-info

rm -f "$MARKER" "$BUILD_INFO"

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
# Prefer wheels where available, but allow source builds. RPi.GPIO does not
# provide a compatible ARM64 wheel for this dependency set.
"$VENV/bin/python" -m pip install --prefer-binary -r "$APP/requirements.txt"
"$VENV/bin/python" -m pip check
printf 'Raspberry Pi image dependency installation passed\n'

install -d -o trashbot -g trashbot -m 0755 "$APP/data/tubs" "$APP/data/logs" "$APP/models"

install -d -m 0755 /etc/systemd/system
install -m 0644 "$SERVICE_SOURCE" "$SERVICE_TARGET"
cmp -s "$SERVICE_SOURCE" "$SERVICE_TARGET" || {
  echo "Installed service unit differs from canonical repository unit" >&2
  exit 1
}
bash "$APP/scripts/validate-service-unit.sh" "$SERVICE_TARGET" trashbot trashbot "$APP"
systemctl enable trashcan-donkeycar.service
systemctl is-enabled --quiet trashcan-donkeycar.service || {
  echo "trashcan-donkeycar.service was not enabled" >&2
  exit 1
}
printf 'Trashcan robot service contract validated\n'

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$APP" "$VENV/bin/python" - <<'PY'
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
cfg = load_config('/opt/trashcan-robot/donkeycar/config/robot.yaml')
assert cfg.serial.baud == 115200
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
print('Raspberry Pi image Python smoke test passed')
PY

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$APP" "$VENV/bin/python" -m pytest -q -p no:cacheprovider "$APP/tests"
printf 'Raspberry Pi image application tests passed\n'
systemd-analyze verify "$SERVICE_TARGET"
printf 'Raspberry Pi image systemd verification passed\n'

# Python imports and pytest can create cache files even after an earlier chown.
# Make ownership the final mutation before validating and writing the marker.
chown -R trashbot:trashbot /opt/trashcan-robot
bad_owner="$(find /opt/trashcan-robot \( ! -user trashbot -o ! -group trashbot \) -print -quit)"
if [[ -n "$bad_owner" ]]; then
  echo "Repository ownership validation failed at $bad_owner" >&2
  stat -c '%U:%G %n' "$bad_owner" >&2 || true
  exit 1
fi
printf 'Raspberry Pi image repository ownership validated\n'

cat > "$BUILD_INFO" <<EOF_INFO
architecture=$architecture
release=$VERSION_CODENAME
first_user=pi
first_boot_userconfig=disabled
ssh=enabled
cloud_init=disabled
robot_user=trashbot
robot_group=trashbot
robot_working_directory=$APP
robot_service=enabled
robot_service_contract=validated
python_smoke=passed
application_tests=passed
systemd_verify=passed
dependency_check=passed
repository_owner=trashbot:trashbot
EOF_INFO

touch "$MARKER"
sync
