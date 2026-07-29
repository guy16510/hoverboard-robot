#!/usr/bin/env bash
set -euo pipefail

UNIT_PATH="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/systemd/trashcan-donkeycar.service}"
EXPECTED_USER="${2:-trashbot}"
EXPECTED_GROUP="${3:-trashbot}"
EXPECTED_APP="${4:-/opt/trashcan-robot/donkeycar}"

[[ -f "$UNIT_PATH" ]] || {
  echo "Missing systemd unit: $UNIT_PATH" >&2
  exit 1
}

assert_single_directive() {
  local key="$1"
  local expected_value="$2"
  local expected_line="${key}=${expected_value}"
  local -a matches=()

  mapfile -t matches < <(grep -E "^[[:space:]]*${key}=" "$UNIT_PATH" || true)
  if (( ${#matches[@]} != 1 )); then
    echo "Expected exactly one $key directive in $UNIT_PATH, found ${#matches[@]}" >&2
    exit 1
  fi
  if [[ "${matches[0]}" != "$expected_line" ]]; then
    echo "Invalid $key directive in $UNIT_PATH" >&2
    echo "Expected: $expected_line" >&2
    echo "Actual:   ${matches[0]}" >&2
    exit 1
  fi
}

assert_single_directive User "$EXPECTED_USER"
assert_single_directive Group "$EXPECTED_GROUP"
assert_single_directive SupplementaryGroups "dialout video render gpio i2c input"
assert_single_directive WorkingDirectory "$EXPECTED_APP"
assert_single_directive ExecStart "$EXPECTED_APP/.venv/bin/python $EXPECTED_APP/manage.py drive --config $EXPECTED_APP/config/robot.yaml"
assert_single_directive EnvironmentFile "-/etc/default/trashcan-donkeycar"
assert_single_directive Restart "always"

if grep -Fq '/opt/hoverboard-robot' "$UNIT_PATH"; then
  echo "Legacy /opt/hoverboard-robot path is forbidden in $UNIT_PATH" >&2
  exit 1
fi
if grep -Eq 'dev-tty|\.device' "$UNIT_PATH"; then
  echo "Serial device unit dependencies are forbidden in $UNIT_PATH" >&2
  exit 1
fi
if ! grep -Fxq 'After=network-online.target' "$UNIT_PATH"; then
  echo "Service must start after network-online.target only" >&2
  exit 1
fi

printf 'Validated service contract: user=%s group=%s app=%s unit=%s\n' \
  "$EXPECTED_USER" "$EXPECTED_GROUP" "$EXPECTED_APP" "$UNIT_PATH"
