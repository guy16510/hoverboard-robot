#!/usr/bin/env bash
set -euo pipefail

IMAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSID="${1:-${TRASHCAN_WIFI_SSID:-}}"
PASSWORD="${2:-${TRASHCAN_WIFI_PASSWORD:-}}"
TARGET="$IMAGE_DIR/wifi.env"

if [[ -z "$SSID" || -z "$PASSWORD" ]]; then
  echo "Usage: $0 SSID PASSWORD" >&2
  exit 2
fi
if [[ "$SSID" == *$'\n'* || "$PASSWORD" == *$'\n'* ]]; then
  echo "Wi-Fi credentials may not contain newlines" >&2
  exit 2
fi

umask 077
{
  printf 'TRASHCAN_WIFI_SSID=%q\n' "$SSID"
  printf 'TRASHCAN_WIFI_PASSWORD=%q\n' "$PASSWORD"
} > "$TARGET"
chmod 0600 "$TARGET"
printf 'Created ignored Wi-Fi build config: %s\n' "$TARGET"
