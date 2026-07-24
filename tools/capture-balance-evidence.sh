#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
port=""
firmware=""
output=""
duration="120"
poll_ms="500"
stage="mpu-diagnostic"
label=""
command_plan=""
query="1"

usage() {
  echo "usage: tools/capture-balance-evidence.sh --port DEVICE --firmware FILE [--output DIRECTORY] [--duration SECONDS] [--poll-ms MILLISECONDS] [--stage NAME] [--label TEXT] [--command-plan FILE] [--no-query]" >&2
}

while (($# > 0)); do
  case "$1" in
  --port | --firmware | --output | --duration | --poll-ms | --stage | --label | --command-plan)
    if (($# < 2)); then
      usage
      exit 2
    fi
    option="$1"
    value="$2"
    shift 2
    case "$option" in
    --port) port="$value" ;;
    --firmware) firmware="$value" ;;
    --output) output="$value" ;;
    --duration) duration="$value" ;;
    --poll-ms) poll_ms="$value" ;;
    --stage) stage="$value" ;;
    --label) label="$value" ;;
    --command-plan) command_plan="$value" ;;
    esac
    ;;
  --no-query)
    query="0"
    shift
    ;;
  --help)
    usage
    exit 0
    ;;
  *)
    usage
    exit 2
    ;;
  esac
done

if [[ -z "$port" || -z "$firmware" ]]; then
  usage
  exit 2
fi
if [[ ! -f "$firmware" ]]; then
  echo "firmware file not found: $firmware" >&2
  exit 2
fi
if ! command -v node >/dev/null 2>&1; then
  echo "Node.js is required" >&2
  exit 2
fi
if ! command -v tar >/dev/null 2>&1; then
  echo "tar is required" >&2
  exit 2
fi

if [[ -z "$output" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  output="$PWD/balance-evidence-${stage}-${timestamp}"
fi
archive="${output}.tar.gz"
checksum="${archive}.sha256"
if [[ -e "$output" || -e "$archive" || -e "$checksum" ]]; then
  echo "refusing to overwrite existing evidence output" >&2
  exit 2
fi

capture_arguments=(
  "$repo_dir/tools/pi-client/capture.mjs"
  --port "$port"
  --firmware "$firmware"
  --output "$output"
  --duration "$duration"
  --poll-ms "$poll_ms"
  --stage "$stage"
  --label "$label"
)
if [[ "$query" == "0" ]]; then
  capture_arguments+=(--no-query)
fi
if [[ -n "$command_plan" ]]; then
  capture_arguments+=(--command-plan "$command_plan")
fi

capture_status="0"
node "${capture_arguments[@]}" || capture_status="$?"

output_parent="$(cd "$(dirname "$output")" && pwd)"
output_name="$(basename "$output")"
tar -czf "$archive" -C "$output_parent" "$output_name"
archive_parent="$(cd "$(dirname "$archive")" && pwd)"
archive_name="$(basename "$archive")"
if command -v sha256sum >/dev/null 2>&1; then
  (cd "$archive_parent" && sha256sum "$archive_name") >"$checksum"
elif command -v shasum >/dev/null 2>&1; then
  (cd "$archive_parent" && shasum -a 256 "$archive_name") >"$checksum"
else
  echo "neither sha256sum nor shasum is available" >&2
  exit 2
fi

echo "Return these two files for analysis:"
echo "  $archive"
echo "  $checksum"
exit "$capture_status"
