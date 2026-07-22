#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

destination="${1:-}"
[[ -n "$destination" ]] || { echo "usage: $0 DESTINATION" >&2; exit 2; }
[[ ! -e "$destination" ]] || { echo "destination already exists" >&2; exit 2; }
mkdir -p "$destination"
git clone https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32 "$destination/gd32"
git -C "$destination/gd32" checkout fa089568a523d8a43887bf8925b0ddb077ebb6ae
git clone https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x "$destination/layouts"
git -C "$destination/layouts" checkout eefe281b25086c55ff35b0322dd1de2c31965652
echo "pinned upstream snapshots fetched for optional human audit"

