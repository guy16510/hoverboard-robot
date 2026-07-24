#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"
[[ -f SOURCE_CLASSIFICATION.csv ]] || { echo "missing SOURCE_CLASSIFICATION.csv" >&2; exit 1; }
temp_dir="$(mktemp -d)"
{
  git ls-files
  git ls-files --others --exclude-standard
} | sort -u > "$temp_dir/tracked"
tail -n +2 SOURCE_CLASSIFICATION.csv | cut -d, -f1 | sort > "$temp_dir/classified"
diff -u "$temp_dir/tracked" "$temp_dir/classified"
awk -F, 'NR > 1 && $2 !~ /^(Unmodified upstream|Modified upstream|New GAUSSTOP-specific code|Reused legacy code|New ESP32 code)$/ { print "invalid classification: " $0; bad=1 } END { exit bad }' SOURCE_CLASSIFICATION.csv
