#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
formatter="$repo_dir/.venv/bin/clang-format"
[[ -x "$formatter" ]] || { echo "missing pinned clang-format" >&2; exit 1; }
cd "$repo_dir"
sources=()
while IFS= read -r source; do
  sources+=("$source")
done < <(rg --files firmware tests -g '*.c' -g '*.h' -g '*.cpp')
"$formatter" --dry-run --Werror "${sources[@]}"
