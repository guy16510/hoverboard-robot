#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_dir"
test -s LICENSE
rg -q 'GNU GENERAL PUBLIC LICENSE' LICENSE
source_files=()
while IFS= read -r source; do
  source_files+=("$source")
done < <(rg --files firmware tests tools -g '*.c' -g '*.h' -g '*.cpp' -g '*.sh' -g '*.py')
for source in "${source_files[@]}"; do
  if ! sed -n '1,8p' "$source" | rg -q 'SPDX-License-Identifier:'; then
    echo "missing SPDX header: $source" >&2
    exit 1
  fi
done
