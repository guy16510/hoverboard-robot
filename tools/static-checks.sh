#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
"$repo_dir/tools/check-format.sh"
"$repo_dir/tools/check-classification.sh"
"$repo_dir/tools/check-licenses.sh"
"$repo_dir/tools/check-hardware-isolation.sh"
"$repo_dir/tools/check-architecture.sh"
for script in "$repo_dir"/tools/*.sh "$repo_dir"/tools/hardware/*.sh; do
  bash -n "$script"
done
echo "static checks passed"

