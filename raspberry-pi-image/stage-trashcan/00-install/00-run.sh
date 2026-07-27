#!/bin/bash -e

set -euo pipefail

install -d -m 0755 "${ROOTFS_DIR}/opt/trashcan-robot"
rsync -a --delete "${STAGE_DIR}/00-install/files/repository/" "${ROOTFS_DIR}/opt/trashcan-robot/"
