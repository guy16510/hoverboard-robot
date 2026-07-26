#!/bin/bash -e

install -d -m 0755 "${ROOTFS_DIR}/opt/trashcan-robot"
rsync -a --delete "${STAGE_DIR}/00-install/files/repository/" "${ROOTFS_DIR}/opt/trashcan-robot/"
install -d -m 0755 "${ROOTFS_DIR}/etc/systemd/system"
install -m 0644 \
  "${ROOTFS_DIR}/opt/trashcan-robot/donkeycar/systemd/trashcan-donkeycar.service" \
  "${ROOTFS_DIR}/etc/systemd/system/trashcan-donkeycar.service"
