#!/bin/bash -e

set -euo pipefail

marker="${ROOTFS_DIR}/etc/trashcan-robot-image-validated"
build_info="${ROOTFS_DIR}/etc/trashcan-robot-build-info"
app="${ROOTFS_DIR}/opt/trashcan-robot/donkeycar"
service_source="$app/systemd/trashcan-donkeycar.service"
service_target="${ROOTFS_DIR}/etc/systemd/system/trashcan-donkeycar.service"
enabled_link="${ROOTFS_DIR}/etc/systemd/system/multi-user.target.wants/trashcan-donkeycar.service"

if [[ ! -f "$marker" ]]; then
  echo "Trashcan robot image validation marker was not created by the chroot validation step" >&2
  exit 1
fi
if [[ ! -f "$build_info" ]]; then
  echo "Trashcan robot image build information was not created" >&2
  exit 1
fi

for fact in \
  'architecture=arm64' \
  'release=bookworm' \
  'first_boot_userconfig=disabled' \
  'ssh=enabled' \
  'robot_user=trashbot' \
  'robot_group=trashbot' \
  'robot_working_directory=/opt/trashcan-robot/donkeycar' \
  'robot_service=enabled' \
  'robot_service_contract=validated' \
  'python_smoke=passed' \
  'application_tests=passed' \
  'systemd_verify=passed' \
  'dependency_check=passed' \
  'repository_owner=trashbot:trashbot'; do
  grep -Fxq "$fact" "$build_info" || {
    echo "Missing validated image fact: $fact" >&2
    exit 1
  }
done

[[ -x "$app/.venv/bin/python" ]] || {
  echo "Validated image is missing the Donkeycar virtual environment" >&2
  exit 1
}
[[ -f "$service_source" && -f "$service_target" ]] || {
  echo "Validated image is missing the canonical or installed service unit" >&2
  exit 1
}
cmp -s "$service_source" "$service_target" || {
  echo "Installed service unit differs from the canonical repository unit" >&2
  exit 1
}
"$app/scripts/validate-service-unit.sh" \
  "$service_target" trashbot trashbot /opt/trashcan-robot/donkeycar

[[ -L "$enabled_link" ]] || {
  echo "trashcan-donkeycar.service enablement symlink is missing" >&2
  exit 1
}
link_target="$(readlink "$enabled_link")"
if [[ "$link_target" != '/etc/systemd/system/trashcan-donkeycar.service' && \
      "$link_target" != '../trashcan-donkeycar.service' ]]; then
  echo "Unexpected trashcan-donkeycar.service enablement target: $link_target" >&2
  exit 1
fi

trashbot_uid="$(awk -F: '$1 == "trashbot" { print $3 }' "${ROOTFS_DIR}/etc/passwd")"
trashbot_gid="$(awk -F: '$1 == "trashbot" { print $3 }' "${ROOTFS_DIR}/etc/group")"
[[ -n "$trashbot_uid" && -n "$trashbot_gid" ]] || {
  echo "trashbot user or group is missing from the generated root filesystem" >&2
  exit 1
}
bad_owner="$(find "${ROOTFS_DIR}/opt/trashcan-robot" \( ! -uid "$trashbot_uid" -o ! -gid "$trashbot_gid" \) -print -quit)"
if [[ -n "$bad_owner" ]]; then
  echo "Generated root filesystem contains incorrectly owned repository content: $bad_owner" >&2
  stat -c '%u:%g %n' "$bad_owner" >&2 || true
  exit 1
fi

printf 'Validated trashcan robot image marker: %s\n' "$marker"
printf 'Validated trashcan robot image facts:\n'
cat "$build_info"
