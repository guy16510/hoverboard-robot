#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_DIR="$ROOT/raspberry-pi-image"
WORK_DIR="${WORK_DIR:-$IMAGE_DIR/.work}"
PI_GEN_DIR="$WORK_DIR/pi-gen"
PI_GEN_BRANCH="${PI_GEN_BRANCH:-bookworm-arm64}"
PI_GEN_COMMIT="${PI_GEN_COMMIT:-d7a31c6aa09f4b867902c51da2b45807c0a1709e}"
BASE_CONFIG="$IMAGE_DIR/config"
GENERATED_CONFIG="$WORK_DIR/pi-gen-config"
WIFI_ENV_FILE="${TRASHCAN_WIFI_ENV_FILE:-$IMAGE_DIR/wifi.env}"
WIFI_PROFILE="$WORK_DIR/trashcan-wifi.nmconnection"
wifi_provisioned=no

cleanup() {
  rm -f "$GENERATED_CONFIG" "$WIFI_PROFILE"
}
trap cleanup EXIT

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This production image build must run on an ARM64 Debian/Ubuntu host." >&2
  exit 1
fi

for tool in git rsync sudo sha256sum xz python3; do
  command -v "$tool" >/dev/null || { echo "missing required tool: $tool" >&2; exit 1; }
done

mkdir -p "$WORK_DIR"
cp "$BASE_CONFIG" "$GENERATED_CONFIG"
chmod 0600 "$GENERATED_CONFIG"

if [[ -f "$WIFI_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$WIFI_ENV_FILE"
fi
wifi_ssid="${TRASHCAN_WIFI_SSID:-}"
wifi_password="${TRASHCAN_WIFI_PASSWORD:-}"
if [[ -n "$wifi_ssid" || -n "$wifi_password" ]]; then
  if [[ -z "$wifi_ssid" || -z "$wifi_password" ]]; then
    echo "TRASHCAN_WIFI_SSID and TRASHCAN_WIFI_PASSWORD must be supplied together" >&2
    exit 1
  fi
  if [[ "$wifi_ssid" == *$'\n'* || "$wifi_ssid" == *$'\r'* || \
        "$wifi_password" == *$'\n'* || "$wifi_password" == *$'\r'* ]]; then
    echo "Wi-Fi SSID and password must not contain line breaks" >&2
    exit 1
  fi
  if ! (( ${#wifi_password} >= 8 && ${#wifi_password} <= 63 )) && \
     ! [[ "$wifi_password" =~ ^[0-9A-Fa-f]{64}$ ]]; then
    echo "Wi-Fi password must be 8-63 characters or a 64-character hexadecimal PSK" >&2
    exit 1
  fi

  wifi_uuid="$(python3 - "$wifi_ssid" <<'PY'
import sys
import uuid
print(uuid.uuid5(uuid.NAMESPACE_DNS, f"trashcan-robot:{sys.argv[1]}"))
PY
)"
  cat > "$WIFI_PROFILE" <<EOF_WIFI
[connection]
id=trashcan-wifi
uuid=$wifi_uuid
type=wifi
interface-name=wlan0
autoconnect=true
autoconnect-priority=100

[wifi]
mode=infrastructure
ssid=$wifi_ssid

[wifi-security]
auth-alg=open
key-mgmt=wpa-psk
psk=$wifi_password

[ipv4]
method=auto

[ipv6]
addr-gen-mode=default
method=auto

[proxy]
EOF_WIFI
  chmod 0600 "$WIFI_PROFILE"
  wifi_provisioned=yes
fi

if [[ ! -d "$PI_GEN_DIR/.git" ]]; then
  git init "$PI_GEN_DIR"
  git -C "$PI_GEN_DIR" remote add origin https://github.com/RPi-Distro/pi-gen.git
else
  git -C "$PI_GEN_DIR" remote set-url origin https://github.com/RPi-Distro/pi-gen.git
fi

git -C "$PI_GEN_DIR" fetch --force origin \
  "refs/heads/${PI_GEN_BRANCH}:refs/remotes/origin/${PI_GEN_BRANCH}"
git -C "$PI_GEN_DIR" fetch --force origin "$PI_GEN_COMMIT"

if ! git -C "$PI_GEN_DIR" merge-base --is-ancestor \
  "$PI_GEN_COMMIT" "refs/remotes/origin/${PI_GEN_BRANCH}"; then
  echo "Pinned pi-gen commit $PI_GEN_COMMIT is not on $PI_GEN_BRANCH" >&2
  exit 1
fi

git -C "$PI_GEN_DIR" checkout --detach "$PI_GEN_COMMIT"
resolved_pi_gen_commit="$(git -C "$PI_GEN_DIR" rev-parse HEAD)"
if [[ "$resolved_pi_gen_commit" != "$PI_GEN_COMMIT" ]]; then
  echo "pi-gen checkout mismatch: expected $PI_GEN_COMMIT, got $resolved_pi_gen_commit" >&2
  exit 1
fi

grep -Fq 'if [ "$RELEASE" != "bookworm" ]; then' "$PI_GEN_DIR/stage0/prerun.sh" || {
  echo "Pinned pi-gen source is not the Bookworm release branch" >&2
  exit 1
}
grep -Fq 'http://deb.debian.org/debian/' "$PI_GEN_DIR/stage0/prerun.sh" || {
  echo "Pinned pi-gen source is not the ARM64 Debian bootstrap" >&2
  exit 1
}
printf 'Using pi-gen %s from %s\n' "$resolved_pi_gen_commit" "$PI_GEN_BRANCH"
printf 'Wi-Fi provisioning: %s\n' "$wifi_provisioned"

sudo rm -rf "$PI_GEN_DIR/work" "$PI_GEN_DIR/deploy"
rm -rf "$PI_GEN_DIR/stage-trashcan"
cp -a "$IMAGE_DIR/stage-trashcan" "$PI_GEN_DIR/stage-trashcan"
find "$PI_GEN_DIR/stage-trashcan" -type f \( -name '*.sh' -o -name 'prerun.sh' \) -exec chmod 0755 {} +
mkdir -p "$PI_GEN_DIR/stage-trashcan/00-install/files/repository"
rsync -a --delete \
  --exclude '.git/' \
  --exclude '.venv/' \
  --exclude '.pio/' \
  --exclude '.platformio/' \
  --exclude 'data/' \
  --exclude 'raspberry-pi-image/.work/' \
  --exclude 'raspberry-pi-image/dist/' \
  --exclude 'raspberry-pi-image/wifi.env' \
  "$ROOT/" "$PI_GEN_DIR/stage-trashcan/00-install/files/repository/"

if [[ "$wifi_provisioned" == yes ]]; then
  install -D -m 0600 "$WIFI_PROFILE" \
    "$PI_GEN_DIR/stage-trashcan/01-network/files/trashcan-wifi.nmconnection"
fi

rm -rf "$IMAGE_DIR/dist"
mkdir -p "$IMAGE_DIR/dist"
(
  cd "$PI_GEN_DIR"
  sudo bash ./build.sh -c "$GENERATED_CONFIG"
)

image_name="$(bash -c 'source "$1"; printf "%s" "$IMG_NAME"' _ "$BASE_CONFIG")"
stage_root="$PI_GEN_DIR/work/$image_name/stage-trashcan/rootfs"
[[ -d "$stage_root" ]] || {
  echo "pi-gen completed without the expected stage rootfs: $stage_root" >&2
  exit 1
}
sudo env ROOTFS_DIR="$stage_root" \
  bash "$IMAGE_DIR/stage-trashcan/00-install/01-verify-marker.sh"
if [[ "$wifi_provisioned" == yes ]]; then
  sudo test -f "$stage_root/etc/trashcan-wifi-provisioned"
  sudo test "$(sudo stat -c '%a' "$stage_root/etc/NetworkManager/system-connections/trashcan-wifi.nmconnection")" = 600
fi

find "$PI_GEN_DIR/deploy" -maxdepth 1 -type f \( \
  -name '*trashcan-robot*.img.xz' -o \
  -name '*trashcan-robot*.info' -o \
  -name '*trashcan-robot*.bmap' \
\) -exec cp -f {} "$IMAGE_DIR/dist/" \;

image="$(find "$IMAGE_DIR/dist" -maxdepth 1 -name '*trashcan-robot*.img.xz' -print -quit)"
[[ -n "$image" ]] || { echo "pi-gen completed without producing a compressed image" >&2; exit 1; }
xz --test "$image"

cat > "$IMAGE_DIR/dist/IMAGE_SOURCE.txt" <<EOF_INFO
pi_gen_branch=$PI_GEN_BRANCH
pi_gen_commit=$resolved_pi_gen_commit
release=bookworm
architecture=arm64
first_user=pi
first_boot_user_rename=disabled
ssh=enabled
wifi_provisioned=$wifi_provisioned
wifi_backend=NetworkManager
nodejs=installed
nodejs_minimum_major=18
npm=installed
node_protocol_tests=passed
robot_user=trashbot
robot_group=trashbot
robot_working_directory=/opt/trashcan-robot/donkeycar
robot_service_contract=validated
EOF_INFO

(
  cd "$IMAGE_DIR/dist"
  rm -f SHA256SUMS
  mapfile -d '' artifacts < <(find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 | sort -z)
  ((${#artifacts[@]} > 0)) || { echo "no image artifacts available for checksum" >&2; exit 1; }
  sha256sum -- "${artifacts[@]}" > SHA256SUMS
  sha256sum --check SHA256SUMS
)
printf 'Image artifacts written to %s\n' "$IMAGE_DIR/dist"
