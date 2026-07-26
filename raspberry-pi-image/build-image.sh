#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_DIR="$ROOT/raspberry-pi-image"
WORK_DIR="${WORK_DIR:-$IMAGE_DIR/.work}"
PI_GEN_DIR="$WORK_DIR/pi-gen"
PI_GEN_COMMIT="${PI_GEN_COMMIT:-314262cb286b8f33327a6f0cbabe14c625021ca0}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This production image build must run on an ARM64 Debian/Ubuntu host." >&2
  exit 1
fi

for tool in git rsync sudo sha256sum xz; do
  command -v "$tool" >/dev/null || { echo "missing required tool: $tool" >&2; exit 1; }
done

mkdir -p "$WORK_DIR"
if [[ ! -d "$PI_GEN_DIR/.git" ]]; then
  git clone --branch arm64 https://github.com/RPi-Distro/pi-gen.git "$PI_GEN_DIR"
fi

git -C "$PI_GEN_DIR" fetch --depth 1 origin "$PI_GEN_COMMIT"
git -C "$PI_GEN_DIR" checkout --detach "$PI_GEN_COMMIT"

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
  "$ROOT/" "$PI_GEN_DIR/stage-trashcan/00-install/files/repository/"

rm -rf "$IMAGE_DIR/dist"
mkdir -p "$IMAGE_DIR/dist"
(
  cd "$PI_GEN_DIR"
  sudo bash ./build.sh -c "$IMAGE_DIR/config"
)

# The in-image marker is verified inside stage-trashcan immediately after the
# chroot smoke tests. pi-gen may remove or rotate temporary rootfs directories
# during export, so checking that temporary path after build.sh returns is not
# reliable and caused successful image builds to be rejected.

find "$PI_GEN_DIR/deploy" -maxdepth 1 -type f \( \
  -name '*trashcan-robot*.img.xz' -o \
  -name '*trashcan-robot*.info' -o \
  -name '*trashcan-robot*.bmap' \
\) -exec cp -f {} "$IMAGE_DIR/dist/" \;

image="$(find "$IMAGE_DIR/dist" -maxdepth 1 -name '*trashcan-robot*.img.xz' -print -quit)"
[[ -n "$image" ]] || { echo "pi-gen completed without producing a compressed image" >&2; exit 1; }
xz --test "$image"
(
  cd "$IMAGE_DIR/dist"
  rm -f SHA256SUMS
  mapfile -d '' artifacts < <(find . -maxdepth 1 -type f ! -name SHA256SUMS -print0 | sort -z)
  ((${#artifacts[@]} > 0)) || { echo "no image artifacts available for checksum" >&2; exit 1; }
  sha256sum -- "${artifacts[@]}" > SHA256SUMS
  sha256sum --check SHA256SUMS
)
printf 'Image artifacts written to %s\n' "$IMAGE_DIR/dist"
