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

for tool in git rsync sudo; do
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
mkdir -p "$PI_GEN_DIR/stage-trashcan/00-install/files/repository"
rsync -a --delete \
  --exclude '.git/' \
  --exclude '.venv/' \
  --exclude '.pio/' \
  --exclude '.platformio/' \
  --exclude 'data/' \
  --exclude 'raspberry-pi-image/.work/' \
  "$ROOT/" "$PI_GEN_DIR/stage-trashcan/00-install/files/repository/"

sudo "$PI_GEN_DIR/build.sh" -c "$IMAGE_DIR/config"

mkdir -p "$IMAGE_DIR/dist"
find "$PI_GEN_DIR/deploy" -maxdepth 1 -type f \( -name 'trashcan-robot*.img.xz' -o -name 'trashcan-robot*.info' -o -name 'trashcan-robot*.bmap' \) -exec cp -f {} "$IMAGE_DIR/dist/" \;
sha256sum "$IMAGE_DIR"/dist/* > "$IMAGE_DIR/dist/SHA256SUMS"
printf 'Image artifacts written to %s\n' "$IMAGE_DIR/dist"
