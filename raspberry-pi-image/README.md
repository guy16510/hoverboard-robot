# Trashcan robot Raspberry Pi image

The image is built from the Raspberry Pi `pi-gen` `bookworm-arm64` branch. Validation runs inside the generated root filesystem, again against the completed pi-gen stage, and finally against the actual compressed image before GitHub uploads it.

## Wi-Fi provisioning

Raspberry Pi OS Bookworm uses NetworkManager. The image build creates a root-owned, mode `0600` NetworkManager profile when both Wi-Fi values are supplied.

For a local image build:

```sh
cp raspberry-pi-image/wifi.env.example raspberry-pi-image/wifi.env
chmod 600 raspberry-pi-image/wifi.env
# Edit wifi.env with the real password, then build.
bash raspberry-pi-image/build-image.sh
```

`raspberry-pi-image/wifi.env` is gitignored. Never commit the real Wi-Fi password to this public repository.

For GitHub Actions builds, configure repository secrets named:

```text
TRASHCAN_WIFI_SSID
TRASHCAN_WIFI_PASSWORD
```

The workflow passes those secrets to the image build. `IMAGE_SOURCE.txt` records `wifi_provisioned=yes` only when the profile was actually included.

## First boot

1. Flash the newest successful `trashcan-robot-rpi5-image-*` artifact.
2. Insert the card into the Raspberry Pi.
3. Power it on. A Wi-Fi-provisioned image connects automatically; otherwise connect Ethernet for setup.
4. Allow filesystem expansion and the first reboot to finish.
5. Connect over SSH:

```sh
ssh pi@trashcan-robot.local
```

Initial credentials:

```text
username: pi
password: trashcan-robot
```

Change the password immediately after the first login:

```sh
passwd
```

The image deliberately disables the interactive first-boot user rename flow and enables SSH. `userconfig.service` should not run.

The robot application is installed at `/opt/trashcan-robot/donkeycar`. Its service runs as the non-login `trashbot` user and group. The `pi` account is only the initial administrative login.

After startup, open:

```text
http://trashcan-robot.local:8888   unified status, cameras, AprilTags, and embedded controls
http://trashcan-robot.local:8887   Donkeycar manual controller directly
```

## Validation

The build fails unless all of these conditions are true:

- Debian architecture is `arm64`
- Raspberry Pi OS release is Bookworm
- the `pi` administrative user exists
- SSH is enabled
- `userconfig.service` is not enabled
- a supplied Wi-Fi profile is root-owned, mode `0600`, and enables NetworkManager autoconnect
- the Donkeycar dependency installation and `pip check` pass
- OpenCV includes AprilTag dictionaries
- the Donkeycar Python smoke tests and application tests pass
- the canonical and installed systemd units are identical
- `User`, `Group`, `WorkingDirectory`, and `ExecStart` match the `trashbot` deployment contract
- no service unit references `/opt/hoverboard-robot`
- the service is enabled
- repository files are owned by `trashbot:trashbot`
- the compressed image itself passes the service, ownership, virtual-environment, and import checks

The artifact includes `IMAGE_SOURCE.txt`, `SHA256SUMS`, the compressed image, and the complete build log. Do not flash an artifact unless its workflow is green.

To validate a downloaded artifact manually on an ARM64 Linux host:

```sh
sudo bash raspberry-pi-image/validate-image.sh path/to/trashcan-robot.img.xz
```
