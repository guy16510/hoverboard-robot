# Trashcan robot Raspberry Pi image

The image is built from the Raspberry Pi `pi-gen` `bookworm-arm64` branch. Validation runs inside the generated root filesystem, again against the completed pi-gen stage, and finally against the actual compressed image before GitHub uploads it.

## First boot

1. Flash the newest successful `trashcan-robot-rpi5-image-*` artifact.
2. Insert the card into the Raspberry Pi 5.
3. Connect Ethernet and power it on.
4. Allow the filesystem expansion and first reboot to finish.
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

The image deliberately disables the interactive first-boot user rename flow, enables SSH, and uses normal Ethernet DHCP. `userconfig.service` should not run.

The robot application is installed at `/opt/trashcan-robot/donkeycar`. Its service runs as the non-login `trashbot` user and group. The `pi` account is only the initial administrative login.

## Validation

The build fails unless all of these conditions are true:

- Debian architecture is `arm64`
- Raspberry Pi OS release is Bookworm
- the `pi` administrative user exists
- SSH is enabled
- `userconfig.service` is not enabled
- the Donkeycar dependency installation and `pip check` pass
- the Donkeycar Python smoke tests and application tests pass
- the canonical and installed systemd units are identical
- `User`, `Group`, `WorkingDirectory`, and `ExecStart` match the `trashbot` deployment contract
- no service unit references `/opt/hoverboard-robot`
- the service is enabled
- repository files are owned by `trashbot:trashbot`
- the compressed image itself passes the same service, ownership, virtual-environment, and import checks

The artifact includes `IMAGE_SOURCE.txt`, `SHA256SUMS`, the compressed image, and the complete build log. Do not flash an artifact unless its workflow is green.

To validate a downloaded artifact manually on an ARM64 Linux host:

```sh
sudo bash raspberry-pi-image/validate-image.sh path/to/trashcan-robot.img.xz
```
