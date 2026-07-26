# Trashcan robot Raspberry Pi image

The image is built from the Raspberry Pi `pi-gen` `bookworm-arm64` branch and is validated inside the generated root filesystem before export.

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

## Validation

The build fails unless all of these conditions are true:

- Debian architecture is `arm64`
- Raspberry Pi OS release is Bookworm
- the `pi` user exists
- SSH is enabled
- `userconfig.service` is not enabled
- the Donkeycar Python environment and tests pass
- the robot systemd unit validates

The artifact includes `IMAGE_SOURCE.txt`, `SHA256SUMS`, the compressed image, and the complete build log. Do not flash an artifact unless its workflow is green.
