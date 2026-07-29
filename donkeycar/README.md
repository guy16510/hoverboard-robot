# Donkeycar driveway MVP

This directory adds Donkeycar as the Raspberry Pi high-level driving system while preserving the existing GD32 hoverboard motor controller and its safety boundary.

## Architecture

```text
Pi Camera -> Donkeycar vehicle pipeline -> steering/throttle
                                      -> ESP32Drive
                                      -> velocity/yaw over USB serial
                                      -> ESP32 differential-drive coordinator
                                      -> existing GD32 motor controller firmware

USB backup camera -> OpenCV AprilTag detector -> annotated backup image and relative tag geometry
```

The Pi never generates motor PWM. The adapter uses the repository's existing binary protocol from `docs/pi-serial-protocol.md`. The ESP32 drive coordinator converts velocity and yaw into bounded left and right demands, applies slew limiting, monitors the MPU6050 and motor feedback, and sends the existing command frames to the GD32 controller.

The Raspberry Pi still has required work even with the ESP32 firmware installed. It owns the USB serial session, drive-mode selection, arm handshake, command lease renewal, neutral-on-reconnect behavior, web controller, camera pipeline, logging, and process supervision.

## Build the ESP32 drive firmware

```sh
python -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
./tools/bootstrap-tools.sh
PLATFORMIO_CORE_DIR="$PWD/.platformio" \
  .venv/bin/pio run -c platformio-drive.ini -e esp32_drive_coordinator
```

The application image is generated at:

```text
.pio/build/esp32_drive_coordinator/firmware.bin
```

The workflow publishes a complete ESP32 artifact containing the application, bootloader, partition table, ELF, and map. Do not flash or run powered wheels until the lifted-wheel gate is complete.

## Install on Raspberry Pi

```sh
git clone https://github.com/guy16510/hoverboard-robot.git
cd hoverboard-robot
git checkout feature/pi-esp32-wifi-simulated-drive
chmod +x donkeycar/scripts/install-pi.sh
./donkeycar/scripts/install-pi.sh
source donkeycar/.venv/bin/activate
cd donkeycar
```

The configured ESP32 path uses `/dev/serial/by-id`, not an unstable `/dev/ttyUSB0` number. Override it with `TRASHCAN_SERIAL_PORT` when the adapter identity differs. On connection the Pi sends hello, selects operating mode `drive`, explicitly arms, and requires neutral input before forwarding motion. Motion remains lease-bound and falls to zero when commands expire.

## Backup USB camera and AprilTags

The backup camera defaults to `device: auto`. It tries stable `/dev/v4l/by-id` and `/dev/v4l/by-path` entries first, confirms that a candidate actually returns frames, and then falls back to other V4L2 capture nodes. Override it with:

```sh
export TRASHCAN_BACKUP_CAMERA_DEVICE=/dev/v4l/by-id/<camera>-video-index0
```

The detector supports AprilTag families `16h5`, `25h9`, `36h10`, and `36h11`. Each detection includes:

- tag ID and optional semantic role
- left, center, or right position
- horizontal bearing
- image-relative lateral and vertical error
- tag rotation and image area
- estimated distance when `tag_size_m` is calibrated

Optional semantic names can be assigned in `config/robot.yaml`:

```yaml
tag_roles:
  1: driveway-home
  2: trashcan-position
```

## Mock validation without hardware

```sh
cd donkeycar
python -m pytest -q
python manage.py drive --mock --config config/robot.yaml
```

Mock mode validates the pipeline boundary and dashboard without opening a serial device. It does not prove camera drivers, motor direction, loaded braking, hill control, or physical emergency-stop behavior.

## Manual driving and recording

The normal Donkeycar controller remains the source of manual controls and recorded data. The unified dashboard embeds that controller beside both camera feeds and AprilTag state.

```sh
python manage.py drive --config config/robot.yaml
```

Open either hostname or IP address from another device on the same network:

```text
http://trashcan-robot.local:8888   unified dashboard and embedded manual controller
http://trashcan-robot.local:8887   manual controller directly
```

Both servers bind to `0.0.0.0`. Closing or losing the manual-control WebSocket forces steering and throttle to zero. The ESP32 independently expires the 500 ms command lease.

Use manual mode first. Start recording only when the camera view and steering signs are correct. Tubs remain standard Donkeycar tubs, no custom dataset format is introduced.

## Training workflow

Record multiple clean passes in both driveway directions and across lighting conditions. Copy the tubs to the training machine, then use standard Donkeycar training commands. A representative command is:

```sh
donkey train --tub ./data/tubs/* --model ./models/driveway.h5 --type linear
```

Use the exact CLI supported by the installed Donkeycar version. Copy the resulting model back to the Pi and set `model.path` and `model.name` in YAML.

## Autonomous run

```sh
python manage.py drive --config config/robot.yaml --model models/driveway.h5
```

Autonomous output is gated by the ESP32 connection. If the serial transport fails, `ESP32Drive` attempts a zero command, closes the transport, reports disconnected state, and the pipeline stops pilot output until reconnection. The ESP32 independently disables output on stale leases, feedback loss, excessive tilt, transport acknowledgment timeout, local disarm, or internal fault.

## Configuration

All integration settings are in `config/robot.yaml`, including camera dimensions, serial device, reconnect timing, logging paths, dashboard bind address, output limits, and model path.

## Logs

Each process creates JSON Lines logs under `logging.directory`. Records include camera FPS, inference rate, CPU and RAM use, process RSS, serial latency, ESP32 connection heartbeat, raw telemetry packets, wheel-speed and IMU fields when decoded, model name, mode, faults, and UTC timestamp.

## Fast driveway test order

1. Confirm both GitHub workflows are green and the image reports the expected Wi-Fi provisioning state.
2. Flash the matched ESP32 drive coordinator and Raspberry Pi image.
3. Boot with drive wheels lifted and casters restrained.
4. Open `http://trashcan-robot.local:8888` and confirm both cameras render.
5. Show a `36h11` tag to the backup camera and confirm ID and bearing update.
6. Confirm the dashboard reports the ESP32 connected with no faults.
7. Confirm zero throttle keeps both motor commands at zero.
8. Verify low positive and negative throttle produce the intended wheel directions.
9. Verify steering sign and left/right mixing.
10. Verify releasing control and closing the browser force zero demand.
11. Verify unplugging Pi USB disables motion.
12. Manually drive at walking pace with a spotter and immediate physical emergency-stop access.

This remains an MVP integration. It does not provide obstacle avoidance, route planning, docking, GPS, RTK, ROS, SLAM, or safe unattended operation.
