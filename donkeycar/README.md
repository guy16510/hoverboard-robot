# Donkeycar driveway MVP

This directory adds Donkeycar as the Raspberry Pi high-level driving system while preserving the existing GD32 hoverboard motor controller and its safety boundary.

## Architecture

```text
Pi Camera 3 -> Donkeycar vehicle pipeline -> steering/throttle
                                       -> ESP32Drive
                                       -> velocity/yaw over USB serial
                                       -> ESP32 differential-drive coordinator
                                       -> existing GD32 motor controller firmware
```

The Pi never generates motor PWM. The adapter uses the repository's existing binary protocol from `docs/pi-serial-protocol.md`. The ESP32 drive coordinator converts velocity and yaw into bounded left and right demands, applies slew limiting, monitors the MPU6050 and motor feedback, and sends the existing command frames to the GD32 controller.

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

The pull request workflow also publishes a complete ESP32 artifact containing the application, bootloader, partition table, ELF, and map. Flash using the established ESP32 method for this repository. Do not flash or run powered wheels until the lifted-wheel gate below is complete.

## Install on Raspberry Pi 5

```sh
git clone https://github.com/guy16510/hoverboard-robot.git
cd hoverboard-robot
git checkout goal/donkeycar-driveway-mvp
chmod +x donkeycar/scripts/install-pi.sh
./donkeycar/scripts/install-pi.sh
source donkeycar/.venv/bin/activate
cd donkeycar
```

Confirm `/dev/ttyACM0` is the ESP32 and adjust `config/robot.yaml` when needed. On connection the Pi sends hello, selects operating mode `drive`, and explicitly arms. Motion remains lease-bound and falls to zero when commands expire.

## Mock validation without hardware

```sh
cd donkeycar
python -m pytest -q
python manage.py drive --mock --config config/robot.yaml
```

Mock mode validates the pipeline boundary and dashboard without opening a serial device. It does not prove camera drivers, motor direction, loaded braking, hill control, or physical emergency-stop behavior.

## Manual driving and recording

Donkeycar's normal controller and tub pipeline remain the source of manual controls and recorded data. Use the Donkeycar web controller from another device on the same network, or configure its supported joystick controller for an Xbox controller.

```sh
python manage.py drive --config config/robot.yaml
```

Open:

* Donkeycar controller, `http://<pi-ip>:8887`
* Robot status dashboard, `http://<pi-ip>:8888`

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

Autonomous output is gated by the ESP32 connection. If the serial transport fails, `ESP32Drive` attempts a zero command, closes the transport, reports disconnected state, and the pipeline stops pilot inference/output until reconnection. The ESP32 independently disables output on stale leases, feedback loss, excessive tilt, transport acknowledgment timeout, local disarm, or internal fault.

## Configuration

All integration settings are in `config/robot.yaml`, including camera dimensions, serial device, reconnect timing, logging paths, dashboard bind address, output limits, and model path. There are no hardcoded absolute paths.

The initial ESP32 mixer defaults are intentionally bounded to 700 command units with a 900-unit-per-second slew limit. Change those only after lifted-wheel direction and braking tests.

## Logs

Each process creates JSON Lines logs under `logging.directory`. Records include camera FPS, inference rate, CPU and RAM use, process RSS, serial latency, ESP32 connection heartbeat, raw telemetry packets, wheel-speed and IMU fields when decoded, model name, mode, faults, and UTC timestamp.

## Fast driveway test order

1. Confirm the pull request CI built the production ESP32 artifact and passed Python and mixer tests.
2. Flash the ESP32 drive coordinator.
3. Boot with drive wheels lifted and casters restrained.
4. Confirm the MPU6050 is detected and the dashboard reports an ESP32 connection.
5. Confirm zero throttle keeps both motor commands at zero.
6. Verify positive throttle produces the intended forward wheel direction at the lowest Donkeycar limit.
7. Verify steering sign and left/right mixing.
8. Verify releasing control lets the 500 ms lease expire to zero.
9. Verify unplugging Pi USB disables motion.
10. Verify tilting the chassis beyond 45 degrees disables motion.
11. Manually drive at walking pace and record several tubs.
12. Train a basic linear model.
13. Run autonomous mode with a spotter and immediate physical emergency-stop access.

This is an MVP integration. It intentionally excludes obstacle avoidance, route planning, docking, GPS, RTK, ROS, SLAM, and trash-can detection.
