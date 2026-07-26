# Donkeycar driveway MVP

This directory adds Donkeycar as the Raspberry Pi high-level driving system while preserving the existing ESP32 motor controller and safety boundary.

## Architecture

```text
Pi Camera 3 -> Donkeycar vehicle pipeline -> steering/throttle
                                      -> ESP32Drive
                                      -> velocity/yaw over USB serial
                                      -> existing ESP32 closed-loop motor control
```

The Pi never generates motor PWM. The adapter uses the repository's existing binary protocol from `docs/pi-serial-protocol.md`.

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

Confirm `/dev/ttyACM0` is the ESP32 and adjust `config/robot.yaml` when needed. The ESP32 firmware must support operating mode `drive`, the velocity-and-yaw command, heartbeat leases, and telemetry as documented by the existing protocol.

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

Autonomous output is gated by the ESP32 connection. If the serial transport fails, `ESP32Drive` attempts a zero command, closes the transport, reports disconnected state, and the pipeline stops pilot inference/output until reconnection.

## Configuration

All integration settings are in `config/robot.yaml`, including camera dimensions, serial device, reconnect timing, logging paths, dashboard bind address, output limits, and model path. There are no hardcoded absolute paths.

## Logs

Each process creates JSON Lines logs under `logging.directory`. Records include camera FPS, inference rate, CPU and RAM use, process RSS, serial latency, ESP32 connection heartbeat, raw telemetry packets, wheel-speed and IMU fields when decoded, model name, mode, faults, and UTC timestamp.

## Fast driveway test order

1. Run all tests in mock mode.
2. Boot the Pi with drive wheels lifted and casters restrained.
3. Confirm camera and dashboard.
4. Confirm serial connection with zero throttle.
5. Verify positive throttle produces the intended forward wheel direction at the lowest configured limit.
6. Verify steering sign.
7. Verify unplugging USB stops Pi output and the ESP32 watchdog stops the motors.
8. Manually drive at walking pace and record several tubs.
9. Train a basic linear model.
10. Run autonomous mode with a spotter and immediate physical emergency-stop access.

This is an MVP integration. It intentionally excludes obstacle avoidance, route planning, docking, GPS, RTK, ROS, SLAM, and trash-can detection.
