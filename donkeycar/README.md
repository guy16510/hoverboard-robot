# Donkeycar driveway runtime

Donkeycar is the Raspberry Pi high-level driving system. It sends normalized steering and throttle through the existing versioned USB serial protocol to an ESP32. The ESP32 now directly controls two WinXu 36/48 V 350 W 18 A motor controllers instead of forwarding commands to GAUSSTOP boards.

## Architecture

```text
Pi camera -> Donkeycar pipeline -> steering/throttle
                              -> ESP32Drive
                              -> USB serial
                              -> ESP32
                                   -> left/right throttle DAC
                                   -> left/right reverse isolation
                                   -> left/right brake isolation
                                   -> front/left/right ultrasonic sensors
                              -> WinXu controllers -> hoverboard wheels
```

The Pi never generates motor phase PWM. It sends velocity/yaw commands with a 500 ms lease. The ESP32 performs differential mixing, throttle slew limiting, direction-change interlocking and local lease timeout braking.

## ESP32 firmware

From the repository root:

```sh
python -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
.venv/bin/pio run -e esp32_winxu_drive
```

The application image is generated under:

```text
.pio/build/esp32_winxu_drive/
```

See `docs/WIRING.md` before flashing or powering the controllers.

## Raspberry Pi

```sh
chmod +x donkeycar/scripts/install-pi.sh
./donkeycar/scripts/install-pi.sh
source donkeycar/.venv/bin/activate
cd donkeycar
```

`config/robot.yaml` uses `serial.port: auto`. The transport prefers a stable `/dev/serial/by-id/*` ESP32 path, then `/dev/ttyUSB*`, then `/dev/ttyACM*`.

## Manual drive

```sh
python manage.py drive --config config/robot.yaml
```

Open:

* Donkeycar controller: `http://<pi-ip>:8887`
* Robot dashboard: `http://<pi-ip>:8888`

The dashboard `/api/state` includes the three ultrasonic ranges.

## Donkeycar signals

The drivetrain publishes:

```text
esp32/connected
drive/linear
drive/angular
serial/latency_ms
drive/fault
ultrasonic/front_m
ultrasonic/left_m
ultrasonic/right_m
```

Ultrasonic distances are meters. `None` means no valid recent ESP32 measurement.

These values are available to additional Donkeycar Parts, so obstacle avoidance can consume them without opening a second serial connection or talking directly to the ESP32.

## Mock validation

```sh
cd donkeycar
python -m pytest -q
python manage.py drive --mock --config config/robot.yaml
```

Mock mode validates the Donkeycar pipeline and dashboard but cannot prove controller direction, braking, throttle voltage, reverse wiring or ultrasonic electrical wiring.

## First hardware run

1. Use the wiring guide and keep both wheels lifted.
2. Complete the WinXu self-learning procedure one motor at a time.
3. Confirm the dashboard sees the ESP32 and three ultrasonic ranges.
4. Command zero throttle and confirm neither wheel moves.
5. Command a very small positive throttle and confirm both contact patches move robot-forward.
6. Release throttle and confirm both low-level brakes engage.
7. Only if both controller harnesses have a connector explicitly labeled `Reverse`, command a very small negative throttle and confirm both wheels reverse.
8. Unplug Pi USB or stop Donkeycar commands and confirm the ESP32 lease expires to zero/brake within 500 ms.
9. Lower the robot only after those checks pass.

## Training

Tubs remain standard Donkeycar tubs. Existing camera, steering and throttle training workflows are unchanged. Ultrasonic distances are live Donkeycar signals and dashboard/log telemetry, but are not automatically added to the training tub schema.
