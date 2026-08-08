# Trashcan robot drivetrain

This repository drives two hoverboard wheels using two independent WinXu 36/48 V 350 W 18 A brushless motor controllers and one ESP32.

The previous GAUSSTOP GD32 firmware, reverse-engineering notes, research documents, SWD transport, and master/slave controller stack have been removed.

## Architecture

```text
Donkeycar on Raspberry Pi
        |
        | USB serial, 115200 baud
        | versioned binary command + lease protocol
        v
      ESP32
        |
        +-- GPIO25 DAC -> left controller throttle
        +-- GPIO26 DAC -> right controller throttle
        +-- GPIO27 -> isolated left reverse switch
        +-- GPIO14 -> isolated right reverse switch
        +-- GPIO33 -> isolated left low-brake switch
        +-- GPIO32 -> isolated right low-brake switch
        |
        +-- front ultrasonic: TRIG16 / ECHO34
        +-- left ultrasonic:  TRIG17 / ECHO35
        +-- right ultrasonic: TRIG18 / ECHO39
        |
        +-- WinXu controller -> left hoverboard wheel
        +-- WinXu controller -> right hoverboard wheel
```

The ESP32 starts with both low-level brakes asserted. Motion requires Donkeycar to select drive mode, establish a current 500 ms command lease, and arm. If Pi commands stop arriving, throttle drops to zero and the brakes are asserted locally on the ESP32.

Direction changes are interlocked. The ESP32 ramps throttle to zero, applies the brake, changes the isolated reverse input, waits for the controller to settle, then ramps throttle back up.

## Wiring

Read [`docs/WIRING.md`](docs/WIRING.md) before powering the motor controllers. It contains the exact ESP32 pin map, the WinXu controller reference image, throttle wiring, brake/reverse isolation requirements, ultrasonic voltage dividers, and the lifted-wheel bring-up sequence.

## ESP32 build

```sh
python -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
.venv/bin/pio run -e esp32_winxu_drive
```

Flash with PlatformIO after selecting the ESP32 serial port:

```sh
.venv/bin/pio run -e esp32_winxu_drive -t upload --upload-port /dev/ttyUSB0
```

The production build defaults are in `platformio.ini`.

## Donkeycar

The Raspberry Pi side remains under `donkeycar/`. The serial port is configured as `auto`, which prefers `/dev/serial/by-id/*` and falls back to `/dev/ttyUSB*` or `/dev/ttyACM*`.

Donkeycar publishes these drivetrain outputs:

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

The ultrasonic values are also included in the robot dashboard state and JSON run logs.

Run the Pi tests with:

```sh
PYTHONPATH=donkeycar python -m pytest -q donkeycar/tests
```

## Safety boundary

Software is not the emergency stop. Put the WinXu power-lock/controller-enable path behind a physical emergency-stop or keyed switch. The ESP32 brake outputs are useful fail-safe behavior while firmware is running, but an ESP32 reset or wiring failure must not be treated as a substitute for a physical power disconnect.

Do all first motion and reverse testing with the wheels lifted and the chassis physically restrained.
