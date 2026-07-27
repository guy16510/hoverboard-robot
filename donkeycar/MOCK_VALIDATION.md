# Donkeycar integration mock validation

## What was mocked

The physical Raspberry Pi 5, Camera Module 3, USB serial device, ESP32, hoverboard motor controllers, Hall sensors, MPU6050, wheels, driveway grade, and trash-can load were not available to this implementation environment.

`MockMotorTransport` replaces only the serial transport boundary. It records requested linear and angular velocity commands, can simulate connection state and failures, and returns injected protocol frames. Donkeycar itself remains an external runtime dependency and is not reimplemented.

## Expected mock outputs

Running `python -m pytest -q` from `donkeycar/` should report all tests passing. The tests assert:

* CRC-16/CCITT-FALSE check value `0x29B1` for `123456789`.
* Binary frame encoding and decoding round trips.
* Decoder recovery after debug text or unrelated serial noise.
* Normalized throttle and steering are converted to configured velocity and yaw limits.
* A transport exception results in disconnected status and zero commanded velocity/yaw.
* Shutdown sends zero before disconnecting.
* Autonomous mode is suppressed while the ESP32 is disconnected.
* Manual and autonomous source selection use the appropriate Donkeycar outputs.

Running `python manage.py drive --mock --config config/robot.yaml` should start the Donkeycar process and the status dashboard without opening `/dev/ttyACM0`. The dashboard should show the ESP32 connection becoming healthy after the mock connects, and command records should remain bounded by YAML limits.

## Expected physical outputs

With compatible ESP32 firmware and wheels safely lifted:

1. Opening serial sends a protocol hello frame.
2. At zero throttle and steering, repeated velocity-and-yaw packets contain zero milli-units and a 500 ms lease.
3. Positive throttle `1.0` is limited to `0.35` configured linear velocity.
4. Steering `1.0` is limited to `0.8` configured angular velocity.
5. Removing the USB serial connection causes the Pi to stop autonomous output immediately, attempt zero, and enter disconnected state.
6. Independently, the ESP32's command lease and watchdog expire and stop motor demand.
7. Reconnection creates a fresh protocol session before motion resumes.
8. JSONL run logs appear in `donkeycar/data/logs` with timestamps, system metrics, serial latency, connection heartbeat, model name, mode, faults, and received telemetry frames.

## Not proven by mocks

The following require physical validation and must not be inferred from passing tests:

* Raspberry Pi Camera Module 3 operation and actual camera FPS.
* Donkeycar package compatibility with the exact Raspberry Pi OS and Python version.
* Current ESP32 firmware enabling drive mode and accepting velocity/yaw while not compiled in dry-run mode.
* Serial device permissions and stable USB enumeration.
* Wheel direction, steering sign, Hall feedback scale, velocity calibration, braking distance, hill-hold behavior, caster stability, traction, loaded thermal behavior, or operation on a 30-degree driveway.
* ESP32 telemetry schemas beyond raw framed packet capture.
* Physical emergency stop and independent motor watchdog behavior.

## Required hardware gate

Do not begin a driveway run until the ESP32 is running a non-dry-run drive-capable build, zero-demand serial communication is verified, wheel direction is confirmed with the chassis restrained, disconnect behavior is physically demonstrated, and a person has immediate access to a hardware emergency stop.
