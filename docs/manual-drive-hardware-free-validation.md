# Raspberry Pi to ESP32 Manual Drive, Hardware-Free Validation

## Scope

This branch completes and validates the Raspberry Pi web controller to Donkeycar pipeline to USB serial to ESP32 manual-drive path without connecting to a Raspberry Pi, ESP32, GD32 controller, MPU6050, camera, motor, or robot.

The prior user-provided hardware baseline remains the only physical evidence: `esp32_stage5_transport` moved both motors at direct left/right output `250`, both Hall odometers changed, and CRC errors and acknowledgment timeouts remained zero. Everything added and reported below is simulation, native-test, static-validation, or build evidence.

## Source and architecture changes

- Reworked `esp32_drive_coordinator` around the proven Stage 5 GD32 transport primitives:
  - RMT pulse command transport and timing;
  - production GS frame parser and feedback decoder;
  - command sequencer with exact ESP32, MASTER-forwarded, and SLAVE-applied acknowledgments;
  - feedback freshness, controller runtime-health, fault, CRC, timeout, and applied-output handling;
  - zero and bridge-disable behavior when output is not permitted.
- Kept the Raspberry Pi protocol at USB serial `115200`, operating mode `2`, with leased linear-velocity and yaw commands.
- Added a production differential-drive mixer with proportional saturation, a hard `-250..250` ceiling, conservative `500 units/second` slew limiting, and fail-closed handling for non-finite or invalid configuration.
- Added a portable drive safety state machine requiring:
  - mode `2`;
  - neutral input after every connection or mode change;
  - fresh and runtime-healthy feedback;
  - exact acknowledged zero applied output;
  - zero MASTER and SLAVE faults;
  - healthy MPU samples and safe orientation;
  - explicit ARM;
  - no latched safety fault.
- Kept the MPU6050 only for pitch, roll, missing-sample, invalid-value, and timeout shutdown. No balancing logic was added.
- Added drive telemetry for requested velocity/yaw, mixed output, slew-limited command, applied output, source, mode, ARM state, safety faults, feedback health, CRC errors, acknowledgment timeouts, and odometry.
- Changed production Raspberry Pi serial configuration to the known Linux by-id candidate:
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
  with `TRASHCAN_SERIAL_PORT` as an override.
- Removed the systemd unit dependency on `dev-ttyACM0.device`; the application starts without the ESP32 and retries when the configured path appears.
- Explicitly bound the Donkeycar web controller to `0.0.0.0:8887` and the dashboard to `0.0.0.0:8888`.
- Updated shutdown behavior so the Python and Node clients independently attempt zero demand, STOP, and DISARM before closing.
- Decoded ESP32 telemetry into shared robot state so the dashboard API receives the real production protocol fields.

## Simulation components

- `SimulatedEsp32Serial`, a pyserial-compatible endpoint that parses and emits the production binary Raspberry Pi protocol.
- `SimulatedGd32Boundary`, modeling command sequencing, exact/delayed/dropped/stale acknowledgments, malformed feedback, CRC failure, MASTER and SLAVE faults, feedback timeout, applied outputs, odometry, one-motor failure, and left/right disagreement.
- `SimulatedMpu6050`, modeling level operation, pitch, roll, missing samples, invalid values, and sensor timeout.
- `FakeClock` and deterministic scheduling for leases, feedback age, acknowledgment deadlines, telemetry cadence, reconnects, and slew limiting.
- `SimulatedSerialFactory`, modeling service startup with no ESP32, later device appearance, USB disconnect, and reconnect.
- End-to-end simulated coverage through web input, Donkeycar `DriveMode`, `ESP32Drive`, Python serial transport, simulated ESP32/GD32/MPU, telemetry decoding, logging, shared state, and dashboard API.

## Commands executed in GitHub Actions

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=donkeycar \
  .venv/bin/python -m pytest -vv --tb=short donkeycar/tests

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=donkeycar \
  .venv/bin/python -m compileall -q donkeycar/trashcan_robot donkeycar/manage.py

node --test tools/pi-client/test/*.test.mjs

g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/esp32/control \
  tests/esp32/test_differential_drive.cpp \
  firmware/esp32/control/differential_drive_mixer.cpp \
  -o /tmp/test-differential-drive
/tmp/test-differential-drive

g++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/esp32/control \
  tests/esp32/test_drive_safety.cpp \
  firmware/esp32/control/drive_safety.cpp \
  -o /tmp/test-drive-safety
/tmp/test-drive-safety

./tools/test-native.sh
./tools/test-sanitized.sh
./tools/test-esp32-balance.sh
.venv/bin/pio run -e native_tests
.pio/build/native_tests/program

.venv/bin/pio run -c platformio-drive.ini -e esp32_drive_coordinator

bash -n donkeycar/scripts/install-pi.sh
bash -n donkeycar/scripts/install-service.sh
bash -n donkeycar/scripts/validate-service-unit.sh
bash donkeycar/scripts/validate-service-unit.sh \
  donkeycar/systemd/trashcan-donkeycar.service \
  trashbot trashbot /opt/trashcan-robot/donkeycar
```

## Test and build results

Validation run `30321370983` completed successfully against branch commit `ec1feac188d9f3856b3adc14d165d5cd3ee4e25a` before this report was added.

- Python Donkeycar, protocol, transport, simulation, reconnect, service-start, dashboard, and integration tests: **56 passed**.
- Node protocol and client tests: **passed**.
- Deterministic C++ differential-drive mixer tests: **passed**.
- Deterministic C++ drive safety-gate tests: **passed**.
- Existing native firmware tests: **passed**.
- Sanitized native tests: **passed**.
- Existing ESP32 balance/control native tests: **passed**.
- PlatformIO `native_tests`: **passed**.
- `esp32_drive_coordinator` production build: **passed** with `-Wall -Wextra -Werror`.
  - RAM: `23,500 / 327,680 bytes`, `7.2%`.
  - Flash: `307,717 / 1,310,720 bytes`, `23.5%`.
  - `firmware.bin` SHA-256: `69ec218c004d21834dc4853bcc164215e4ee64ce82a062b34385230700d711b3`.
  - `firmware.elf` SHA-256: `5aabf5c6817ea4ee33f948052a31c439e2a662ddd1d60370399057f4d67051c4`.
  - `firmware.map` SHA-256: `470ce2c21b619819814b8e3b76a2e30b7726767fad609452a2eaf9e8c718c28a`.
- Raspberry Pi scripts and service contract: **passed**.
- Confirmed no macOS `/dev/cu.*` production path and no systemd serial-device unit dependency.
- Confirmed configured web bind `0.0.0.0:8887` and dashboard bind `0.0.0.0:8888`.

No firmware was flashed, no serial device was enumerated, no GPIO or camera was opened, and no real motor command was issued.

## Simulated success paths

- HELLO, capabilities, mode `2`, neutral zero demand, exact zero acknowledgment, ARM, leased movement, telemetry streaming, STOP, DISARM, and clean shutdown.
- Straight throttle produced equal left and right output.
- Steering produced the expected differential output.
- Combined throttle and steering remained proportionally bounded.
- Slew limiting prevented instantaneous command jumps.
- Releasing throttle reached zero.
- Both motors applied output and advanced odometry in the healthy model.
- Service startup succeeded with no serial endpoint, then connected safely after the endpoint appeared.
- Reconnect repeated HELLO, mode `2`, zero demand, neutral gating, and ARM, without restoring a stale nonzero command.
- Active command telemetry reached the dashboard API through the complete simulated production pipeline.

## Injected fault cases

The test suite injected and verified fail-closed behavior for:

- command lease expiration;
- USB serial disconnect and reconnect;
- stale and duplicate Raspberry Pi sequences;
- malformed commands and bad command CRC;
- non-neutral startup and reconnect input;
- delayed, dropped, and stale GD32 acknowledgments;
- acknowledgment timeout;
- malformed GD32 feedback and feedback CRC failure;
- feedback timeout;
- MASTER fault and SLAVE fault;
- excessive pitch and roll;
- missing, invalid, and timed-out MPU samples;
- local disarm;
- STOP, DISARM, emergency stop, and process shutdown;
- one motor failing to respond;
- left/right odometry disagreement;
- repeated web commands after a latched ESP32 safety fault.

## Confirmed safety invariants

Simulation and native tests confirm:

- No nonzero GD32 command is emitted before explicit ARM.
- ARM is rejected until mode `2`, neutral input, fresh healthy feedback, exact acknowledged zero output, healthy orientation, and zero controller faults are present.
- Every mixed, commanded, and transmitted output remains within `-250..250`.
- The legacy unvalidated `700` ceiling is rejected.
- Lease expiration, disconnect, STOP, DISARM, local disarm, unsafe orientation, feedback loss, acknowledgment timeout, controller fault, malformed command, stale sequence, or feedback CRC failure results in zero output and bridge-disable commands.
- Latched faults cannot be bypassed by the web controller and require a healthy explicit clear before re-ARM.
- Shutdown attempts zero demand, STOP, and DISARM independently, even if an earlier shutdown write fails.
- Reconnect cannot restore a stale nonzero command.
- The Raspberry Pi application and systemd service do not require the ESP32 to exist at process startup.

## Physical verification still required

This work does **not** validate:

- motor direction or left/right polarity;
- steering sign;
- braking or coast behavior and braking strength;
- unloaded or loaded acceleration and deceleration;
- loaded current, torque, thermal behavior, traction, or hill behavior;
- actual Raspberry Pi USB `/dev/serial/by-id/...` naming;
- actual MPU mounting orientation, pitch/roll signs, noise, or cutoff behavior;
- Wi-Fi reachability of ports `8887` and `8888` from another device;
- physical emergency-stop wiring or effectiveness;
- real USB disconnect timing;
- real controller feedback timing under load;
- one-wheel stall detection or odometry disagreement thresholds as a physical shutdown policy.

## Exact next hardware-validation steps

1. Check out `feature/pi-esp32-wifi-simulated-drive` on the machine used for flashing. Build `esp32_drive_coordinator` again and retain the generated binary hash.
2. With motor wheels lifted and the robot mechanically secured, flash only `esp32_drive_coordinator`. Do not begin with wheels on the ground.
3. Connect the ESP32 to the Raspberry Pi by USB. Run `ls -l /dev/serial/by-id/` and compare the actual path with the configured candidate. Set `TRASHCAN_SERIAL_PORT` in `/etc/default/trashcan-donkeycar` if it differs.
4. Start or restart `trashcan-donkeycar.service`. Verify it remains running before the ESP32 is connected, then reconnect the ESP32 and confirm the application performs HELLO, mode `2`, zero demand, and ARM only after exact zero acknowledgment.
5. With the robot level, verify actual MPU pitch and roll values, signs, sample age, and mounting orientation. Physically tilt each axis with motors disabled and confirm the configured shutdown direction and threshold before allowing motion.
6. Confirm MASTER and SLAVE faults are zero, feedback is fresh and runtime-healthy, applied output is exactly zero, CRC errors are zero, and acknowledgment timeouts are zero.
7. From another device on the intended Wi-Fi network, verify the web controller at port `8887` and dashboard at port `8888`. Confirm telemetry fields update while commands are active.
8. With wheels still lifted, command straight motion at small bounded values such as `25`, `50`, and `100`. Verify both wheel directions, left/right identity, applied outputs, Hall odometry direction, and no faults or protocol errors.
9. Test left/right steering separately at small values. Correct motor polarity or mixer sign only from observed physical evidence.
10. With wheels lifted, test release-to-zero, lease expiration, STOP, DISARM, serial unplug, service restart, excessive tilt, and physical emergency stop. Confirm the bridges disable and wheels stop in every case.
11. Perform the first ground test unloaded at the lowest useful command in a clear area. Validate steering, stopping distance, coast/brake behavior, and emergency-stop access.
12. Only after unloaded validation, test the trash-can load at low speed on level ground, then incrementally validate driveway grade, traction, braking, current draw, temperatures, and fault behavior.
