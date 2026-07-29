# Raspberry Pi to ESP32 Manual Drive, Hardware-Free Validation

## Scope

This branch completes and validates the Raspberry Pi web controller to Donkeycar pipeline to USB serial to ESP32 manual-drive path without connecting to a Raspberry Pi, ESP32, GD32 controller, MPU6050, camera, motor, or robot.

The user-provided hardware baseline remains the only physical evidence: `esp32_stage5_transport` moved both motors at direct left/right output `250`, both Hall odometers changed, and CRC errors and acknowledgment timeouts remained zero. Everything below is simulation, native-test, static-validation, build, or image-artifact evidence.

## Source and architecture changes

- Reworked `esp32_drive_coordinator` around the proven Stage 5 GD32 transport primitives: RMT timing, GS feedback parsing, exact ESP32/MASTER/SLAVE acknowledgments, feedback freshness, controller faults, CRC accounting, acknowledgment timeouts, applied-output tracking, zero output, and bridge disable.
- Kept the Pi protocol at USB serial `115200`, operating mode `2`, with leased linear-velocity and yaw commands.
- Added proportional differential-drive mixing, a hard `-250..250` limit, conservative `500 units/second` slew limiting, and fail-closed handling for invalid values.
- Added a safety state machine requiring mode `2`, neutral input, fresh healthy feedback, exact acknowledged zero output, zero MASTER and SLAVE faults, healthy MPU samples, safe orientation, explicit ARM, and no latched safety fault.
- Kept the MPU6050 only for tilt, rollover, invalid/missing sample, and timeout shutdown. No balancing logic was added.
- Preserved telemetry for requested velocity/yaw, mixed output, slew-limited output, applied output, command source, mode, ARM state, faults, CRC errors, acknowledgment timeouts, feedback health, and odometry.
- Changed production serial configuration to a Linux `/dev/serial/by-id/...` candidate with `TRASHCAN_SERIAL_PORT` override.
- Removed the systemd dependency on `dev-ttyACM0.device`; the application starts without the ESP32 and reconnects later.
- Explicitly bound the web controller to `0.0.0.0:8887` and dashboard to `0.0.0.0:8888`.
- Updated Python and Node shutdown paths to attempt zero demand, STOP, and DISARM independently.
- Added Node.js and npm to both the immutable Raspberry Pi image and standalone Pi installer.
- Required Node.js 18 or newer, ran a production-protocol smoke test, and ran the real Node client tests inside the ARM64 Bookworm image chroot.
- Aligned the Node direct-transport ceiling with the proven firmware ceiling: `250` is accepted and `251` is rejected.

## Simulation components

- `SimulatedEsp32Serial`, a pyserial-compatible endpoint speaking the production binary Pi protocol.
- `SimulatedGd32Boundary`, modeling command sequencing, exact/delayed/dropped/stale acknowledgments, malformed feedback, CRC failure, MASTER and SLAVE faults, feedback timeout, applied outputs, odometry, one-motor failure, and left/right disagreement.
- `SimulatedMpu6050`, modeling healthy orientation, excessive pitch/roll, missing samples, invalid values, and timeout.
- `FakeClock` and deterministic scheduling for leases, feedback age, acknowledgment deadlines, telemetry cadence, reconnects, and slew limiting.
- `SimulatedSerialFactory`, modeling startup without an ESP32, later device appearance, disconnect, and reconnect.
- End-to-end coverage through web input, Donkeycar `DriveMode`, `ESP32Drive`, Python serial transport, simulated ESP32/GD32/MPU, telemetry decoding, shared state, logging, and dashboard API.

## Commands run

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=donkeycar \
  .venv/bin/python -m pytest -vv --tb=short donkeycar/tests

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

bash donkeycar/scripts/validate-service-unit.sh \
  donkeycar/systemd/trashcan-donkeycar.service \
  trashbot trashbot /opt/trashcan-robot/donkeycar

bash raspberry-pi-image/build-image.sh
sudo bash raspberry-pi-image/validate-image.sh \
  raspberry-pi-image/dist/*.img.xz
```

The image chroot also ran:

```bash
node --version
npm --version
node --input-type=module  # production protocol CRC/frame smoke test
node --test /opt/trashcan-robot/tools/pi-client/test/*.test.mjs
```

## Final test and build results

Final code commit validated by the full suite: `2ffcb11b81f2125da21d50f6cb37fb6f9af4987f`.

- Manual-drive hardware-free workflow `30322745580`: **passed**.
- Donkeycar driveway workflow `30322745581`: **passed**.
- SWD/full firmware matrix workflow `30322745578`: **passed**, including clean-build reproducibility.
- Raspberry Pi image workflow `30322745575`: **passed**.
- Python Donkeycar, protocol, transport, simulation, reconnect, startup, dashboard, and integration tests: **56 passed**.
- Node protocol and client tests: **passed** on the host and inside the ARM64 image.
- Deterministic C++ differential-drive mixer and safety-gate tests: **passed**.
- Native, sanitized, ESP32 control, and PlatformIO native tests: **passed**.
- `esp32_drive_coordinator` production build with `-Wall -Wextra -Werror`: **passed**.
  - RAM: `23,500 / 327,680 bytes`, `7.2%`.
  - Flash: `307,717 / 1,310,720 bytes`, `23.5%`.
  - `firmware.bin` SHA-256: `69ec218c004d21834dc4853bcc164215e4ee64ce82a062b34385230700d711b3`.
- Raspberry Pi service/deployment contract: **passed**.
- ARM64 Bookworm image build, compressed-image mount, boot-invariant validation, Python smoke test, Node.js 18+ smoke test, npm presence, Node protocol tests, dependency check, ownership check, service enablement, and systemd validation: **passed**.
- Uploaded image artifact: `trashcan-robot-rpi5-image-2ffcb11b81f2125da21d50f6cb37fb6f9af4987f`.
  - Artifact ZIP size: `1,065,459,516 bytes`.
  - Artifact digest: `sha256:42dfbc674489efc860402ef4dcc8632df7acb6b056533b36d16b2bf8775532c7`.
  - Retention expiration: `2026-08-11`.

No firmware was flashed, no serial device was enumerated, no GPIO or camera was opened, and no real motor command was issued.

## Simulated success paths

- HELLO, capabilities, mode `2`, neutral zero demand, exact zero acknowledgment, ARM, leased movement, telemetry streaming, STOP, DISARM, and shutdown.
- Straight throttle produced equal outputs; steering produced differential output; combined input remained bounded.
- Slew limiting prevented instantaneous jumps and throttle release reached zero.
- Healthy simulated motors applied output and advanced odometry.
- Startup succeeded with no serial endpoint, then connected safely after the endpoint appeared.
- Reconnect repeated HELLO, mode selection, zero demand, neutral gating, and ARM without restoring stale nonzero output.
- Active command telemetry reached the dashboard through the complete production pipeline.

## Injected fault cases

The suite verified fail-closed behavior for lease expiration, USB disconnect, stale/duplicate sequences, malformed commands, command CRC failure, non-neutral startup/reconnect, delayed/dropped/stale acknowledgments, acknowledgment timeout, malformed feedback, feedback CRC failure, feedback timeout, MASTER fault, SLAVE fault, excessive pitch/roll, missing/invalid/timed-out MPU samples, local disarm, STOP, DISARM, emergency stop, process shutdown, one-motor failure, odometry disagreement, and repeated web commands after a latched fault.

## Confirmed safety invariants

- No nonzero GD32 command is emitted before explicit ARM.
- ARM is rejected until neutral input, fresh healthy feedback, exact acknowledged zero output, safe orientation, and zero controller faults are present.
- Every mixed, commanded, and transmitted output remains within `-250..250`.
- The legacy `700` ceiling and values above `250` are rejected.
- Every listed shutdown condition results in zero output and bridge-disable commands.
- Latched faults cannot be bypassed by the web controller and require a healthy explicit clear before re-ARM.
- Shutdown attempts zero demand, STOP, and DISARM even if an earlier write fails.
- Reconnect cannot restore stale nonzero demand.
- The application and systemd service start without an ESP32 present.

## Physical verification still required

This work does **not** validate motor direction, left/right polarity, steering sign, braking/coast behavior, loaded acceleration, current draw, torque, thermals, traction, hill behavior, actual `/dev/serial/by-id/...` naming, MPU mounting orientation or signs, Wi-Fi reachability, physical emergency-stop behavior, real USB timing, or real controller feedback timing under load.

## Exact next hardware-validation steps

1. Flash the uploaded Raspberry Pi image or install this branch on the Pi. Confirm `node --version`, `npm --version`, and `systemctl status trashcan-donkeycar.service`.
2. Build `esp32_drive_coordinator` from this branch and retain the generated firmware hash.
3. Secure the robot with both drive wheels lifted, then flash only `esp32_drive_coordinator`.
4. Connect the ESP32 by USB, inspect `/dev/serial/by-id/`, and set `TRASHCAN_SERIAL_PORT` if the real path differs.
5. Verify the service stays running with the ESP32 absent, then reconnect it and confirm HELLO, mode `2`, zero demand, exact zero acknowledgment, neutral gating, and ARM.
6. With motors disabled, verify actual MPU pitch/roll signs, level offsets, sample age, and shutdown thresholds.
7. Confirm zero controller faults, fresh feedback, applied output exactly zero, zero CRC errors, and zero acknowledgment timeouts.
8. From another device, verify `http://<pi>:8887` and `http://<pi>:8888` and confirm telemetry updates.
9. With wheels lifted, test straight commands `25`, `50`, and `100`, then steering in each direction. Verify wheel identity, direction, applied output, and Hall odometry.
10. Test release-to-zero, lease expiry, STOP, DISARM, serial unplug, service restart, excessive tilt, and physical emergency stop.
11. Perform the first unloaded ground test at the lowest useful command in a clear area.
12. Only after unloaded validation, test the trash-can load on level ground, then incrementally test the driveway grade while monitoring current, temperature, traction, stopping distance, and faults.
