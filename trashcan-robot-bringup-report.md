# Trashcan Robot Bring-up Report

Date: 2026-07-28  
Raspberry Pi: `pi@192.168.2.47` (`trashcan-robot`)  
Repository baseline: `feature/pi-esp32-wifi-simulated-drive` at
`b5fd54c629cfdfa2ada38978efebef5383b6d467`  
Source-control status: **all changes are local and uncommitted; no commit,
push, merge, pull request, amend, tag, or GitHub write was performed**

## Readiness verdict

**The Pi → USB serial → ESP32 → dual-GD32 controller path, camera, web
controls, service, telemetry, and fail-safes are operational. The robot is
ready for continued lifted-wheel diagnosis and zero-demand web operation, but
it is not ready for floor driving.**

The blocking defect is isolated to logical right-forward:

- Right-forward demand `250` enables the right bridge and is accepted by the
  slave controller, but the wheel produces no Hall odometry.
- Right-reverse demand `-250` works and produces right odometry.
- Left forward/reverse work.
- A power cycle did not resolve right-forward. Post-reboot retries of 0.75 and
  1.0 seconds both ended with right odometry delta `0`.
- The 1.0-second retry showed commanded right output ramping through `170`,
  applied feedback `39`, right bridge enabled, and no CRC, acknowledgment,
  master, slave, or balance fault at the sampled instant, but still no Hall
  count. An earlier longer observation latched the expected GD32
  `GS_FAULT_STARTUP_TIMEOUT` (`0x8`).

Bridge-disabled manual-spin telemetry subsequently proved that both Hall
assemblies report all six legal states. The right wheel produced both expected
directional cycles, including forward `2→3→1→5` and reverse
`2→6→4→5→1`. A follow-up audit found that the legacy phase-advanced reverse
table was physically proven only on the first wheel; the second-wheel evidence
explicitly said smooth multi-sector reverse rotation remained unproven. The
current firmware incorrectly promoted that table as a shared motor profile.
This unsupported firmware assumption is now the leading root cause—not a
missing Hall signal or a gross phase-order wiring error.

A slave-only symmetric reverse profile has been implemented and host-verified.
For each Hall state it swaps the source and sink of the already-working
positive vector while leaving the floating phase unchanged. The master retains
its physically proven phase-advanced reverse profile. The corrected slave
image has not been flashed because no ST-Link is presently connected.

Final live state:

- service active/running, `NRestarts=0`
- ESP32 connected and armed only after a fresh exact-zero session
- throttle `0`, steering `0`
- requested/mixed/commanded/applied `[0,0]`
- safety, master, slave, and balance faults `0`
- left and right bridges explicitly disabled
- feedback peer healthy; no transport-overflow flag
- CRC errors `0`; acknowledgment timeouts `0`

## Scope and physical constraints

- Both wheels were explicitly confirmed off the ground before any nonzero
  test.
- Motor demand was hard-limited to `-250..250`.
- Motion segments were bounded to 0.75–1.0 seconds.
- The production service was stopped before direct serial motor tests.
- Every direct test ended with exact zero, STOP, and DISARM; exception cleanup
  also sent EMERGENCY STOP and DISARM.
- No ground test was performed.
- No safety gate was bypassed.

## Hardware and runtime inventory

- Raspberry Pi 4 Model B Rev 1.5, Debian 12 Bookworm arm64.
- ESP32-D0WD-V3 revision 3.1, 4 MB flash, MAC
  `28:56:2f:4b:73:5c`.
- USB bridge: Silicon Labs CP2102, VID:PID `10c4:ea60`.
- Stable serial path:
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
- Camera: OV5647 through libcamera/Picamera2.
- Deployed source: `/opt/trashcan-robot`, service user `trashbot`.
- Service unit: `/etc/systemd/system/trashcan-donkeycar.service`, enabled.
- `trashbot` has `dialout`, `video`, `render`, `gpio`, `i2c`, and `input`
  supplementary groups.

Key installed versions:

- Python 3.11.2; pip 24.3.1
- Donkeycar 5.3.0
- pyserial 3.5
- Picamera2 0.3.31
- Flask 3.1.3; Tornado 6.5.7; Werkzeug 3.1.8
- numpy 1.26.4; psutil 7.2.2; PyYAML 6.0.3
- pytest 9.1.1
- Node.js 18.20.4; npm 9.2.0
- PlatformIO Core 6.1.19
- esptool.py 4.11.0
- rpicam-apps 1.9.0; libcamera 0.5.2+99-bfd68f78

The complete Python environment was inspected with:

```sh
/opt/trashcan-robot/donkeycar/.venv/bin/python -m pip freeze
```

## ESP32 firmware and recovery

The previous balance image was replaced with the required non-dry-run
`esp32_drive_coordinator`.

- Protocol version: 1
- Operating mode: drive mode 2
- Control rate: 100 Hz
- Motor transport rate: 10 Hz
- Maximum northbound payload: 48 bytes
- Application image: 308,816 bytes
- Application SHA-256:
  `30c9663263828eca9fe4e153bcb1490ff6540ddd9497a2a5664e21b93b9ad64f`

The application partition was independently read back after flashing and
matched the image exactly. Bootloader, partition table, and NVS were preserved.

Full pre-change 4 MB backup:

`/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/preflash-full-4mb.bin`

SHA-256:
`0274b21068ce34c472a41476793b487e80fa9766e2485e4570bf9c94c89982db`

Initial zero-session application readback:

`/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/readback-zero-session-firmware.bin`

Final Hall-telemetry application and byte-identical readback:

- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/controller-hall-telemetry.bin`
- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/controller-hall-telemetry-readback.bin`

## Implemented fixes

ESP32/GD32 safety path:

- Added a controller fault-clear relay that transmits exact zero with
  `DISABLE|CLEAR_FAULT` until both controller acknowledgments and zero fault
  words are observed.
- Fixed the zero-session deadlock: while disarmed, a tightly gated exact-zero
  command may bring both GD32 controllers from DISABLED to READY without
  energizing either bridge. Nonzero output still requires the full ARM gate.
- A nonzero demand invalidates the prior neutral observation, preventing a
  zero-then-nonzero pre-arm bypass.
- Separated the first-wheel phase-advanced reverse profile from the slave
  reverse profile. The candidate slave profile uses the exact opposing vector
  for each Hall state and does not alter logical polarity, Hall ordering, or
  the working master path. This change is built but not yet installed.

Pi serial/runtime path:

- Startup waits for exact zero acknowledgment, fresh healthy controller
  feedback, IMU health, serial ownership, and zero faults before ARM.
- Startup performs bounded fault clear and sequence-matched STATUS checks.
- Serial open holds DTR/RTS deasserted and retries HELLO.
- A protocol ERROR or nonzero safety-fault telemetry invalidates the session.
  The service disconnects, sends safe shutdown controls, and reconnects through
  a fresh zero-only handshake.
- Added a command-renewal worker to bridge short Donkeycar loop jitter. It only
  renews application input younger than 250 ms; stale input is not renewed.
  The ESP32's independent 500 ms movement lease remains unchanged.
- Added explicit decode of feedback peer health, controller/motor flags,
  left/right bridge-enabled state, PA4 state/bypass, clear-pending state, and
  transport-overflow state.
- Added a read-only controller telemetry packet carrying raw left/right Hall
  states, bridge flags, controller states, command/link ages, compare offsets,
  and remote framing counters.
- Added a bridge-disabled manual Hall-capture utility. It never sends ARM or
  movement and aborts if either bridge becomes enabled or a Hall state leaves
  the legal `1–6` range.

Web/runtime path:

- Patched Donkeycar websocket close handling to force angle `0`, throttle `0`,
  recording false, and manual/user mode before client removal.
- Added a real Tornado websocket verifier that captures live ESP32
  requested/mixed/commanded/applied telemetry, enforces the ±250 limit, waits
  for settled telemetry, and checks release/disconnect odometry independently.
- Fixed the lifted-wheel verifier so a final demand with zero Hall odometry is
  reported as failure instead of printing a misleading PASS label.

## Local source changes

All source changes remain local on the workstation and are mirrored as needed
under `/opt/trashcan-robot` on the Pi.

Modified source/test/config files:

- `LEGACY.md`
- `donkeycar/scripts/preflight.py`
- `donkeycar/tests/test_manual_drive_simulation.py`
- `donkeycar/tests/test_pipeline.py`
- `donkeycar/tests/test_preflight.py`
- `donkeycar/tests/test_protocol.py`
- `donkeycar/tests/test_transport.py`
- `donkeycar/trashcan_robot/pipeline.py`
- `donkeycar/trashcan_robot/protocol.py`
- `donkeycar/trashcan_robot/transport.py`
- `firmware/esp32/control/drive_safety.cpp`
- `firmware/esp32/control/drive_safety.h`
- `firmware/esp32/control/serial_command_source.cpp`
- `firmware/esp32/control/serial_messages.cpp`
- `firmware/esp32/control/serial_messages.h`
- `firmware/esp32/control/serial_protocol.h`
- `firmware/esp32/drive_coordinator/main.cpp`
- `firmware/gd32/common/motor/gs_commutation.c`
- `firmware/gd32/common/motor/gs_commutation.h`
- `firmware/gd32/common/motor/gs_motor_control.c`
- `firmware/gd32/common/motor/gs_motor_control.h`
- `firmware/gd32/slave/main.c`
- `platformio-drive.ini`
- `tests/native/test_control.c`
- `tests/esp32/test_balance.cpp`
- `tests/esp32/test_drive_safety.cpp`
- `tools/test-esp32-balance.sh`

New local source/report files:

- `donkeycar/scripts/runtime_safety_check.py`
- `donkeycar/scripts/capture_hall_movement.py`
- `donkeycar/scripts/safe_motion_bringup.py`
- `donkeycar/scripts/web_runtime_check.py`
- `donkeycar/tests/test_safe_motion_bringup.py`
- `donkeycar/tests/test_capture_hall_movement.py`
- `firmware/esp32/control/controller_fault_clear.cpp`
- `firmware/esp32/control/controller_fault_clear.h`
- `trashcan-robot-bringup-report.md`

Python cache files were also changed/generated by local test execution; they
were not staged or committed.

## Test results

- GD32 native suite: 760,216 assertions passed in normal and sanitizer builds.
- ESP32 host suite: 2,262 assertions passed in normal and sanitizer builds.
- Dedicated drive-safety suite passed in normal and sanitizer builds,
  including exact 45-degree limits, non-finite orientation, feedback loss,
  controller faults, lease expiry, non-neutral ARM rejection, and zero-session
  establishment.
- Node protocol/client: 25 tests passed.
- Pi Python application/simulation: **74 tests passed**.
- ESP32 PlatformIO build passed: RAM 23,500/327,680 (7.2%), flash
  308,457/1,310,720 (23.5%).
- Candidate slave GD32 build passed: RAM 1,036/8,192, flash 11,852/65,536;
  firmware binary 11,860 bytes.
- `git diff --check` passed.

The pytest cache warning is non-functional: the invoking `pi` account could not
write `/opt/trashcan-robot/donkeycar/.pytest_cache`; all 74 tests passed.

## Safety and reconnect evidence

Exact-zero preflight:

```text
PREFLIGHT PASS state=4 mode=2 source=2 health=0x7d control_hz=100 motor_hz=10 overruns=0 rejected=0
```

- ARM before zero: rejected with unsafe-state error 6.
- Movement lease expiry: fault `0x2`, state 6, calculated/applied `[0,0]`.
- Emergency stop: acknowledged.
- Invalid/non-finite/tilt inputs: rejected or faulted closed in host tests.
- Service restart after a transient feedback-freshness fault remained at exact
  zero, then automatically cleared/reconnected through the safe handshake.
- Controlled USB detach/reconnect test used the sole CP2102 sysfs node
  `/sys/bus/usb/devices/1-1.1`:
  - stable serial path disappeared
  - application reported disconnected with zero controls
  - stable path returned
  - fresh HELLO/mode/zero/clear/ARM completed
  - no stale command was replayed
  - ending requested, calculated, and applied values were `[0,0]`
- Motor feedback during successful motion had CRC errors `0` and
  acknowledgment timeouts `0`.
- Zero-state feedback word `0x00000405` decodes to peer healthy, no transport
  overflow, both bridges disabled, slave PA4 high, and master PA4 bypass
  active.
- Controller timing evidence at final zero: approximately 49–60 ms last
  acknowledgment latency and 149–160 ms last apply latency; 10 Hz transmit
  rate.
- Bridge-disabled manual-spin capture:
  - first capture observed 45 changes on the left Hall channel and 84 on the
    right
  - right-wheel-only capture observed 239 right Hall changes while the left
    Hall channel remained fixed
  - all Hall values were legal states `1–6`
  - both directional cycles matched the GD32 transition tables
  - controller states remained disabled during capture
  - both bridge flags and both compare offsets remained zero
  - master command, slave feedback, and slave command ages remained bounded
  - the cumulative remote framing counter was 1462 and did not increase during
    the final slow bidirectional capture
- MPU6050: address `0x68`, calibrated/valid, no I2C errors, sample age in the
  low milliseconds, measured pitch approximately -28 degrees, below the
  45-degree shutdown limit.

## Lifted-wheel motion results

| Test | Logical command | Duration | Odometry delta | Result |
|---|---:|---:|---:|---|
| Low-output trials | 25–100 | ≤0.75 s | `[0,0]` | No startup; below observed torque threshold |
| Left forward | `[250,0]` | 0.75 s | `[+24,0]` | Pass |
| Right forward, original | `[0,250]` | 0.75 s | `[0,0]` | Fail; startup timeout `0x8` |
| Right reverse | `[0,-250]` | 0.75 s | `[0,-26]` | Pass |
| Both reverse | `[-250,-250]` | 0.75 s | `[-27,-27]` | Pass |
| Opposite-wheel steering | `[250,-250]` | 0.75 s | `[+22,-25]` | Pass |
| Right forward after power cycle | `[0,250]` | 0.75 s | `[0,0]` | **Fail; bridge enabled, no Hall odometry** |
| Right forward after power cycle | `[0,250]` | 1.0 s | `[0,0]` | **Fail; applied demand observed, no Hall odometry** |
| Both forward | `[250,250]` | — | — | Not accepted; right-forward unresolved |

Post-power-cycle right-forward logs:

- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/right-forward-after-motor-reboot-075s.json`
- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/right-forward-after-motor-reboot-1s.json`

Other motion logs:

- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/safe-motion-250.json`
- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/right-reverse-250.json`
- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/both-reverse-250.json`
- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/opposite-steering-250.json`

Bridge-disabled Hall evidence:

- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/manual-hall-capture-both.json`
- `/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/manual-hall-capture-right-bidirectional.json`

Final Hall log SHA-256:
`318656d90c629eb71a68310466348c51a51782d2e6b399ee12697631705834e5`

## Real web-control evidence

Actual Donkeycar/Tornado websocket tests used steering demand `1.0`, which the
ESP32 bounded to logical wheels `[250,-250]`.

Explicit release:

- seven live nonzero observations
- requested yaw `0.8`
- mixed/commanded `[250,-250]`
- peak sampled applied `[170,-167]`
- odometry delta `[+26,-30]`
- ending calculated/applied `[0,0]`

Abrupt websocket disconnect, without sending a release message:

- eight live nonzero observations
- requested yaw `0.8`
- mixed/commanded `[250,-250]`
- peak sampled applied `[207,-200]`
- odometry delta `[+25,-27]`
- close handler returned application demand to exact zero
- ending calculated/applied `[0,0]`

Both paths had zero balance/master/slave faults, zero CRC errors, and zero
acknowledgment timeouts. The service subsequently settled armed at exact zero
with both bridges disabled.

Evidence:

`/opt/trashcan-robot/logs/bringup-20260728-esp32-drive/web-runtime-check.json`

SHA-256:
`780a1ccc1ea5fcd78891ccd13f9c2cb574e4e83d867904b6c88b0721e4b1fbd1`

## Camera, LAN, and service

- Independent camera capture: valid 640×480 JPEG.
- Dashboard: HTTP 200.
- Donkeycar drive UI: HTTP 200.
- Dashboard camera endpoint: HTTP 200, valid 640×480 JPEG.
- Live camera rate: approximately 19.95–19.97 FPS.
- Dashboard, drive UI, and camera were fetched successfully through the LAN IP
  `192.168.2.47`, not only loopback.
- `systemd-analyze verify` passed.
- Stop/start/restart completed cleanly.
- Final service: enabled, active/running, `NRestarts=0`, `ExecMainStatus=0`.

LAN URLs:

- Dashboard: `http://192.168.2.47:8888/`
- Drive controller: `http://192.168.2.47:8887/drive`

## Operations

Inspect:

```sh
ssh pi@192.168.2.47
systemctl status trashcan-donkeycar.service --no-pager
curl -fsS http://127.0.0.1:8888/api/state
journalctl -u trashcan-donkeycar.service -n 100 --no-pager
```

Start/stop/restart:

```sh
sudo systemctl start trashcan-donkeycar.service
sudo systemctl stop trashcan-donkeycar.service
sudo systemctl restart trashcan-donkeycar.service
```

Run the zero-only preflight:

```sh
sudo systemctl stop trashcan-donkeycar.service
cd /opt/trashcan-robot/donkeycar
.venv/bin/python scripts/preflight.py --config config/robot.yaml
sudo systemctl start trashcan-donkeycar.service
```

Run a bounded lifted-wheel diagnostic only after physically confirming both
wheels are lifted:

```sh
sudo systemctl stop trashcan-donkeycar.service
cd /opt/trashcan-robot/donkeycar
.venv/bin/python scripts/safe_motion_bringup.py \
  --confirm-wheels-lifted \
  --levels 250 \
  --maneuvers right-forward \
  --duration 1.0 \
  --log /tmp/right-forward.json
sudo systemctl start trashcan-donkeycar.service
```

Do not run the motion command on the floor. Do not run direct serial tests while
the service is active.

## Backups and rollback

Source backups exist under `/opt/trashcan-robot/backups`, including:

- `20260728T132337Z-drive-source`
- `20260728T141800Z-zero-session`
- `20260728T143000Z-safe-motion`
- `20260728T145000Z-production-handshake`
- `20260728T160000Z-websocket-neutral`
- `20260728T141000Z-lease-renewal`
- `20260728T141500Z-feedback-decode`
- `20260728T143000Z-controller-hall-telemetry`

ESP32 recovery artifacts and flash/readback evidence are in:

`/opt/trashcan-robot/logs/bringup-20260728-esp32-drive`

Unflashed slave-correction artifact:

`/opt/trashcan-robot/artifacts/gausstop_slave_symmetric_reverse_20260728`

Candidate slave firmware SHA-256:
`64e027e84af7fb8428f63eedbd19fc5cb8e34290b2cc688451ae2017dfcf5ffe`

## Required next action and remaining risk

Do not change logical polarity or Hall transition ordering: both are supported
by live evidence. Fully power down and disconnect the motor battery, then
connect an ST-Link to the slave controller using GND, SWCLK, SWDIO, and NRST
only; leave both ST-Link power pins disconnected. Preserve and verify two
identical 64 KiB reads of the current slave flash before writing.

Flash and read back the candidate slave image, disconnect the ST-Link,
power-cycle, and repeat only right-forward with wheels lifted at command `250`
for no more than 1.0 second. Require the expected physical direction, positive
logical right odometry, both bridges returning off, and zero faults/counters
before attempting both-forward. If the candidate does not start cleanly,
restore the preserved slave image and do not cycle through arbitrary vectors.

**Floor testing remains prohibited until right-forward and both-forward pass
with correct physical direction and a verified final STOP/DISARM.**
