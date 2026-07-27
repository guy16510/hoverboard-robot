# Raspberry Pi Donkeycar Bring-Up Report

## Summary

Initial readiness classification: **PI_CAMERA_READY**

Follow-up bring-up completed later on 2026-07-27. The current classification is
**PI_SOFTWARE_READY / ESP32_PHYSICALLY_BLOCKED**. The controller, dashboard,
production camera, state API, tub recording, rate-limited logging, systemd
lifecycle, and safe no-ESP32 behavior now pass. See
`docs/raspberry-pi-final-validation-2026-07-27.md` for the final evidence,
rollback instructions, Git invariants, and exact next physical action.

The 2026-07-27 bring-up progressed through SSH access, Raspberry Pi system-health
checks, independent camera enumeration, still capture, video capture, Python
dependency/import checks, the Donkeycar application unit-test suite, and bounded
LAN reachability testing. The OV5647 camera produced a decodable 2592x1944 JPEG
and a decodable 640x480 H.264 stream when run as `trashbot`.

The integrated drive application did not become operational.
`trashcan-donkeycar.service` repeatedly exits in
`trashcan_robot/pipeline.py:92` because the installed Donkeycar 5.3.0
`TubWriter` constructor accepts `base_path`, not `path`. Consequently, the
process never remains alive, ports 8887 and 8888 do not listen, no tub is
recorded, and no dashboard telemetry is displayed.

No ESP32 was physically connected to the Raspberry Pi. No zero-motion protocol
test, zero-demand soak, lifted-wheel motion test, safety-shutdown test,
flat-ground test, or driveway test was performed during this session. No
nonzero motion command was sent. Historical ESP32 and lifted-wheel evidence
already present in the repository predates this session and is not counted as
evidence for the July 27 gates.

## Environment

Evidence was collected over SSH from `2026-07-27T10:15:23-04:00` through
`2026-07-27T10:31:30-04:00`.

| Item | Observed value |
|---|---|
| Hostname | `trashcan-robot` (`trashcan-robot.local` resolved over mDNS) |
| IPv4 | `192.168.2.44/24` on `eth0` |
| IPv6 | `fd09:4b84:4a90:e857:dd98:f58f:95f0:eedf/64` and `fe80::d6c4:f532:3f0f:8fa4/64` on `eth0` |
| Operating system | Debian GNU/Linux 12 (bookworm) |
| Architecture | `arm64` / `aarch64` |
| Kernel | `6.12.93+rpt-rpi-v8`, build `#1 SMP PREEMPT Debian 1:6.12.93-1+rpt1 (2026-06-12)` |
| Raspberry Pi model | Raspberry Pi 4 Model B Rev 1.5 |
| Active network interface | `eth0`, default route via `192.168.2.1` |
| Wi-Fi | `wlan0` was `DOWN`; no Wi-Fi SSID applied |
| Time zone | EDT (`-04:00`, configured zone `America/New_York`) |
| Root filesystem | `/dev/mmcblk0p2`, 28 GiB displayed capacity, 4.0 GiB used, 23 GiB available, 16% used |
| Boot filesystem | `/dev/mmcblk0p1`, 512 MiB, 13% used |
| Memory | 3.7 GiB total, 270 MiB used, 3.0 GiB free, 3.4 GiB available |
| Swap | 511 MiB total, 0 used |
| CPU temperature | `49.6'C` |
| Throttling/undervoltage | `vcgencmd get_throttled` returned `throttled=0x0`; no current or historical throttle/undervoltage bits were set |

`uptime -s` reported `2026-07-27 09:12:23`. The persistent boot record began
with an uncorrected clock value of `2026-07-26 16:46:09`; later journal entries
use the corrected July 27 time. No reboot was initiated during this evidence
pass.

## Repository State

### Raspberry Pi deployment tree

- Expected and observed deployment path: `/opt/trashcan-robot`
- Expected and observed Donkeycar application path:
  `/opt/trashcan-robot/donkeycar`
- The deployment tree contains no `.git` directory.
- `git remote -v`, `git branch --show-current`, `git rev-parse HEAD`, and
  `git status --short`, all run from `/opt/trashcan-robot`, returned:
  `fatal: not a git repository (or any of the parent directories): .git`.
- Therefore the Raspberry Pi tree's Git remote, branch, current commit, initial
  status, and final status are **BLOCKED / unavailable from the deployed
  snapshot**. No commit is inferred from file timestamps.
- Firmware manifests under `/opt/trashcan-robot/dist/*/manifest.txt` contain
  `git_commit=38c6d3fdacdd01f6c62454d3279e60803953629d`. Those manifests describe
  their generated firmware artifacts and are not proof of the Donkeycar
  application snapshot's commit.

### Documentation repository on the controller computer

- Repository path: `/Users/burnschris/Git/hoverboard-robot`
- Git remote:
  `origin git@github.com:guy16510/hoverboard-robot.git`
- Branch: `main`
- Commit: `0a431191524ac53d2f6540f7e8c6c7c84784731e`
- Initial `git status --short`: empty output (clean)
- Final `git status --short`:

  ```text
  ?? docs/raspberry-pi-bringup-2026-07-27.md
  ```

No commit, push, branch, or pull request was created. No tracked source file was
modified.

## Raspberry Pi Health

### Service state

- Unit: `trashcan-donkeycar.service`
- Unit file: `/etc/systemd/system/trashcan-donkeycar.service`
- Enablement: `enabled` with vendor preset `enabled`
- Execution user/group: `trashbot:trashbot`
- Supplementary groups declared by the unit:
  `dialout video render gpio i2c input`
- Working directory: `/opt/trashcan-robot/donkeycar`
- Process command:

  ```sh
  /opt/trashcan-robot/donkeycar/.venv/bin/python \
    /opt/trashcan-robot/donkeycar/manage.py drive \
    --config /opt/trashcan-robot/donkeycar/config/robot.yaml
  ```

At `2026-07-27T10:15` the unit was `activating (auto-restart)`, the prior main
process had exited `status=1/FAILURE`, and `NRestarts=603`. At `10:20`,
`NRestarts=644`. After the diagnostic camera stop/start was reversed, a final
sample showed:

```text
MainPID=0
Result=exit-code
NRestarts=672
ExecMainStatus=1
ActiveState=activating
SubState=auto-restart
```

The first corrected-time failure was recorded at `2026-07-27 09:13:05 EDT`.
The repeated relevant journal error is:

```text
File "/opt/trashcan-robot/donkeycar/trashcan_robot/pipeline.py", line 92, in build_vehicle
    tub = TubWriter(path=str(tub_path), inputs=tub_inputs, types=tub_types)
TypeError: TubWriter.__init__() got an unexpected keyword argument 'path'
```

`systemctl --failed` displayed zero failed units because this unit remains in
the `activating/auto-restart` state rather than settling in `failed`. The
application service is nevertheless unhealthy.

Other active/enabled services checked were `ssh.service`,
`NetworkManager.service`, and `ModemManager.service`. Boot-level error-priority
journal entries were limited to Bluetooth VCP/MCP/BAP/SAP plugin failures and
one `wpa_supplicant` nl80211 registration warning; none affected the active
Ethernet or SSH test.

### Hardware and permissions

`lsusb` showed only the two Linux root hubs and a VIA Labs USB 2.0 hub:

```text
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

Camera devices included `/dev/video0`, `/dev/video10` through relevant codec
and ISP nodes, and `/dev/media0` through `/dev/media4`. `v4l2-ctl
--list-devices` mapped `/dev/video0` and `/dev/media4` to `unicam`.

No `/dev/ttyACM*`, `/dev/ttyUSB*`, or `/dev/serial/by-id/*` path existed.

The interactive user was:

```text
uid=1000(pi) gid=1000(pi) groups=1000(pi),4(adm),20(dialout),24(cdrom),27(sudo),29(audio),44(video),46(plugdev),60(games),100(users),102(input),105(render),110(netdev),993(gpio),994(i2c),995(spi)
```

The service user was:

```text
uid=995(trashbot) gid=992(trashbot) groups=992(trashbot),20(dialout),44(video),102(input),105(render),994(i2c),993(gpio)
```

All files checked under `/opt/trashcan-robot` were owned
`trashbot:trashbot`. The `trashbot` account could write the tub, log, and model
directories. Power, storage, and memory checks were healthy: `throttled=0x0`,
23 GiB free on `/`, 3.4 GiB available RAM, and no kernel I/O-error or
undervoltage message was found.

## Camera Validation

Status: **PASS for functional capture; user visual-quality review NOT TESTED**

The detected sensor was:

```text
0 : ov5647 [2592x1944 10-bit GBRG] (/base/soc/i2c0mux/i2c@1/ov5647@36)
```

Enumeration command:

```sh
rpicam-hello --list-cameras
```

It advertised 640x480, 1296x972, 1920x1080, and 2592x1944 modes. No
enumeration error occurred.

### Still image

The failing application service was temporarily stopped to avoid repeated
camera acquisition. The still was then captured as `trashbot`:

```sh
sudo -u trashbot rpicam-still \
  --nopreview --timeout 2000 \
  --width 2592 --height 1944 \
  --output /tmp/codex-camera-still-20260727.jpg
```

Observed result:

- Capture timestamp: `2026-07-27T10:18:43-04:00`
- Requested dimensions: 2592x1944
- Observed dimensions: 2592x1944
- File size: 1,301,238 bytes
- Ownership: `trashbot:trashbot`
- Format: baseline 8-bit, three-component JPEG
- Pillow verification and decode: successful
- Mean RGB: `(23.64, 23.67, 18.12)`
- Channel extrema: `((0, 85), (0, 93), (0, 95))`
- All black: `False`
- All white: `False`
- Uniform frame: `False`
- Camera errors: none during this independent capture

The low mean values indicate a dark captured scene, but they do not establish
whether the camera exposure or physical scene was appropriate.

### Video

Video command, run as `trashbot`:

```sh
sudo -u trashbot rpicam-vid \
  --nopreview --timeout 5000 \
  --width 640 --height 480 \
  --framerate 30 --codec h264 \
  --metadata /tmp/codex-camera-video-20260727.json \
  --metadata-format json \
  --output /tmp/codex-camera-video-20260727.h264
```

Observed result:

- Start timestamp: `2026-07-27T10:19:08-04:00`
- Requested duration: 5,000 ms
- Program completion: `Halting: reached timeout of 5000 milliseconds.`
- Requested frame rate: 30 fps
- Observed stream: H.264, 640x480, `r_frame_rate=30/1`,
  `avg_frame_rate=30/1`
- Frames decoded/countable by `ffprobe`: 146
- Approximate emitted rate across the requested five-second interval:
  29.2 frames/s
- Per-frame metadata `FrameDuration`: 33,302 microseconds for the first and
  last entries
- Sensor timestamp span: 4.552933 seconds
- H.264 file size: 3,216,527 bytes
- Metadata entries: 146
- Camera errors: none during this independent capture

The service was restored with:

```sh
sudo systemctl start trashcan-donkeycar.service
```

It resumed the same unrelated `TubWriter` restart loop. The three temporary
camera files were deleted after their dimensions, sizes, decode results, and
statistics were collected.

The camera worked as `trashbot`. The user did **not** confirm orientation,
brightness, focus, latency, or field of view, so those five visual/physical
criteria remain **NOT TESTED**.

## Python and Donkeycar Runtime Validation

Status: **unit-level PASS; integrated runtime FAIL**

- Python: `Python 3.11.2`
- Virtual environment:
  `/opt/trashcan-robot/donkeycar/.venv`
- Donkeycar: 5.3.0
- Direct dependency versions checked:
  Flask 3.1.3, pyserial 3.5, PyYAML 6.0.3, psutil 7.2.2, pytest 9.1.1

Current dependency check:

```sh
sudo -u trashbot \
  /opt/trashcan-robot/donkeycar/.venv/bin/python -m pip check
```

Result:

```text
No broken requirements found.
```

The current import/config/protocol smoke check imported `flask`, `psutil`,
`serial`, `yaml`, `donkeycar`, `load_config`, and `crc16_ccitt_false`, then
verified serial baud 115200 and CRC-16/CCITT-FALSE value `0x29B1` for
`123456789`. It printed:

```text
Raspberry Pi image Python smoke test passed
```

Current application test command:

```sh
sudo -u trashbot env \
  PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH=/opt/trashcan-robot/donkeycar \
  /opt/trashcan-robot/donkeycar/.venv/bin/python \
  -m pytest -q -p no:cacheprovider \
  /opt/trashcan-robot/donkeycar/tests
```

Result:

```text
..........                                                               [100%]
10 passed in 0.59s
```

No test failure occurred in that ten-test suite. This result does not override
the real application startup failure.

The installed constructor signature was inspected directly:

```text
(base_path, inputs=[], types=[], metadata=[], max_catalog_len=1000)
```

The application supplies the unsupported keyword `path`, proving the runtime
incompatibility.

Filesystem ownership under `/opt/trashcan-robot` is `trashbot:trashbot`.
`/opt/trashcan-robot/donkeycar/data/tubs`,
`/opt/trashcan-robot/donkeycar/data/logs`, and
`/opt/trashcan-robot/donkeycar/models` are mode 0755, owned by `trashbot`, and
returned success from `sudo -u trashbot test -w`.

## Web Controller Validation

Status: **FAIL**

- Process command: the `manage.py drive` command shown in Raspberry Pi Health
- Configured port: 8887
- Bind address: not explicitly set in `robot.yaml`; no listener was observed,
  so a runtime bind address was not claimed
- `ss -lntp | grep -E ':8887|:8888'`: empty output

Local Pi HTTP command and result:

```sh
curl --silent --show-error --max-time 3 \
  --output /dev/null \
  --write-out 'status=%{http_code} connect=%{time_connect} total=%{time_total}\n' \
  http://127.0.0.1:8887/
```

```text
curl: (7) Failed to connect to 127.0.0.1 port 8887
status=000 connect=0.000000 total=0.001730
```

Controller-computer HTTP result from `192.168.2.10` to `192.168.2.44`:

```text
curl: (7) Failed to connect to 192.168.2.44 port 8887
status=000 connect=0.000000 total=0.031420
```

No user phone/tablet test was performed. Controls could not be loaded and no
control values were changed. The process did not stay alive; it exited at
vehicle construction before `vehicle.start(...)`.

## Dashboard Validation

Status: **FAIL**

- Configured bind address: `0.0.0.0`
- Configured port: 8888
- No listener was observed.

Local Pi HTTP result:

```text
curl: (7) Failed to connect to 127.0.0.1 port 8888
status=000 connect=0.000000 total=0.000294
```

Controller-computer HTTP result from `192.168.2.10`:

```text
curl: (7) Failed to connect to 192.168.2.44 port 8888
status=000 connect=0.000000 total=0.011398
```

The dashboard code is intended to serve `/`, `/api/state`, and `/camera.jpg`,
and the browser code intends to refresh state and camera every 500 ms. None of
that update behavior was observed because `DashboardServer.start()` occurs
after the failing `TubWriter` construction.

No live camera image or telemetry was displayed. Intended state fields include
timestamp, mode, recording, steering, throttle, ESP32 connection, FPS,
inference rate, serial latency, telemetry, faults, and model name, but no live
values were received from port 8888. No remote user-device test was performed.

## Tub Recording Validation

Status: **BLOCKED**

- Tub root: `/opt/trashcan-robot/donkeycar/data/tubs`
- Intended per-run path:
  `/opt/trashcan-robot/donkeycar/data/tubs/tub_<UTC timestamp>`
- Records: 0
- Images: 0
- Files: 0
- Intended fields from `pipeline.py`:
  `cam/image_array`, `user/angle`, `user/throttle`, `user/mode`
- Sample steering values: none
- Sample throttle values: none
- Root ownership/mode: `trashbot:trashbot`, 0755
- Corrupted files: none found
- Empty files: none found
- Values changing with user input: not tested; the web controller was
  unavailable

The `TubWriter` constructor failed before a tub could be created.

## Network Validation

Status: **PASS for the bounded current LAN test; operating-location test NOT
TESTED**

- Raspberry Pi interface: `eth0`
- Raspberry Pi address: `192.168.2.44/24`
- Controller computer interface: `en0`
- Controller computer address: `192.168.2.10`
- Route: direct on `192.168.2.0/24`
- Pi Wi-Fi: `wlan0` down; Wi-Fi signal and SSID not applicable

Bounded reachability command:

```sh
ping -c 20 192.168.2.44
```

Result:

```text
20 packets transmitted, 20 packets received, 0.0% packet loss
round-trip min/avg/max/stddev = 3.474/8.110/52.001/10.302 ms
```

SSH remained reachable throughout evidence collection and closed normally at
the end. Ports 8887 and 8888 never became reachable because the application
was not listening; this was an application failure, not packet loss.

The user did not confirm that the robot was in the driveway or other intended
operating location. Network performance at the actual operating location is
therefore **NOT TESTED**.

## ESP32 Detection

Status: **BLOCKED — ESP32 was not physically connected**

Because the ESP32 was absent, this section is not marked completed.

- USB VID: not observed
- USB PID: not observed
- Manufacturer: not observed
- Product: not observed
- Serial number: not observed
- Kernel device path: none
- `/dev/serial/by-id/` path: none; directory did not exist
- Permissions: not applicable; no device node
- Process conflicts: none could exist on a missing device node
- ModemManager: active and enabled; `mmcli -L` returned
  `No modems were found`
- `/dev/ttyACM0`: did not exist and therefore did not match an actual device

The only USB peripheral enumerated beyond the root hubs was the VIA Labs
`2109:3431` USB hub.

This enumeration was repeated after the user offered to connect the ESP32.
Results at `2026-07-27T10:30:00-04:00`,
`2026-07-27T10:30:34-04:00`, and
`2026-07-27T10:31:30-04:00` were unchanged: the ESP32 had not appeared on USB
and no serial node had been created.

## Zero-Motion Protocol Validation

Status: **BLOCKED — no ESP32 serial device**

No protocol session was opened and no command was sent.

| Required observation | July 27 observed value |
|---|---|
| Protocol version | Not observed |
| Dry-run flag | Not observed |
| Advertised operating modes | Not observed |
| Selected mode | Not observed |
| State | Not observed |
| Health flags | Not observed |
| Fault mask | Not observed |
| Control rate | Not observed |
| Motor rate | Not observed |
| Rejected frames | Not observed |
| Loop overruns | Not observed |
| STOP result | Not sent / not observed |
| DISARM result | Not sent / not observed |

None of the required predicates passed during this session:

```text
protocol == 1                 NOT TESTED
dry_run == false              NOT TESTED
mode == 2                     NOT TESTED
state == 4                    NOT TESTED
faults == 0                   NOT TESTED
IMU healthy                   NOT TESTED
motor feedback fresh          NOT TESTED
armed health present          NOT TESTED
serial health present         NOT TESTED
```

No existing `PREFLIGHT PASS` output was used.

Historical repository files
`logs/esp32-zero-demand-gate.jsonl`,
`logs/esp32-zero-demand-gate-settled.jsonl`, and
`logs/hardware-validation-20260723.md` predate this session. Among other
values, they contain protocol 2, peer health false, and a historical MASTER
fault 8192. They are not proof of the required protocol 1 conditions on the
current Raspberry Pi and do not advance this gate.

## Zero-Demand Soak

Status: **NOT TESTED**

- Duration: 0 seconds
- Command rate: not applied
- Lease duration: configured value is 500 ms, but not exercised
- Initial counters: not observed
- Final counters: not observed
- Fault changes: not observed
- Rejected-frame changes: not observed
- Overrun changes: not observed
- Feedback freshness: not observed
- Unexpected disarms: not observed
- Observed wheel motion: no physical observation was requested or received
- Final STOP: not sent
- Final DISARM: not sent

No motion command, including a zero-demand serial stream, was sent because no
ESP32 serial device existed.

## Lifted-Wheel Motor Validation

Status: **NOT TESTED**

| Check | Left wheel | Right wheel |
|---|---|---|
| Smallest forward input | Not sent | Not sent |
| Smallest reverse input | Not sent | Not sent |
| Physical forward direction | Not observed | Not observed |
| Physical reverse direction | Not observed | Not observed |
| Steering-left behavior | Not observed | Not observed |
| Steering-right behavior | Not observed | Not observed |
| User physical confirmation | Not received | Not received |
| Telemetry result | Not observed | Not observed |
| Noise/vibration/oscillation/acceleration | Not observed | Not observed |

The user did not confirm that the wheels were lifted. Historical July 24
lifted-wheel evidence in `docs/remote-hardware-handoff.md` is not a July 27
physical test and is not counted here.

## Safety Shutdown Validation

No safety behavior was tested against a connected, armed drive system.

| Safety case | Status | Exact observed behavior |
|---|---|---|
| Throttle release | NOT TESTED | Web control unavailable; no throttle applied |
| Donkeycar process termination | NOT TESTED | Process crashed during construction before serial drive; this is not a valid motor-safety termination test |
| Pi USB disconnect | NOT TESTED | No ESP32 USB connection existed |
| Command lease expiration | NOT TESTED | No protocol session |
| Missing GD32 feedback | NOT TESTED | No ESP32 telemetry session |
| Local disarm | NOT TESTED | No DISARM command sent |
| Excessive tilt | NOT TESTED | No July 27 armed system |
| Serial reconnection | NOT TESTED | No serial device |
| ESP32 reboot | NOT TESTED | ESP32 absent |
| Raspberry Pi process crash | NOT TESTED | The startup crash occurred before motor connection and cannot establish shutdown behavior |

## Flat-Ground Test

Status: **NOT TESTED**

- Trash can attached/detached: not recorded
- Surface: not recorded
- Incline: not recorded
- Speed limit: not set or exercised
- Operator: not recorded
- Spotter: not recorded
- Emergency disconnect: not confirmed
- Forward control: not tested
- Reverse control: not tested
- Steering: not tested
- Stopping distance: not measured
- Downhill braking: not tested
- Network stability on the test surface: not tested
- Faults: no drive telemetry available
- Abort reason: prerequisites were not met; Donkeycar runtime failed and no
  ESP32 was connected

## Driveway Test

Status: **NOT TESTED**

- Test type: none; neither manual nor autonomous
- Trash can attached/detached: not recorded
- Surface: not recorded
- Incline: not recorded
- Speed limit: not set or exercised
- Operator: not recorded
- Spotter: not recorded
- Emergency disconnect: not confirmed
- Forward/reverse/steering: not tested
- Stopping distance and downhill braking: not measured
- Network stability in the driveway: not tested
- Faults: no drive telemetry available
- Abort reason: prerequisite gates were incomplete

The first future driveway test remains required to be manual unless the user
explicitly approves otherwise.

## Changes Made on the Raspberry Pi

This list distinguishes image-provisioned state, an earlier interactive action,
and changes made only for this evidence pass. No source file was edited on the
Pi.

### Application snapshot deployed by the prebuilt image

Reason: Install the robot repository payload in the custom Raspberry Pi image.

Command:

```sh
install -d -m 0755 "${ROOTFS_DIR}/opt/trashcan-robot"
rsync -a --delete \
  "${STAGE_DIR}/00-install/files/repository/" \
  "${ROOTFS_DIR}/opt/trashcan-robot/"
```

Previous state: absent in the base image.

New state: `/opt/trashcan-robot`, owned recursively by
`trashbot:trashbot` after final provisioning. It is a deployment snapshot
without `.git`.

Reversible: yes, by replacing/removing the deployed snapshot or reflashing.

### Operating-system packages requested by the robot image stage

Reason: Provide Python, camera, build, joystick, media, and linear-algebra
runtime support.

Command: pi-gen installed the exact package names listed in
`raspberry-pi-image/stage-trashcan/00-install/00-packages`:

```text
python3 python3-venv python3-dev python3-pip python3-picamera2
libcamera-dev build-essential git rsync joystick ffmpeg
libopenblas-dev libatlas-base-dev libhdf5-dev
```

Previous state: base-image package state; exact individual presence was not
captured.

New state: packages present in the generated image. Checked versions included
Python 3.11.2, `python3-picamera2` 0.3.31-1, `libcamera-dev`
0.5.2+rpt20250903-1~bpo12+1, `build-essential` 12.9, Git
2.39.5-0+deb12u3, and `joystick` 1.8.1-1.

Reversible: yes through package removal or image replacement, with dependency
impact.

### Robot service account and groups

Reason: Run the drive service without root while permitting camera, serial,
rendering, GPIO, I2C, and input access.

Command:

```sh
groupadd --system trashbot
useradd --system --create-home --gid trashbot \
  --shell /usr/sbin/nologin trashbot
usermod -aG dialout,video,render,gpio,i2c,input trashbot
```

The actual build script performs one `usermod -aG` call per existing group.

Previous state: `trashbot` absent from the base image.

New state: system UID 995, group 992, supplementary groups
`dialout video input render i2c gpio`.

Reversible: yes by removing memberships/account after disabling the service.

### Python virtual environment and Python packages

Reason: Install the pinned Donkeycar application runtime.

Command:

```sh
python3 -m venv --system-site-packages \
  /opt/trashcan-robot/donkeycar/.venv
/opt/trashcan-robot/donkeycar/.venv/bin/python \
  -m pip install --upgrade 'pip<25' wheel setuptools
/opt/trashcan-robot/donkeycar/.venv/bin/python \
  -m pip install --prefer-binary \
  -r /opt/trashcan-robot/donkeycar/requirements.txt
```

Previous state: no application virtual environment.

New state: Python 3.11.2 environment with Donkeycar 5.3.0, Flask 3.1.3,
pyserial 3.5, PyYAML 6.0.3, psutil 7.2.2, pytest 9.1.1, and transitive
dependencies.

Reversible: yes by deleting/recreating the virtual environment.

### Writable application directories

Reason: Provide storage for tubs, logs, and models.

Command:

```sh
install -d -o trashbot -g trashbot -m 0755 \
  /opt/trashcan-robot/donkeycar/data/tubs \
  /opt/trashcan-robot/donkeycar/data/logs \
  /opt/trashcan-robot/donkeycar/models
```

Previous state: absent before image customization.

New state: empty, writable 0755 directories owned `trashbot:trashbot`.

Reversible: yes after preserving desired data.

### systemd unit installation and enablement

Reason: Start the Donkeycar drive service at boot.

Command:

```sh
install -m 0644 \
  /opt/trashcan-robot/donkeycar/systemd/trashcan-donkeycar.service \
  /etc/systemd/system/trashcan-donkeycar.service
systemctl enable trashcan-donkeycar.service
```

Previous state: unit absent/disabled in the base image.

New state: unit installed and enabled. At runtime it is in an auto-restart
failure loop.

Reversible: yes with `systemctl disable --now
trashcan-donkeycar.service` followed by unit removal and daemon reload.

### Image validation marker files

Reason: Record that build-time architecture, dependencies, Python smoke tests,
application tests, systemd validation, and ownership checks passed.

Command: the image build wrote `/etc/trashcan-robot-build-info` and touched
`/etc/trashcan-robot-image-validated` after its checks.

Previous state: absent.

New state: both marker files present. The build-info file says
`python_smoke=passed`, `application_tests=passed`,
`systemd_verify=passed`, `dependency_check=passed`, and
`repository_owner=trashbot:trashbot`.

Reversible: yes by deleting the marker files, though doing so would remove the
image's validation record.

### Interactive `pi` password change before evidence collection

Reason: not recorded; presumably first-login credential replacement.

Command: `passwd` was the only state-changing command present in the prior
interactive shell history.

Previous state: credential value not inspected and not recorded.

New state: `passwd -S pi` reported the password last changed on `2026-07-27`;
`/etc/shadow` was modified at `2026-07-27 09:26:19-04:00`.

Reversible: yes with another authorized password change. No password or
credential is included in this report.

### Camera diagnostic service stop/start

Reason: Prevent the restart-looping application from repeatedly acquiring the
camera during independent still and video tests.

Command:

```sh
sudo systemctl stop trashcan-donkeycar.service
sudo systemctl start trashcan-donkeycar.service
```

Previous state: enabled and repeatedly auto-restarting after exit status 1.

New state: restored to enabled/start-requested state and the same auto-restart
loop. No unit file or enablement setting changed.

Reversible: already reversed by the start command.

### Temporary camera evidence files

Reason: Validate still/video capture, dimensions, decoding, and frame rate.

Command: the `rpicam-still` and `rpicam-vid` commands documented in Camera
Validation created:

```text
/tmp/codex-camera-still-20260727.jpg
/tmp/codex-camera-video-20260727.h264
/tmp/codex-camera-video-20260727.json
```

Previous state: files absent.

New state: files were created as `trashbot:trashbot` and analyzed. The first
cleanup attempt, run as `pi`, failed with `Operation not permitted` because
`/tmp` has sticky-directory semantics and the files belonged to `trashbot`.
They were then removed successfully with:

```sh
sudo -u trashbot rm -f \
  /tmp/codex-camera-still-20260727.jpg \
  /tmp/codex-camera-video-20260727.h264 \
  /tmp/codex-camera-video-20260727.json
```

Reversible: the removed temporary evidence can be recreated with the documented
capture commands.

No package installation, udev-rule edit, network configuration edit, group
change, source edit, service-unit edit, firmware flash, or reboot was performed
during the current SSH evidence pass.

## Files Created or Modified

### Controller repository

- Created:
  `/Users/burnschris/Git/hoverboard-robot/docs/raspberry-pi-bringup-2026-07-27.md`
- Modified tracked source files: none
- Commits, pushes, branches, or pull requests: none

### Raspberry Pi

Pre-existing image-provisioned files/directories relevant to this session:

- `/opt/trashcan-robot` deployment tree
- `/opt/trashcan-robot/donkeycar/.venv`
- `/opt/trashcan-robot/donkeycar/data/tubs`
- `/opt/trashcan-robot/donkeycar/data/logs`
- `/opt/trashcan-robot/donkeycar/models`
- `/etc/systemd/system/trashcan-donkeycar.service`
- `/etc/trashcan-robot-build-info`
- `/etc/trashcan-robot-image-validated`
- `trashbot` entries in `/etc/passwd` and `/etc/group`

Earlier in the bring-up, `/etc/shadow` changed at
`2026-07-27 09:26:19-04:00` because `passwd` was run. No credential was read or
recorded.

The evidence pass created and deleted the three `/tmp/codex-camera-*` files
listed above. It temporarily changed only the runtime state of
`trashcan-donkeycar.service`; the original start-requested/enabled condition was
restored. No persistent Raspberry Pi file was intentionally modified by the
evidence pass.

## Failures and Diagnostics

### Raspberry Pi deployment is not a Git working tree

Symptom: repository metadata commands fail at `/opt/trashcan-robot`.

Evidence: all four requested Git commands returned
`fatal: not a git repository (or any of the parent directories): .git`; a
search found no `.git` directory.

Root cause: the image contains an rsynced deployment snapshot rather than a Git
checkout.

Action taken: recorded the controller repository's exact Git state separately
and did not infer an application commit from timestamps or firmware manifests.

Retest result: not applicable; no Git metadata was added.

Current status: **BLOCKED** for Pi-side remote/branch/commit/status reporting.

### Donkeycar application restart loop

Symptom: `trashcan-donkeycar.service` exits every few seconds and systemd
restarts it.

Evidence: exit status 1, `NRestarts=672` in the final sample, and the repeated
traceback at `pipeline.py:92`.

Root cause: application code calls
`TubWriter(path=str(tub_path), ...)`; installed Donkeycar 5.3.0 exposes
`TubWriter(base_path, inputs=[], types=[], metadata=[], max_catalog_len=1000)`.

Action taken: confirmed the installed runtime signature and ran the unit suite;
no source was changed because this task permits only the new documentation
file.

Retest result: `10 passed in 0.59s`, but the actual service still exits with
the same `TypeError`.

Current status: **FAIL / unresolved**.

### Web controller unavailable

Symptom: port 8887 does not listen locally or over the LAN.

Evidence: no `ss` listener; both curls returned HTTP status 000 and connection
failure.

Root cause: the vehicle fails construction before `vehicle.start(...)` can
start the controller thread.

Action taken: tested both loopback and controller-computer paths.

Retest result: still unreachable.

Current status: **FAIL / blocked by Donkeycar runtime**.

### Dashboard unavailable

Symptom: port 8888 does not listen; no camera or telemetry page is available.

Evidence: no `ss` listener; loopback and LAN curls returned status 000.

Root cause: `DashboardServer.start()` is after the failing `TubWriter`
constructor and is never reached.

Action taken: tested both loopback and controller-computer paths.

Retest result: still unreachable.

Current status: **FAIL / blocked by Donkeycar runtime**.

### Tub recording unavailable

Symptom: tub directory contains zero files, records, and images.

Evidence: `find` produced no entries and `find ... -type f | wc -l` returned
`0`.

Root cause: the `TubWriter` constructor itself raises before it can create a
tub.

Action taken: verified ownership and writability to exclude filesystem
permissions as the cause.

Retest result: directories are writable, but construction still fails.

Current status: **BLOCKED by API incompatibility**.

### ESP32 and downstream hardware gates unavailable

Symptom: no serial device or ESP32 USB identity exists on the Pi.

Evidence: `lsusb` showed only root hubs and the VIA hub; no `/dev/ttyACM*`,
`/dev/ttyUSB*`, or `/dev/serial/by-id/*` existed; `mmcli -L` found no modem.

Root cause: the ESP32 was not physically connected.

Action taken: no serial path was guessed and no protocol or motion command was
sent.

Retest result: final USB/serial enumeration remained unchanged.

Current status: **BLOCKED pending physical connection after the runtime defect
is corrected**.

### Camera visual quality not user-confirmed

Symptom: orientation, brightness, focus, latency, and field of view have no
user verdict.

Evidence: functional files decoded and were nonuniform, but no user visual
confirmation was provided.

Root cause: no user visual review occurred during this evidence pass.

Action taken: collected objective decode and pixel statistics without claiming
subjective quality.

Retest result: functional capture passed; subjective criteria remain unknown.

Current status: **NOT TESTED for visual quality**.

### Temporary camera cleanup initially used the wrong account

Symptom: `rm -f /tmp/codex-camera-*` returned `Operation not permitted` for all
three temporary evidence files.

Evidence: the files were owned `trashbot:trashbot`, while the failed removal
was run by `pi` in the sticky `/tmp` directory.

Root cause: a non-owner cannot remove another account's files from a
sticky-bit directory even when the files themselves are readable.

Action taken: reran the exact cleanup through
`sudo -u trashbot rm -f` against the three explicit paths.

Retest result: three subsequent `test ! -e <path>` checks each returned exit
status 0.

Current status: **resolved; no temporary camera evidence file remains**.

## Remaining Blockers

1. The `TubWriter` keyword mismatch prevents the Donkeycar process from
   remaining alive.
2. Web controller, dashboard, logging, and tub recording cannot be validated
   until application construction succeeds.
3. User visual confirmation is still required for camera orientation,
   brightness, focus, latency, and field of view.
4. The ESP32 must be physically connected and identified before any serial
   protocol gate.
5. Zero-motion protocol values, soak counters, and STOP/DISARM results remain
   unobserved.
6. All motion and safety tests remain unperformed in this July 27 sequence.
7. Network performance has not been tested where the robot will physically
   operate.

## Exact Next Step

Change the `TubWriter` call in
`donkeycar/trashcan_robot/pipeline.py` from the unsupported `path=` keyword to
the installed Donkeycar 5.3.0 `base_path=` keyword.

## Final Status

| Gate | Status | Evidence |
|---|---|---|
| Raspberry Pi health | PASS | Ethernet/SSH healthy, 23 GiB free, 3.4 GiB RAM available, 49.6 C, `throttled=0x0`; application failure is recorded in its own gate |
| Camera | PASS | OV5647 enumerated; 2592x1944 JPEG decoded and was nonuniform; 146-frame 640x480 H.264 capture decoded; subjective visual review not tested |
| Donkeycar runtime | FAIL | Unit tests 10/10 passed, but real process exits on `TubWriter(path=...)` |
| Web controller | FAIL | No 8887 listener; loopback and LAN HTTP status 000 |
| Dashboard | FAIL | No 8888 listener; loopback and LAN HTTP status 000 |
| Tub recording | BLOCKED | Zero files/records/images; constructor raises |
| Network | PASS | 20/20 ICMP replies, 0% loss, 8.110 ms average; actual driveway location not tested |
| ESP32 USB | BLOCKED | ESP32 not connected; no serial device |
| Zero-motion protocol | BLOCKED | No ESP32; all required fields unobserved |
| Zero-demand soak | NOT TESTED | No protocol session |
| Lifted-wheel motion | NOT TESTED | No July 27 physical wheel test |
| Safety shutdowns | NOT TESTED | No connected/armed drive system |
| Flat-ground drive | NOT TESTED | Not physically performed |
| Manual driveway drive | NOT TESTED | Not physically performed |

Final readiness classification: **PI_CAMERA_READY**

No code was committed or pushed.

## Follow-up Completion — 2026-07-27

The preceding sections preserve the initial `PI_CAMERA_READY` evidence and
failure state. They are historical and are superseded by the completed
follow-up validation in
`docs/raspberry-pi-final-validation-2026-07-27.md`.

The Donkeycar 5.3 `TubWriter(base_path=...)` incompatibility was fixed and
covered by a failing-then-passing regression test. Subsequent live diagnosis
also fixed the ignored 5 Hz logging configuration, missing dashboard runtime
FPS, pipeline output-contract wiring, stale nonzero input on serial reconnect,
and missing zero-demand frames before ARM retries.

Final follow-up results:

| Gate | Follow-up status |
|---|---|
| Donkeycar deployed suite | PASS — 19 tests |
| Service startup | PASS |
| Controller 8887 loopback/LAN | PASS |
| Dashboard 8888 loopback/LAN | PASS |
| `/api/state` | PASS |
| Production camera endpoint | PASS — changing 640x480 JPEG frames |
| Tub recording | PASS — 40 final zero-demand records and images |
| Logging | PASS — observed 4.678 Hz against configured 5 Hz |
| Graceful stop/restart | PASS |
| No-ESP32 operation | PASS |
| ESP32 physical protocol and motion gates | BLOCKED — serial device absent |

Follow-up readiness classification:
**PI_SOFTWARE_READY / ESP32_PHYSICALLY_BLOCKED**.

No nonzero hardware motor command was sent. No commit or push occurred.
