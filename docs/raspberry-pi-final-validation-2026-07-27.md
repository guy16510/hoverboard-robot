# Raspberry Pi Robot Final Validation

Date: 2026-07-27  
Host: `trashcan-robot.local` (`192.168.2.44`)  
Deployment: `/opt/trashcan-robot`  
Service: `trashcan-donkeycar.service`

## Final readiness classification

**PI_SOFTWARE_READY / ESP32_PHYSICALLY_BLOCKED**

The complete Raspberry Pi application is operational with the ESP32 absent.
The controller, dashboard, live camera, state API, tub recording, logging,
systemd lifecycle, and safe disconnected behavior pass. No nonzero hardware
motor command was sent.

The next gate requires the user to connect the ESP32 physically. Current-session
protocol, telemetry, fault, STOP, DISARM, lease-expiration, feedback-freshness,
and physical motion evidence cannot be collected while no serial device exists.

## What was fixed

1. Donkeycar 5.3.0 tub creation now calls `TubWriter` with `base_path` instead
   of the unsupported `path` keyword.
2. The Raspberry Pi image build and exported-image validation now check both
   the installed Donkeycar `base_path` API and the application call contract.
3. JSON run logging now honors `logging.telemetry_hz` instead of writing at the
   20 Hz control-loop rate.
4. The pipeline now measures its actual runtime frequency and publishes it to
   `/api/state` and the JSON run log.
5. Explicit pipeline input/output contracts prevent dashboard telemetry keys
   from being accidentally appended to the five-value ESP32 drive output.
6. Initial serial connection and every reconnect are latched at zero until a
   neutral controller input is observed.
7. Every serial ARM retry is preceded by an exact zero-demand velocity/yaw
   lease frame.
8. Targeted regression tests cover every defect above plus missing-device,
   disconnect, reconnect, and shutdown-safe behavior.

The `solid` engineering workflow influenced these changes: each discovered
defect was reproduced by a focused failing test before the smallest production
fix was applied.

## What was tested

### Application and image-source validation

- Final deployed Pi suite:

  ```text
  19 passed in 0.57s
  ```

- Focused tests were observed failing before their corresponding fixes:
  Donkeycar tub keyword, telemetry rate, dashboard FPS, pipeline output wiring,
  reconnect-neutral latch, and zero-before-ARM ordering.
- `bash -n` passed for the image build, image marker, exported-image validator,
  and Donkeycar shell scripts.
- `git diff --check` passed.
- The image build source rsyncs the controller repository into
  `/opt/trashcan-robot`, then runs the complete application suite inside the
  generated filesystem.
- The actual compressed image was not rebuilt during this live Pi session.
  A future image build will include these local source changes and fail its
  validation if the Donkeycar tub API regresses.

### Service lifecycle

- The original restart loop was stopped before deployment.
- A corrected pre-final service instance ran from
  `2026-07-27 10:48:27 EDT` through `10:53:42 EDT`; `NRestarts` stayed at
  `853` for 5 minutes 15 seconds.
- Graceful stop returned `Result=success`, `ExecMainStatus=0`,
  `ActiveState=inactive`, and `SubState=dead`.
- Shutdown logs show orderly camera stop/close, tub close, manifest save, and
  systemd deactivation.
- The complete final service was started at
  `2026-07-27 11:07:02 EDT`.
- At `2026-07-27 11:12:16 EDT`, after 5 minutes 14 seconds, the service
  remained `active/running` with `MainPID=16330`, `NRestarts=0`,
  `ExecMainStatus=0`, and no error/exception/traceback/failure entry in the
  final-instance journal.

## Controller and dashboard results

| Check | Pi loopback | Controller computer over LAN |
|---|---:|---:|
| Controller `/drive` on 8887 | HTTP 200, 7,485 bytes | HTTP 200, 7,485 bytes |
| Dashboard `/` on 8888 | HTTP 200, 560 bytes | HTTP 200, 560 bytes |
| Dashboard `/api/state` | HTTP 200 | HTTP 200 |
| Dashboard `/camera.jpg` | HTTP 200 | HTTP 200 |

Final state included:

```json
{
  "esp32_connected": false,
  "fps": 19.95,
  "inference_rate": 0.0,
  "mode": "Manual",
  "recording": false,
  "serial_latency_ms": null,
  "steering": 0.0,
  "throttle": 0.0
}
```

The controller's root HTTP 301 is its expected redirect to `/drive`; following
the redirect returns the complete controller page with HTTP 200.

## Camera and tub results

- Final production camera frames were baseline JPEG, 640x480, three-component.
- Two frames captured two seconds apart had different SHA-256 values:

  ```text
  54bb871705230a340ced540159fc2d348deb38ac94e3b89e69927f310109782c
  9e315f929fbef1699e47be0485e61e9d2d450cde4f3f968a8507510006491da3
  ```

  This proves the production endpoint was updating rather than serving one
  static image.

- Final zero-demand tub:
  `data/tubs/tub_20260727T150705Z`
- Records: 40
- Images: 40
- Every record decoded as JSON and contained:

  ```text
  user/angle=0.0
  user/throttle=0.0
  user/mode=user
  ```

- Every referenced image existed, was nonempty, decoded as JPEG, and measured
  640x480.
- Tub and log content remained owned by `trashbot:trashbot`.
- Final run log:
  `data/logs/run-20260727T150705Z.jsonl`
- Observed log rate: 4.678 Hz against configured 5 Hz.
- Logged camera frequency: approximately 19.95 Hz.

## ESP32 and protocol results

### Current physical state

- `lsusb` shows only Linux root hubs and the VIA Labs USB hub.
- No `/dev/ttyACM*`, `/dev/ttyUSB*`, or `/dev/serial/by-id/*` device exists.
- `/api/state` correctly reports `esp32_connected=false`, zero steering,
  zero throttle, and a missing-device/retry fault.
- The service remains active while the ESP32 is absent.

### Software-only safety evidence

- Missing serial device returns disconnected state and zero output.
- A simulated first connection with nonzero input sends only zero and reports
  `waiting for neutral input after ESP32 connection`.
- A simulated reconnect with stale nonzero input also sends only zero.
- Motion becomes eligible only after a neutral input has been observed.
- Each simulated ARM attempt is immediately preceded by a zero-demand
  velocity/yaw frame.
- Shutdown sends zero, then transport disconnect performs STOP and DISARM.

### Blocked physical evidence

The following remain blocked because the ESP32 is not connected:

- current protocol version, dry-run flag, operating mode, state, health flags,
  fault mask, feedback freshness, and telemetry counters;
- physical STOP, DISARM, lease-expiration, cable-disconnect, and reconnect
  observations;
- five-minute zero-demand ESP32 soak;
- any lifted-wheel, ground, or driveway motion.

No ESP32 or hoverboard firmware was flashed or modified. No nonzero hardware
motor command was sent.

## Deployment and ownership

Changed files were staged under timestamped `/tmp/codex-deploy-*` directories,
backed up before replacement, and installed as `trashbot:trashbot`.

Backup roots:

```text
/opt/trashcan-robot/backups/20260727T144637Z
/opt/trashcan-robot/backups/20260727T145322Z
/opt/trashcan-robot/backups/20260727T145720Z
/opt/trashcan-robot/backups/20260727T150043Z
/opt/trashcan-robot/backups/20260727T150532Z
```

Pi deployment files changed or added:

```text
/opt/trashcan-robot/donkeycar/trashcan_robot/pipeline.py
/opt/trashcan-robot/donkeycar/trashcan_robot/logging_part.py
/opt/trashcan-robot/donkeycar/trashcan_robot/esp32_drive.py
/opt/trashcan-robot/donkeycar/trashcan_robot/transport.py
/opt/trashcan-robot/donkeycar/tests/test_pipeline.py
/opt/trashcan-robot/donkeycar/tests/test_logging_part.py
/opt/trashcan-robot/donkeycar/tests/test_esp32_drive.py
/opt/trashcan-robot/donkeycar/tests/test_transport.py
/opt/trashcan-robot/raspberry-pi-image/stage-trashcan/00-install/00-run-chroot.sh
/opt/trashcan-robot/raspberry-pi-image/validate-image.sh
```

Runtime evidence also created timestamped tubs and JSON logs under the
application's `data` directory.

## Git invariants

Starting controller repository state for this goal:

```text
branch=main
HEAD=0a431191524ac53d2f6540f7e8c6c7c84784731e
status=?? docs/raspberry-pi-bringup-2026-07-27.md
```

Final controller repository state:

```text
branch=main
HEAD=0a431191524ac53d2f6540f7e8c6c7c84784731e
```

Locally modified tracked files:

```text
donkeycar/tests/test_esp32_drive.py
donkeycar/tests/test_pipeline.py
donkeycar/trashcan_robot/esp32_drive.py
donkeycar/trashcan_robot/logging_part.py
donkeycar/trashcan_robot/pipeline.py
donkeycar/trashcan_robot/transport.py
raspberry-pi-image/stage-trashcan/00-install/00-run-chroot.sh
raspberry-pi-image/validate-image.sh
```

Local untracked files:

```text
docs/raspberry-pi-bringup-2026-07-27.md
docs/raspberry-pi-final-validation-2026-07-27.md
donkeycar/tests/test_logging_part.py
donkeycar/tests/test_transport.py
```

The starting and final branch are both `main`. The starting and final HEAD are
both `0a431191524ac53d2f6540f7e8c6c7c84784731e`; HEAD did not change.

No commit, amend, merge, rebase, cherry-pick, tag, push, branch, pull request,
stash, reset, clean, or discard operation was performed.

## Rollback

Stop the service first:

```sh
sudo systemctl stop trashcan-donkeycar.service
```

The original pre-goal versions of the initial deployment files are under:

```text
/opt/trashcan-robot/backups/20260727T144637Z
```

Later backup roots preserve each intermediate version. Restore the desired
version to the matching `/opt/trashcan-robot/...` path, then enforce ownership:

```sh
sudo chown trashbot:trashbot /opt/trashcan-robot/donkeycar/trashcan_robot/*.py
sudo chown trashbot:trashbot /opt/trashcan-robot/donkeycar/tests/test_*.py
sudo chown trashbot:trashbot /opt/trashcan-robot/raspberry-pi-image/validate-image.sh
sudo chown trashbot:trashbot \
  /opt/trashcan-robot/raspberry-pi-image/stage-trashcan/00-install/00-run-chroot.sh
sudo systemctl start trashcan-donkeycar.service
```

`test_logging_part.py` and `test_transport.py` were new on the Pi and have no
pre-deployment counterpart; remove only those exact files if rolling all the
way back to the original snapshot.

## Remaining blocker and exact next physical action

With the hoverboard system unpowered, wheels lifted and the robot secured,
emergency disconnect accessible, user physically present, and area clear:

1. Connect the ESP32 USB cable to the Raspberry Pi.
2. Keep all controls neutral.
3. Tell Codex: `ESP32 connected`.

The next session must first enumerate the serial device and run only
zero-demand protocol, telemetry, fault, STOP, DISARM, lease-expiration,
disconnect/reconnect, and a five-minute zero-demand soak. It must not send a
nonzero motor command until the user separately confirms all five required
physical-safety conditions, and it must not perform a ground or driveway test
without explicit approval.
