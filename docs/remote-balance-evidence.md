# Remote balance evidence capture

Use this workflow when the ESP32 and MPU6050 are attached to a different
macOS or Linux computer. It produces one checksummed archive containing the
raw serial stream, decoded binary frames, `BALANCE` JSON telemetry (including
raw gyro, bias-corrected gyro, and learned bias), every outbound query, operator
action notes, firmware identity, source identity, and host metadata.

The capture tool does not flash firmware. Without `--command-plan`, it never
arms or sends movement: at startup it sends `hello` and `disarm`; while
recording it sends read-only capability and telemetry queries; at shutdown it
sends `stop` and `disarm`. A validated command plan is allowed only for the
`motor-transport` and `lifted-wheel` stages after their safety gates. The plan
executes on the same serial descriptor as capture, and every outbound frame is
timestamped with its exact raw bytes.

## Prepare the other computer

Copy this exact working tree, including its uncommitted and untracked files, to
the other computer. A fresh clone of `main` is insufficient because the balance
layer is intentionally uncommitted. Preserve the directory as-is rather than
copying only the firmware binary.

Requirements:

- macOS or Linux;
- Node.js 18 or newer;
- `git`, `tar`, `stty`, and either `sha256sum` or `shasum`;
- the project's PlatformIO environment if the ESP32 image must be rebuilt or
  flashed.

Run the local tests on that computer before using hardware:

```sh
./tools/test-all.sh
```

Build the default dry-run image:

```sh
PLATFORMIO_CORE_DIR="$PWD/.platformio" \
  PLATFORMIO_SETTING_ENABLE_TELEMETRY=no \
  .venv/bin/pio run -e esp32_balance_coordinator
```

The firmware passed to the capture command must be the exact binary flashed to
the ESP32:

```text
.pio/build/esp32_balance_coordinator/firmware.bin
```

For the powered Stage 5 transport benchmark, use the dedicated
`esp32_stage5_transport` environment. It permits output only through direct
mode and sends a zero `DIRECT_LR` command while disarmed so the existing
MASTER/SLAVE controllers can reach `READY` without enabling either bridge.
Do not use this image for balancing.

For Stage 6 lifted-wheel correction, use `esp32_stage6_lifted`. It accepts
active output only in balance mode, clamps correction to 50 command units, and
limits output slew to 100 command units per second. It is not approved for a
ground-balancing test.

If flashing is required for Stage 3, flash only
`esp32_balance_coordinator` to the ESP32. Do not flash MASTER or SLAVE. Confirm
the build reports `GS_BALANCE_DRY_RUN=1`; keep motor power removed throughout.
After upload, identify the serial device again rather than assuming its name
stayed the same.

## Stage 3 capture

Follow the wiring and power restrictions in
[`balance-wiring.md`](balance-wiring.md). With motor power removed and the
chassis secured, run:

```sh
./tools/capture-balance-evidence.sh \
  --port /dev/serial/by-id/REPLACE_WITH_EXACT_DEVICE \
  --firmware .pio/build/esp32_balance_coordinator/firmware.bin \
  --duration 180 \
  --stage mpu-diagnostic \
  --label stage3-mpu
```

On macOS, use the exact `/dev/cu.*` device instead. Do not use a guessed port.

While the capture runs, type the following notes into its terminal at the
corresponding physical moments. Notes are written only to the evidence file and
are never transmitted to the robot.

```text
STATIONARY calibration-start
STATIONARY calibrated
TILT_FORWARD start
UPRIGHT returned
TILT_BACKWARD start
UPRIGHT returned
STATIONARY final
```

Wait for `calibrated` before the first tilt. Move slowly, return to upright
between directions, and leave the chassis stationary at the end. Press
Control-C only if an immediate stop is needed; the recorder still sends
`stop` and `disarm` and packages the partial evidence.

Do not proceed to motor-powered stages until the Stage 3–4 archives have been
reviewed and the one-time physical checkpoint has passed. Stage 5–6 command
plans must be reviewed before use; the supplied templates are not permission
to power or move the wheels.

## Evidence produced

The command creates a directory plus `<directory>.tar.gz` and
`<directory>.tar.gz.sha256`.

The directory contains:

| File | Evidence |
|---|---|
| `session-start.json` | UTC start time, stage, port, host, branch, commit, working-tree status, firmware SHA-256 and byte size, and enforced command restrictions |
| `raw-serial.bin` | exact bytes received from the ESP32 |
| `events.ndjson` | timestamped decoded frames, debug telemetry, device text, host queries, errors, and operator notes |
| `session-end.json` | stop reason, elapsed time, raw byte count, and event counts |
| `SHA256SUMS` | integrity hashes for every evidence file |

Even a serial-open failure is packaged with its error metadata. The wrapper
returns a nonzero exit status in that case, so do not mistake an error archive
for a completed physical run.

Verify the returned archive before transferring it:

```sh
sha256sum -c balance-evidence-mpu-diagnostic-*.tar.gz.sha256
```

On macOS:

```sh
shasum -a 256 -c balance-evidence-mpu-diagnostic-*.tar.gz.sha256
```

Return both the `.tar.gz` archive and its `.sha256` file. Those two files are
enough to reproduce the telemetry timeline and determine what physical evidence
is still missing without attaching the devices to this computer.
