# GAUSSTOP one-ESP32 dual-motor firmware

Clean, GPLv3 firmware for one ESP32 coordinating two GAUSSTOP DPHC-V3.3
GD32F130C8T6 motor controllers in a master/slave topology.

```text
USB console (115200) -> ESP32 -> GPIO17/GPIO35 (19200) -> master GD32
                                                         -> PA2/PA3 (115200) -> slave GD32
```

The ESP32 is the only high-level controller. The master mixes logical left and
right demands, drives its local motor, mirrors the slave electrical command,
and combines feedback. The slave accepts commands only from the master.

The USB command interface is newline-delimited ASCII:

```text
enable
drive SPEED STEER
lr LEFT RIGHT
forward VALUE
reverse VALUE
ramp STEP
stop
disable
clearfault
shutdown
status
help
```

Reset is disabled with zero demand. `enable` never creates motor demand.
Motion commands are rejected until enabled. The default ramp is 10 command
units per 50 ms heartbeat.

## Validation status

- **Historically physically verified:** GAUSSTOP bridge and Hall pin mapping
  recorded from the read-only legacy project.
- **Awaiting hardware validation:** the exposed connector route to master
  PB6/PB7, all electrical safety inputs, wheel direction, and loaded behavior.
- No firmware is flashed and no serial or USB hardware is accessed by this
  repository's build or test commands.

Use `./tools/test-all.sh` for host tests and `./tools/build-all.sh` for all
firmware builds. Neither command uploads firmware or enumerates devices.

The next ESP32 balance layer is a separate, dry-run-first environment:

```sh
.venv/bin/pio run -e esp32_balance_coordinator
```

It scans an MPU6050 on GPIO21/22, runs a 200 Hz estimator/controller schedule,
accepts the versioned Raspberry Pi protocol over USB serial, and preserves the
GPIO17/35 motor path while always transmitting disabled zero output by default.
The optional `esp32_balance_web` environment compiles a removable static
page/WebSocket adapter; the default environment explicitly excludes Wi-Fi.

See [the Pi serial protocol](docs/pi-serial-protocol.md),
[balance wiring](docs/balance-wiring.md),
[the tuning guide](docs/balance-tuning.md), and
[local validation notes](docs/local-balance-validation.md). When the hardware
is attached to another computer, use the
[remote evidence capture workflow](docs/remote-balance-evidence.md).

See [UPSTREAM.md](UPSTREAM.md), [LEGACY.md](LEGACY.md), [HARDWARE.md](HARDWARE.md),
[PROTOCOL.md](PROTOCOL.md), [SAFETY.md](SAFETY.md), and
[HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md) before using any artifact.

## License

GPL-3.0-only. See [LICENSE](LICENSE). Modified upstream concepts and retained
legacy behavior are identified in `UPSTREAM.md`, `LEGACY.md`, and
`SOURCE_CLASSIFICATION.csv`.
