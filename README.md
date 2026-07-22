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

See [UPSTREAM.md](UPSTREAM.md), [LEGACY.md](LEGACY.md), [HARDWARE.md](HARDWARE.md),
[PROTOCOL.md](PROTOCOL.md), [SAFETY.md](SAFETY.md), and
[HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md) before using any artifact.

## License

GPL-3.0-only. See [LICENSE](LICENSE). Modified upstream concepts and retained
legacy behavior are identified in `UPSTREAM.md`, `LEGACY.md`, and
`SOURCE_CLASSIFICATION.csv`.

