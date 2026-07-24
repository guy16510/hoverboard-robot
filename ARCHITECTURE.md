# Architecture

```text
USB console (115200)
        |
        v
one ESP32 coordinator
        | GPIO17 TX / GPIO35 RX, 19200 8N1
        v
GAUSSTOP master GD32
        | PA2 / PA3, 115200 8N1
        v
GAUSSTOP slave GD32
```

Only the master accepts the 9-byte ESP32 command. Only the slave accepts the
6-byte master command. The ESP32 contains no slave-coordinator API and never
addresses the slave. **Native-test validated** and **Statically validated**

The master converts speed/steer or direct-left/right input into bounded logical
wheel demands. Its local wheel receives the logical left demand. The logical
right demand is multiplied by `-1` before transmission to the mechanically
mirrored slave. The slave odometer is multiplied by `-1` when combined into
logical right-wheel feedback. **Native-test validated**

## SOLID responsibility boundaries

| Module | Responsibility | Dependency direction |
|---|---|---|
| `gausstop_board` | immutable pin/memory declarations and GD32 adapters | depends on SPL and domain bridge contract |
| `gs_protocol` | exact byte packing and CRC | fixed-width types only |
| `gs_frame_parser` | bounded byte-stream framing and resynchronization | protocol CRC only |
| `gs_wheel_mix` | pure logical wheel conversion and mirroring | value types only |
| `gs_commutation` | pure Hall/direction to phase-vector mapping | value types only |
| `gs_motor_control` | ramp, reversal dwell, Hall progression, bridge decision | small bridge port interface |
| `gs_safety` | input, timeout, ADC baseline, and fault policy | read-only samples |
| `gs_master` | remote command, wheel topology, slave frame, combined feedback | pure domain modules |
| `gs_slave` | master command and feedback role | pure domain modules |
| `gs_console` | strict command-to-request parsing | coordinator value state |
| entrypoints | construct and service role-specific dependencies | role-specific modules only |

The boundaries apply SRP and dependency inversion without dynamic allocation,
inheritance, factories, or containers. Hardware replacement is localized to
the adapter: replacing PB6/PB7 does not change protocol, parsing, mixing,
motor, safety, or coordination. The same value modules are linked by native
tests and firmware targets. This structure was selected using the `solid`
skill. **Statically validated**

## ESP32 balance layer

`esp32_balance_coordinator` is a separate dry-run-first image. It keeps the
existing GPIO17 command and GPIO35 feedback transport while adding MPU6050 I2C
on GPIO21/22 and a cooperative fixed-priority 200 Hz schedule.

| Abstraction | Implementation | Responsibility |
|---|---|---|
| `IImu` | `Mpu6050Imu` | address detection, configuration, burst sampling, calibration, health |
| `IAttitudeEstimator` | `ComplementaryPitchEstimator` | deterministic measured-delta-time pitch fusion |
| `IBalanceController` | `CascadedBalanceController` | velocity PI, pitch PID, yaw mixing, limiting |
| `IMotorCommandSink` | `SwdMotorCommandSink` | lock-free latest motor-command snapshot |
| `ICommandSource` | `SerialCommandSource`, `WebCommandSource` | framed commands and expiring movement leases |
| `ITelemetrySink` | `SerialTelemetrySink` | latest diagnostic/controller snapshot |
| `IClock` | `EspClock` | monotonic microsecond time |

The control pass runs before lower-priority work. Serial and feedback service
are bounded to 64 bytes per pass, RMT transmission is nonblocking, binary
responses use a fixed eight-frame queue, and telemetry is drained according to
available USB transmit capacity. The optional web adapter limits request and
page-response work to 64 bytes per service call. A two-second ESP32 task
watchdog covers the cooperative scheduler. The control modules do not allocate
heap memory or depend on Arduino, Wi-Fi, HTTP, JSON, or a filesystem.
**Native-test validated** and **Build validated**

Binary status, IMU, motor, odometry, fault, capability, and bounded
configuration messages are encoded outside the control pass. Configuration
updates are validated and accepted only while disarmed; they are RAM-only and
reset controller history to zero. Transport metrics separately track frame
rate, ESP acknowledgment latency, and end-to-end applied-sequence latency.
**Native-test validated** and **Build validated**

The default environment explicitly ignores the Wi-Fi library. The optional
`esp32_balance_web` environment alone compiles the static page and WebSocket
adapter. Both environments retain `GS_BALANCE_DRY_RUN=1`. **Build validated**

## Role isolation

- Safe recovery links PB2 and SRAM-heartbeat behavior only; linker symbol checks
  reject operational bridge, ADC, and UART symbols. **Statically validated**
- Communication diagnostic uses PB6/PB7 and PA2/PA3, parses frames, responds
  with disabled feedback, and sends only zero/disabled slave frames. Its ELF is
  rejected if bridge-enable or motor-state symbols survive linking.
  **Statically validated**
- Passive probe constructs its controller UART with TX pin `-1` and contains no
  controller-UART transmit call. USB reporting is separate. **Statically validated**
