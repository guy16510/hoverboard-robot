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

## Role isolation

- Safe recovery links PB2 and SRAM-heartbeat behavior only; linker symbol checks
  reject operational bridge, ADC, and UART symbols. **Statically validated**
- Communication diagnostic uses PB6/PB7 and PA2/PA3, parses frames, responds
  with disabled feedback, and sends only zero/disabled slave frames. Its ELF is
  rejected if bridge-enable or motor-state symbols survive linking.
  **Statically validated**
- Passive probe constructs its controller UART with TX pin `-1` and contains no
  controller-UART transmit call. USB reporting is separate. **Statically validated**

