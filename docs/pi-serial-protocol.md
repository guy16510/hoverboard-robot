# Raspberry Pi to ESP32 serial protocol v1

This is the durable northbound control protocol. It is binary, versioned,
bounded, sequence-numbered, CRC-protected, and explicitly little-endian. USB
serial at 115200 baud is the initial transport. A later hardware UART may carry
the same bytes, but it must not use GPIO17, GPIO35, GPIO21, or GPIO22.

Debug telemetry may be newline-delimited JSON. JSON is not a control wire
format.

## Frame

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | marker 0 | `0xA5` |
| 1 | 1 | marker 1 | `0x5A` |
| 2 | 1 | version | `1` |
| 3 | 1 | message type | table below |
| 4 | 1 | flags | message-specific; zero unless stated |
| 5 | 2 | sequence | unsigned little-endian |
| 7 | 2 | payload length | unsigned little-endian, `0..48` |
| 9 | N | payload | message-specific |
| 9 + N | 2 | CRC | CRC-16/CCITT-FALSE, little-endian |

The CRC covers `version` through the final payload byte. Parameters are
polynomial `0x1021`, initial value `0xFFFF`, no reflection, and no final XOR.
The check value for ASCII `123456789` is `0x29B1`.

A receiver rejects unsupported versions, payloads over 48 bytes, invalid CRCs,
partial frames older than 50 ms, duplicated sequences, and stale sequences.
Sequence `0xFFFF` followed by `0x0000` is fresh. Freshness uses the positive
half of the modulo-65536 sequence space. An empty hello frame explicitly starts
a new serial session and establishes its sequence as the new freshness base.
It also releases any previous serial movement lease and submits zero velocity
and yaw. The reference client sends hello when it opens the port.

## Message types

| Value | Name | Direction |
|---:|---|---|
| `0x01` | hello | Pi -> ESP32 |
| `0x02` | capabilities | both |
| `0x10` | arm | Pi -> ESP32 |
| `0x11` | disarm | Pi -> ESP32 |
| `0x12` | stop | Pi -> ESP32 |
| `0x13` | emergency stop | Pi -> ESP32 |
| `0x14` | clear fault | Pi -> ESP32 |
| `0x15` | set operating mode | Pi -> ESP32 |
| `0x20` | set linear velocity | Pi -> ESP32 |
| `0x21` | set yaw rate | Pi -> ESP32 |
| `0x22` | set velocity and yaw | Pi -> ESP32 |
| `0x23` | heartbeat / lease renewal | Pi -> ESP32 |
| `0x30` | status | both |
| `0x31` | IMU telemetry | both |
| `0x32` | motor telemetry | both |
| `0x33` | odometry | both |
| `0x34` | active faults | both |
| `0x40` | configuration read | both |
| `0x41` | configuration update | Pi -> ESP32 |
| `0x7E` | acknowledgment | ESP32 -> Pi |
| `0x7F` | error response | ESP32 -> Pi |

Empty payloads are used by hello, capabilities, arm, disarm, stop, emergency
stop, clear fault, status, and telemetry requests. Query responses use the
request sequence number. Hello and capabilities requests both produce a
capabilities response.

Operating mode is one byte: `0` diagnostic, `1` balance, and `2` drive.
Unknown values are rejected.

## Movement and lease payload

Types `0x20`, `0x21`, `0x22`, and a movement lease renewal use the same
10-byte payload:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 2 | linear velocity | signed little-endian milli-units |
| 2 | 2 | yaw rate | signed little-endian milli-units |
| 4 | 4 | lease ID | unsigned little-endian |
| 8 | 2 | lifetime | unsigned little-endian milliseconds |

Lifetime zero is invalid. The current client uses 500 ms. When the lease
expires or serial input disconnects, velocity and yaw become zero; the ESP32
does not disarm solely because the Pi disappeared and continues local balance
control if all safety conditions remain healthy.

Only one source may own movement. Priority is:

1. emergency stop;
2. local disarm;
3. Raspberry Pi serial;
4. temporary web controller.

## Responses and telemetry

An acknowledgment payload is:

```text
request_type:u8 status:u8
```

Status zero means accepted. An error payload is:

```text
request_type:u8 error_code:u8 detail:u16-le
```

Error codes are `1` malformed, `2` CRC, `3` stale sequence, `4` unsupported,
`5` lease conflict, `6` unsafe state, `7` invalid configuration, and `8`
response queue full.

All telemetry integers are little-endian. The binary schemas are:

| Message | Bytes | Fields |
|---|---:|---|
| capabilities | 12 | protocol `u8`, dry-run `u8`, web `u8`, mode bits `u8`, control Hz `u16`, motor Hz `u16`, maximum payload `u16`, configuration-key count `u16` |
| status | 16 | state `u8`, mode `u8`, active source `u8`, health bits `u8`, balance faults `u32`, loop overruns `u32`, rejected serial frames `u32` |
| IMU | 44 | address/calibrated/valid/reserved `u8[4]`, acceleration milli-g `i16[3]`, raw gyro centi-dps `i16[3]`, raw pitch/filtered pitch/rate in centi-units `i16[3]`, sample rate centi-Hz `u16`, I2C errors `u32`, missed samples `u32`, sample age microseconds `u32`, accepted calibration samples `u16`, gyro-bias centi-dps `i16[3]` |

The IMU packet carries raw mapped gyroscope measurements and the learned bias
separately. A host can derive the corrected rate as `raw - bias`; the pitch-rate
field is the corrected Y-axis rate used by the controller.
| motor | 46 | calculated L/R `i16[2]`, applied L/R `i16[2]`, sequence `u16`, flags `u16`, transmitted/feedback/CRC/timeout counts `u32[4]`, last/max acknowledgment latency `u32[2]`, last/max applied latency `u32[2]`, transmit rate centi-Hz `u16` |
| odometry | 20 | left/right counts `i32[2]`, velocity milli-units `i32`, timestamp microseconds `u64` |
| active faults | 16 | balance/master/slave/feedback-health masks `u32[4]` |

Status states use `0` BOOT, `1` IMU_CALIBRATING, `2` DISARMED, `3`
ARMED_BALANCE, `4` DRIVING, `5` FALLEN, and `6` FAULT. Active sources use `0`
none, `1` web, `2` serial, and `3` local. Health bits are IMU, calibrated,
motor feedback, loop, output enabled, dry-run, and controller valid in bits
zero through six.

### Configuration

A configuration-read request carries one key byte. Its response and a
configuration-update request both carry:

```text
key:u8 value:i32-le
```

The value is scaled by 1000. Updates are accepted only while DISARMED, are
range-checked, and reset controller history for a bumpless zero-output
transition. They are RAM-only and are not written to flash.

| Key | Setting |
|---:|---|
| 0–3 | inner proportional, integral, derivative, integral limit |
| 4–7 | outer proportional, integral, derivative, integral limit |
| 8 | derivative filter Hz |
| 9 | output limit |
| 10 | slew per second |
| 11 | maximum pitch reference |
| 12–13 | yaw gain and limit |
| 14 | upright offset |
| 15–16 | IMU and wheel signs; values must be exactly `-1` or `1` |

Current diagnostic firmware additionally emits `BALANCE {...}` JSON lines for
human inspection. The Node client discards that text while resynchronizing on
binary markers.

## Reference client

The dependency-free reference implementation is in `tools/pi-client`.

```sh
node tools/pi-client/cli.mjs --port /dev/ttyACM0 status
node tools/pi-client/cli.mjs --port /dev/ttyACM0 arm
node tools/pi-client/cli.mjs --port /dev/ttyACM0 velocity 0.1 0
node tools/pi-client/cli.mjs --port /dev/ttyACM0 yaw 0.1
node tools/pi-client/cli.mjs --port /dev/ttyACM0 stop
node tools/pi-client/cli.mjs --port /dev/ttyACM0 disarm
node tools/pi-client/cli.mjs --port /dev/ttyACM0 telemetry
```

For evidence collection on another computer, use
`tools/capture-balance-evidence.sh` rather than redirecting CLI output. The
capture workflow preserves the raw mixed stream, decoded binary responses,
debug JSON, host queries, operator notes, source metadata, and firmware hash.
See [`remote-balance-evidence.md`](remote-balance-evidence.md).

Protocol tests run with:

```sh
node --test tools/pi-client/test/*.test.mjs
```
