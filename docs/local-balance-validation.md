# Local balance-layer validation record

Date: 2026-07-24

## Baseline

- Branch: `main`
- Commit: `39974c693d5cbe0de89654d63647eb16e3b416de`
- Initial status: clean (`main...origin/main`)
- No commit, push, pull request, tag, release, or Git-history change performed.
- Working firmware backup:
  `/private/tmp/hoverboard-robot-baseline-39974c693d5cbe0de89654d63647eb16e3b416de`

Baseline working binary SHA-256:

| Artifact | SHA-256 |
|---|---|
| ESP32 | `d4ce951cd1afbf6d8cba68bbde645afbfe9a048c7d4651ed6f288d88435bb0fe` |
| MASTER PA4 enforced | `6e2c0685a6c8d84997c5775d35533abdc66fd69cd24f96082b86e41a5b53b1c4` |
| MASTER PA4 bypass | `79dc8f436524ab997505155d8484a738ab0cccc3a9dc6b8c6700cdf590dca941` |
| SLAVE | `bbab760ec041d914224e85c3f191e8ea8ccda8cd279c9a3a963ddee22ec64148` |

## Baseline results before source changes

- All nine existing firmware environments built successfully.
- `tools/test-all.sh` failed to link because the native shell runners omitted
  `test_swd_timing.c`, `test_resync.c`, and the pulse decoder source.
- Static checks failed on pre-existing clang-format drift in the SWD transport,
  coordinator, and tests.
- The test-runner omission was repaired locally; no behavioral GD32 source was
  changed for the balance layer.

## Host validation completed

- Existing native and sanitized suites: 760,181 assertions each.
- New ESP32 balance suite: 2,197 assertions in both normal and
  AddressSanitizer/UndefinedBehaviorSanitizer runs.
- Existing Python drive-tool suite: 19 tests.
- Node serial protocol and evidence-decoder suite: 15 tests.
- MPU register selection, byte decoding, address scan, identity rejection,
  gyro bias, plausibility, timeout, and error counters are covered.
- Complementary-filter convergence and invalid delta time are covered.
- PID terms, filtering, integral clamps, saturation, slew, bumpless reset,
  generated trace signs, state transitions, fall detection, leases, disconnect,
  dry-run, timing metrics, CRC, malformed frames, stale sequence, and rollover
  are covered.
- Conditional anti-windup, non-finite controller shutdown, motorless diagnostic
  recovery, explicit command-source handoff, bounded binary response queues,
  all binary telemetry schemas, range-checked RAM-only configuration, and
  serial-session reset-to-zero behavior are covered.
- The controller, approximately-upright arming gate, and fall detector share
  one pitch frame incorporating the configured IMU sign and mechanical offset.
- Every payload width through 48 bytes is round-trip tested, every byte in a
  maximum-size frame is tested for single-bit corruption, every arming gate is
  failure-injected, and the Node encoder rejects values that do not fit their
  wire fields.
- A no-flash remote capture workflow preserves mixed raw serial, decoded binary
  and debug telemetry, outbound queries, operator notes, firmware/source
  identity, session metadata, and checksums for hardware attached elsewhere.
- MPU telemetry includes accepted calibration progress, raw gyro, corrected
  gyro, and the calculated three-axis gyro-bias vector. Binary telemetry carries
  raw plus bias so corrected values can be reproduced; debug JSON records all
  three explicitly.
- A 10,000-command local transport soak model verifies 50.005 Hz modeled
  throughput, 1 ms acknowledgment latency, and 3 ms applied-command latency.
  These are generated inputs, not hardware measurements.
- The generated pendulum trace reduces a 5-degree error without exceeding
  6 degrees. This is a regression model, not a physical stability claim.
- Worst-case modeled 9-byte pulse duration at an 80 microsecond unit is
  15.2 ms, a theoretical 65.789 Hz ceiling. The retained firmware heartbeat is
  10 Hz. This is not a sustainable hardware rate or latency measurement.

## Firmware build status

- `esp32_balance_coordinator`: dry-run enabled, web disabled, Wi-Fi library
  excluded; 25,884 bytes RAM and 315,705 bytes flash.
- `esp32_balance_web`: dry-run enabled, optional static page and WebSocket
  adapter compiled; 50,540 bytes RAM and 824,349 bytes flash.
- Dry-run coordinator firmware SHA-256:
  `155f894117ebcfef629b8f0e0e1e61844a7d70a5e51bde362213312256082dd2`.
- Optional web firmware SHA-256:
  `8a59f0b50c4f5ec6993568ad36de2437472385c117692abdda8403690fc3d5d4`.
- MASTER and SLAVE firmware sources were not changed for this control layer and
  neither controller was flashed.
- No device was enumerated, opened, flashed, powered, or moved during local
  validation.

## Output restriction audit

The working SWD MASTER and SLAVE select power profile 1:

- timer clock: 8 MHz;
- center-aligned period: 999;
- inferred PWM frequency: 4 kHz;
- midpoint: 500;
- startup compare offset: 40;
- maximum compare offset: 100;
- command deadband: 50;
- full-scale command: 250;
- acceleration: 400 command units/s;
- deceleration: 800 command units/s;
- direction dwell: 250 ms.

Therefore command `250` already reaches compare offset 100 in the active SWD
profile. Command `1000` produces no additional drive. The console, wheel mix,
motor clamp, and Python tool allow an absolute protocol range up to 1000, while
the Python tool requires an explicit override above 250. None of these power or
bench restrictions were changed.

## Requirement audit

| Objective area | Current evidence | Status |
|---|---|---|
| Baseline preservation and local-only Git constraints | original branch/SHA/status and artifact hashes above; backup retained; current branch/SHA unchanged | complete |
| MPU6050 acquisition and diagnostics | host tests cover addresses, identity, register setup, decoding, configuration rejection, calibration, plausibility, counters, and timeout; both firmware images build | locally complete; physical identity/rate pending |
| Pitch estimator and 200 Hz scheduler | generated traces, invalid-delta reset, loop statistics, bounded lower-priority service, scheduler output gate, and task watchdog | locally complete; physical rate/jitter/sign pending |
| Cascaded controller and state machine | pitch PID, velocity PI, yaw mix, conditional anti-windup, limits, slew, non-finite rejection, bumpless transitions, all named states and gates | locally complete; hand-tilt direction pending |
| Mandatory dry-run | compile-time default, disabled-zero sink test, motorless diagnostic behavior, and firmware build | locally complete; physical hand-tilt pending |
| Raspberry Pi protocol and client | framing/CRC/sequence/session/lease/configuration/telemetry tests plus dependency-free CLI | complete locally |
| Optional removable web adapter | excluded from default dependency graph; explicit web build; bounded network writes; disconnect/zero behavior | complete locally |
| Motor transport | pulse-duration bounds, 10,000-command metrics soak model, live latency instrumentation in firmware | local instrumentation complete; physical throughput/latency/motion benchmark pending |
| Output restrictions and hard safeguards | audit above plus runtime feedback-health, Hall, timeout, scheduler, watchdog, fall, and explicit-rearm gates | complete locally |
| Physical stages 3–6 | no device accessed and no physical result inferred | pending user-assisted gated validation |

## Physical evidence still required

| Measurement | Result |
|---|---|
| MPU address / identity | not measured |
| MPU sample rate | not measured |
| Control-loop rate / jitter | not measured |
| Pitch direction / magnitude | not measured |
| Stationary gyro stability | not measured |
| Sustainable motor-command rate | not measured |
| Acknowledgment latency | not measured |
| Command-to-applied latency | not measured |
| CRC / overflow under wheel motion | not measured |
| Dry-run corrective direction by hand | not measured |
| Lifted-wheel correction | not performed |
| Ground balance | prohibited for this task |

The next step is the single grouped physical MPU diagnostic in
`docs/balance-wiring.md`. Live motor output must remain disabled.
