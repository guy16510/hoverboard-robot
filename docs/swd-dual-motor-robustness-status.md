# SWD Dual-Motor Robustness Status

This workspace contains a locally built, uncommitted candidate for unloaded
dual-wheel validation. It does not claim hardware success until both Hall
odometers advance in a bounded lifted-wheel test.

## Root causes corrected

- The previous bridge path disabled the floating phase. The physically proven
  legacy GAUSSTOP drive enabled all three main/complementary phase pairs, with
  source at midpoint plus offset, sink at midpoint minus offset, and the third
  phase at midpoint. The selected bridge profile now reproduces that behavior.
- The previous board adapter disabled and re-enabled TIMER0 outputs on every
  1 ms service pass. The bridge now starts once and uses the proven shadowed
  compare-update sequence while active.
- MASTER measured a missing SLAVE acknowledgment from the timestamp of the
  latest duplicate ESP32 heartbeat. A 20 ms duplicate stream could therefore
  prevent the 100 ms acknowledgment watchdog from expiring forever. The timeout
  now starts when a new sequence is first forwarded.
- ESP32 changed its sequence whenever the ramp changed, faster than the
  47-byte/19,200-baud feedback chain could acknowledge it. Commands now use
  stop-and-wait: resend the in-flight sequence until the MASTER and SLAVE
  acknowledgments match. Disable/shutdown still preempt an in-flight command.
- The previous drive helper could report a successful transport-only run with
  zero mechanical movement. It now requires valid Hall values, bridge/compare
  agreement, a fresh exact acknowledgment, bridge activation for every
  commanded wheel, and a Hall-odometer delta in each commanded logical
  direction.
- Disabling the safety supervisor previously cleared active demand before the
  bridge-off timestamp could be recorded. Fault clear now remains blocked until
  250 ms after demand and bridge output are actually off.
- Ramp arithmetic previously rounded every 1 ms step up to a full command unit,
  making the configured rates ineffective, and reversal dwell accumulated a
  hidden ramp. Fractional ramp progress is now preserved, reversal restarts from
  zero, and startup motion timeout begins only when the bridge actually enables.
- Hall tracking is now re-anchored whenever the bridge is fully off so wheel
  coasting during stop or reversal cannot be misclassified as an illegal
  energized transition.
- End-to-end clear confirmation now remains pending until both MASTER and SLAVE
  have completed their zero-output dwell and cleared their faults.
- ESP32 ramping now remains at zero until the exact safe zero-demand handshake
  completes, preventing a precharged command jump when the session becomes
  ready.
- ESP32 runtime gating now rejects contradictory Hall, PA4, fault-clear,
  peer-health, applied-command, compare, and bridge telemetry. Sub-deadband
  ramp values remain valid while their bridges are correctly off.

## Selected motor behavior

- SWD-header transport remains 19,200 baud on ESP32 GPIO17 TX/GPIO35 RX and
  MASTER PA13 RX/PA14 software TX.
- The conservative command profile maps command 250 to the physically proven
  100-tick differential around a 500-tick midpoint.
- All 12 valid Hall/direction combinations are native-tested for
  source=600, sink=400, floating=500, with all three phase pairs enabled.
- Active commutation updates do not cycle the primary output off and on.
- The MASTER-only PA4 bypass remains an explicit build variant. Raw MASTER PA4,
  bypass state, and raw SLAVE PA4 remain visible in feedback. SLAVE PA4 is never
  bypassed.

## Transport and validation gates

- Protocol v2 carries accepted ESP32, forwarded SLAVE, and accepted SLAVE
  sequence numbers.
- Nonzero demand is held at zero until a healthy zero-demand `READY,READY`
  acknowledgment completes.
- MASTER stops both demands on stale/faulted SLAVE feedback or a missing
  acknowledgment of the actual forwarded sequence.
- Feedback includes both Hall states, compare offsets, bridge-enable states,
  both odometers, raw PA4 information, link ages, faults, and transport counts.
- `tools/drive_esp32.py` always sends stop and disable on exit and records JSONL
  evidence.

## Validation completed

- 2,027 strict native assertions pass, including exhaustive rejection of all
  704 single-bit corruptions across the four wire-frame formats.
- The same 2,027 assertions pass with AddressSanitizer and
  UndefinedBehaviorSanitizer.
- Fourteen host transport and motion-gate tests verify progress, clear
  completion, bridge state, Hall movement, and commanded odometer direction
  enforcement.
- All nine firmware environments compile with warnings treated as errors.
- Final MASTER SWD, MASTER PA4-bypass, SLAVE, and ESP32 SWD images pass the
  repository image-size and static-RAM checks.
- A clean rebuild of all seven flashed binary regions is byte-for-byte
  identical, including the ESP32 boot-app selector at address `0xe000`.
- Formatting, source classification, SPDX/license, architecture, shell syntax,
  and automatic hardware-access isolation checks pass.

## Hardware status

The final images have not been flashed, read back, or motion-tested. No serial
controller or ST-Link was connected during this work. Do not perform a ground
test. The next hardware action is one exact-readback-verified flash of the three
selected images, followed by a zero-demand check and one bounded 250/250
lifted-wheel run. Stop after the first mismatch; do not iterate through guessed
firmware images.
