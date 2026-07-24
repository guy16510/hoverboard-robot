# Balance controller tuning guide

The source defaults are conservative software starting values, not final gains
and not evidence that the robot balances physically.

The controller is cascaded:

```text
velocity PI -> pitch reference
pitch PID + velocity correction -> common output
yaw request -> differential output
left  = common + yaw
right = common - yaw
```

All signs are explicit. Confirm MPU pitch sign and both wheel signs before
changing gains.

The configured IMU sign and upright mechanical offset define one balance-frame
pitch used consistently by the pitch controller, approximately-upright arming
gate, and fall detector. A nonzero mounting offset must therefore be validated
in dry-run telemetry before arming; it is not merely a display correction.

## Required order

1. MPU diagnostic, motor power removed:
   confirm address, stationary bias, sample rate, pitch sign, magnitude, and
   timeout behavior.
2. Controller dry-run, motor power removed:
   set velocity and yaw to zero; tilt by hand; verify the calculated common
   output has a mathematically corrective sign. Confirm fall detection.
3. Motor transport benchmark, wheels lifted and secured:
   measure sustainable command rate, acknowledgment latency, applied-command
   latency, CRC growth, overflow, stop, and disable.
4. Lifted-wheel correction only:
   use bounded output and immediately stop if either wheel reinforces the tilt.
5. Ground balancing is outside this task.

## Inner pitch loop

Start with integral disabled. Increase proportional gain only until a small
hand tilt produces a clear corrective dry-run response. Add derivative damping
to reduce overshoot and noise. Enable a small integral term only to remove a
persistent mechanical offset. Keep derivative filtering enabled.

Never tune around a wrong sign. A wrong sign must be corrected with the
configuration sign, not a negative gain hidden in the tuning.

The binary protocol can read or update one scaled configuration key at a time.
Updates are range-checked, RAM-only, and accepted only in `DISARMED`; changing
a value resets controller history to zero. Key definitions and scaling are in
`docs/pi-serial-protocol.md`.

## Outer velocity loop

Tune only after the pitch loop is corrective in a lifted-wheel test. Keep the
maximum pitch reference small. Increase velocity proportional gain gradually;
add slow integral action only after odometry direction and scaling are known.

## Yaw

Verify left/right mixing independently at zero common output. Positive yaw must
increase one logical wheel and decrease the other by the same bounded amount.
Keep yaw correction below the common-output headroom so saturation preserves a
balanced differential.

## Safeguards that remain active

- integral clamps and anti-windup;
- derivative low-pass filtering;
- pitch-reference, yaw, and total-output saturation;
- output slew limiting;
- fall angle and duration;
- IMU plausibility and timeout;
- loop-overrun tracking;
- controller-feedback freshness and exact acknowledgment;
- Hall, watchdog, CRC, stale-command, and transport-overflow faults;
- explicit rearm after `FALLEN` or `FAULT`;
- mandatory default dry-run.
