# WinXu right-wheel bring-up

This procedure records the successful right-wheel commissioning path. It is a
lifted-wheel test only. Do not test the left motor or reverse direction with
this procedure.

## Wiring used

- Right throttle signal: ESP32 GPIO26 to controller throttle GREEN.
- Signal reference: ESP32 GND to controller throttle BLACK.
- Controller throttle RED: disconnected.
- Right phase and Hall colors: matched color-for-color.
- Electronic Lock RED/PINK: connected whenever the controller is enabled.
- Study connector: disconnected except during controller learning.

GPIO26 produced approximately 0.85 V at idle and 1.65 V at the 25% test
command. A temporary direct GPIO26-to-GREEN connection proved the throttle
path during diagnosis. For permanent wiring, use the protected circuit in
`docs/WIRING.md`: a 1 kOhm resistor inline with the signal and a 10 kOhm
signal-to-ground pull-down. The inline resistor must sit between GPIO26 and
GREEN; the two wires must not touch around it.

## Controller learning

1. Lift and secure the right wheel. Leave the left controller unpowered.
2. Power the ESP32 first and confirm GPIO26 is at its idle voltage.
3. Power the WinXu controller with Electronic Lock RED/PINK connected.
4. Connect the two Study wires. Keep hands clear; the controller may rotate
   the wheel without an ESP32 command.
5. Observe smooth rotation for 5-10 seconds.
6. Disconnect Study while leaving the controller powered. Do not reconnect it
   for the software test.
7. If the wheel chatters, jerks, vibrates, or spins violently, disconnect Study
   and controller power immediately. Do not retry by randomly swapping wires.

## Bounded software test

The test uses the repository `SerialMotorTransport`; it does not bypass the
firmware protocol. Its sequence is HELLO/CAPABILITIES, operating mode, zero
lease, ARM, five acknowledged zero commands, 20 acknowledged motion frames at
20 Hz, immediate zero, DISARM, and disconnect.

First inspect the calculated mixer output without enabling motion:

```sh
PYTHONPATH=donkeycar donkeycar/.venv/bin/python \
  donkeycar/scripts/right_wheel_test.py
```

Expected output is LEFT `0.000000`, RIGHT `0.250000`. To perform the single
lifted-wheel pulse:

```sh
PYTHONPATH=donkeycar donkeycar/.venv/bin/python \
  donkeycar/scripts/right_wheel_test.py \
  --port /dev/cu.usbserial-0001 \
  --execute --wheel-lifted --study-disconnected
```

Do not extend the duration, increase the command, automatically repeat a
failed physical test, or lower the wheel during commissioning. The zero and
DISARM operations are attempted in `finally` even if serial communication
fails.

## Observed result

The controller self-learning rotation was smooth. The first software tests
received valid ACKs but did not move the wheel because the throttle signal was
near zero at the controller connector. GPIO26 measured 1.65 V when isolated,
which localized the fault to the throttle interconnect. After correcting the
connection, the one-second right-only command moved the wheel successfully and
the final zero and DISARM commands were acknowledged.
