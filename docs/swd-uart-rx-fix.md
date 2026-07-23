# ESP32 to MASTER SWD UART receive fix

## Failure signature

The ESP32 transmitted on GPIO17 and the MASTER observed PA13 edges, but MASTER
telemetry reported zero valid ESP command frames while framing and CRC failures
continued increasing. MASTER to SLAVE traffic and MASTER feedback on PA14 were
healthy, which isolated the failure to the PA13 software receiver.

## Root cause

TIMER14 uses a 1.5-bit first period to sample data bit zero at its center, then
switches to a one-bit period for the remaining data and stop samples. The code
rewrote the timer auto-reload register without explicitly disabling ARR
shadowing. A deferred ARR update can retain the 1.5-bit period for an extra
cycle, while a later byte can start with the previous one-bit period. Either
case shifts sampling out of the bit centers and produces framing and CRC errors.

## Fix

- TIMER14 and TIMER13 explicitly disable auto-reload shadowing.
- Timer periods are derived from the fixed 8 MHz system clock, divide-by-eight
  timer clock, and 19,200-baud target.
- Compile-time checks require a 1 MHz timer, 52-tick bit period, and 78-tick
  first sample.
- Native tests verify every data sample is centered, the stop sample is at tick
  494, and baud error remains below one percent.

## Hardware gate

Flash the new `gausstop_master_swd` image and leave demand disabled. The gate
passes only when `remote_parser` shows increasing received bytes and valid
frames, with stable framing errors and zero transport overflow. Then issue
`clearfault`, require an exact MASTER and SLAVE acknowledgement, and only after
both controllers report READY perform a lifted-wheel low-demand motion test.

The ST-Link must not actively drive SWDIO or SWCLK during the ESP32 traffic test.
Disconnect it after verified flashing, or ensure the probe is electrically
released. Firmware cannot correct bus contention from two active transmitters.
