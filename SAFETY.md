# Safety policy

This is software policy, not proof of electrical safety. All physical behavior
remains **Awaiting hardware validation**.

## Initial limits and rationale

| Limit | Value/unit | Source | Reason | Validation on this computer |
|---|---:|---|---|---|
| PWM ceiling | 80 compare ticks | conservative plan selection below historic 400 | bound first energized validation | **Native-test validated**, **Build validated** |
| Startup offset | 40 compare ticks | legacy operational behavior | bounded starting authority | **Native-test validated** |
| Command deadband | 50 command units | legacy software | avoid sub-start oscillation | **Native-test validated** |
| Acceleration | 400 command units/s | legacy internal limit | conservative increase | **Native-test validated** |
| Deceleration | 800 command units/s | legacy internal limit | reach coast sooner | **Native-test validated** |
| ESP32 heartbeat | 50 ms | protocol requirement | deterministic updates | **Statically validated** |
| ESP32 timeout | 400 ms | reconciles legacy 500/600 ms | stop demand below 500 ms maximum | **Native-test validated** |
| Master/slave timeout | 100 ms | new conservative policy | fast subordinate stop | **Native-test validated** |
| Startup timeout | 700 ms | legacy safety | require Hall progress | **Native-test validated** |
| Stall timeout | 300 ms | legacy safety | stop absent transitions | **Native-test validated** |
| Minimum Hall interval | 500 microseconds | legacy safety | reject implausibly fast transitions | **Native-test validated** |
| Direction dwell | 250 ms | legacy safety | coast at zero before reversal | **Native-test validated** |
| Fault-clear safe time | 250 ms bridge-off | new conservative policy | explicit quiet interval | **Native-test validated** |
| Watchdog | nominal 1 second | plan requirement | detect stalled loop | **Build validated** only |

Normal `stop` ramps targets to zero, disables every timer output below deadband,
and coasts. Direction changes decelerate to zero, remain bridge-off for 250 ms,
then restart. Regenerative and active braking are not implemented.
**Native-test validated**

Disable, shutdown, command/link timeout, protocol fault, invalid Hall, illegal
Hall transition, too-fast Hall transition, startup/stall timeout, PA4 low, PA6
fault, ADC-calibration fault, and watchdog lockout all force the bridge off.
Faults latch. Watchdog reset and command shutdown require reset; other eligible
faults require disabled zero demand, safe inputs, and 250 ms bridge-off before
explicit clear. **Native-test validated** and **Statically validated**

PA6 takes 16 bridge-off samples. Spread above 64 counts faults calibration.
After calibration, values below baseline minus 600 or above baseline plus 1400
fault protection. This is relative only, not current or voltage calibration.
**Native-test validated**; thresholds are **Awaiting hardware validation**

PA4 is active-low digital with a weak pull-up. Its source is unknown. PB12 is
input-only and TIMER0 hardware break is disabled. PB2 stays high because its
exact circuit role is unresolved. **Historically physically verified** behavior
with remaining items **Awaiting hardware validation**

