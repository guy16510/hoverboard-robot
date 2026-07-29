# Wire protocol

All integer serialization is explicit. Signed integers are two's-complement,
little-endian. CRC bytes are low byte first. Raw compiler structs are never
transmitted. **Native-test validated**

## CRC-16

- Polynomial: `0x1021`
- Initial value: `0x0000`
- Input/output reflection: none
- Final XOR: none
- Coverage: every frame byte before the two CRC bytes

Known answers shared by GD32, ESP32, and native tests:

| Input | CRC | Wire bytes |
|---|---:|---|
| ASCII `123456789` | `0x31C3` | n/a |

**Native-test validated**

## Exact frames

| Frame | Bytes |
|---|---|
| ESP32 command, 11 | `0:42`, `1..2:sequence`, `3..4:speed`, `5..6:steer`, `7:master flags`, `8:slave flags`, `9..10:CRC` |
| Master-to-slave, 8 | `0:42`, `1..2:sequence`, `3..4:signed electrical command`, `5:flags`, `6..7:CRC` |
| Slave feedback, 28 | `0:43`, `1:state`, `2..3:accepted sequence`, `4..5:applied electrical command`, `6..9:signed Hall odometer`, `10..13:faults`, `14..15:command age`, `16:qualified Hall`, `17:motor status`, `18..19:compare offset`, `20..21:Hall glitches`, `22..23:invalid commands`, `24..25:command framing errors`, `26..27:CRC` |
| Master feedback, 67 | `0..1:CE B3`, `2:protocol version`, `3:master state`, `4:slave state`, `5:status`, `6..7:accepted ESP sequence`, `8..9:forwarded SLAVE sequence`, `10..11:accepted SLAVE sequence`, `12..15:applied commands`, `16..23:odometers`, `24..31:faults`, `32..37:three link ages`, `38..39:qualified Hall states`, `40..43:compare offsets`, `44:motor status`, `45..52:remote RX bytes, valid frames, invalid frames, and framing errors`, `53..56:left/right Hall glitches`, `57..64:invalid/framing counters for SLAVE feedback and commands`, `65..66:CRC` |

MASTER status byte bits 4 through 7 latch the source of transport overflow:
remote RX, remote TX, MASTER–SLAVE RX, and MASTER–SLAVE TX, respectively.

Command values are limited to `-1000..1000`. Master flag bit 5 selects direct
left/right interpretation. Bit 0 clears eligible faults, bit 6 disables, and
bit 7 shuts down. Bits 1--4 are reserved-zero. Direct-left/right is illegal in
the slave byte. **Native-test validated**

Parsers use 67-byte fixed storage, a 100 ms partial-frame timeout, marker search,
nested-marker resynchronization after a bad complete frame, and no allocation.
Noise is ignored. A complete bad-CRC or semantically invalid frame faults an
operational role immediately. **Native-test validated**

## Feedback meaning

Master feedback status byte bits are peer healthy, MASTER PA4 raw high,
MASTER PA4 bypass active, and end-to-end fault clear pending in bits 0 through
3. Slave feedback motor-status bits are bridge enabled, SLAVE PA4 raw high, and
SLAVE fault clear pending in bits 0 through 2. The MASTER clear-pending bit is
set while either controller is still completing its safe clear sequence.

Commands and odometers are meaningful domain values. An odometer counts legal
Hall transitions and is not a calibrated distance. Compare offsets and bridge
bits report commanded TIMER0 output state, not a measured phase voltage or
current. No calibrated voltage, current, or speed measurement is present. Zero
therefore means zero applied/logical command, zero counted transitions, or no
corresponding fault bits; it is not fake sensor telemetry.
**Statically validated**
