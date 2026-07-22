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
| `2F 00 00 00 00 40 40` | `0xA117` | `17 A1` |
| `2F 00 00 40` | `0xAB64` | `64 AB` |

**Native-test validated**

## Exact frames

| Frame | Bytes |
|---|---|
| ESP32 command, 9 | `0:2F`, `1..2:speed`, `3..4:steer`, `5:master flags`, `6:slave flags`, `7..8:CRC` |
| Master-to-slave, 6 | `0:2F`, `1..2:signed electrical command`, `3:flags`, `4..5:CRC` |
| Slave feedback, 12 | `0:2F`, `1:state`, `2..5:signed Hall odometer`, `6..9:faults`, `10..11:CRC` |
| Master feedback, 26 | `0..1:CD AB`, `2:master state`, `3:slave state`, `4..5:left command`, `6..7:right command`, `8..11:left odometer`, `12..15:right odometer`, `16..19:master faults`, `20..23:slave faults`, `24..25:CRC` |

Command values are limited to `-1000..1000`. Master flag bit 5 selects direct
left/right interpretation. Bit 0 clears eligible faults, bit 6 disables, and
bit 7 shuts down. Bits 1--4 are reserved-zero. Direct-left/right is illegal in
the slave byte. **Native-test validated**

Parsers use 26-byte fixed storage, a 100 ms partial-frame timeout, marker search,
nested-marker resynchronization after a bad complete frame, and no allocation.
Noise is ignored. A complete bad-CRC or semantically invalid frame faults an
operational role immediately. **Native-test validated**

## Feedback meaning

Commands and odometers are meaningful domain values. An odometer counts legal
Hall transitions and is not a calibrated distance. No voltage, current, or
speed measurement is present. Zero therefore means zero applied/logical command,
zero counted transitions, or no corresponding fault bits; it is not fake sensor
telemetry. **Statically validated**

