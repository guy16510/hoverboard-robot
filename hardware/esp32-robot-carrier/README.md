# ESP32 Robot Carrier PCB

Carrier PCB for a 30-pin ESP32 DevKit V1, robot sensors, dual motor-controller low-voltage I/O, and Raspberry Pi power.

## Power architecture

- Input: 10S battery, 36 V nominal / 42 V full charge.
- On-board 5 V buck: TPS54560-class 60 V / 5 A converter, target output 5.1 V.
- 5.1 V rail powers the Raspberry Pi, ESP32 VIN, ultrasonic modules, Hall sensors, and servo connector.
- MPU6050 is powered from ESP32 3.3 V.
- All low-voltage grounds share a common ground plane.
- Motor phase/current paths do NOT pass through this PCB.
- Motor-controller throttle +5 V reference wires must NOT be tied to this board's 5.1 V rail.

## ESP32 pin map

| Function | GPIO |
|---|---:|
| Left throttle | 25 |
| Right throttle | 26 |
| Left reverse | 27 |
| Right reverse | 14 |
| Left brake | 33 |
| Right brake | 32 |
| Right Hall A/B/C | 19 / 21 / 22 |
| Left Hall A/B/C | 16 / 17 / 36 |
| Front ultrasonic TRIG | 5 |
| Front ultrasonic ECHO | 34 |
| Left ultrasonic TRIG | 15 |
| Left ultrasonic ECHO | 35 |
| Right ultrasonic TRIG | 18 |
| Right ultrasonic ECHO | 39 |
| MPU6050 SDA | 4 |
| MPU6050 SCL | 23 |
| Servo signal | 13 |

## Connector plan

- J1 BATTERY IN: BAT+ / GND
- J2 PI 5V OUT: +5V1 / GND, high-current connector
- J3 LEFT MOTOR CTRL: THROTTLE / REVERSE / BRAKE / GND
- J4 RIGHT MOTOR CTRL: THROTTLE / REVERSE / BRAKE / GND
- J5 RIGHT HALL: +5V1 / HALL_A / HALL_B / HALL_C / GND
- J6 LEFT HALL: +5V1 / HALL_A / HALL_B / HALL_C / GND
- J7 FRONT ULTRASONIC: +5V1 / TRIG / ECHO / GND
- J8 LEFT ULTRASONIC: +5V1 / TRIG / ECHO / GND
- J9 RIGHT ULTRASONIC: +5V1 / TRIG / ECHO / GND
- J10 MPU6050: +3V3 / SDA / SCL / GND
- J11 SERVO: +5V1 / SIGNAL / GND
- J12 SPARE POWER: +5V1 / +3V3 / GND

## Input protection / level shifting

All six Hall inputs and all three ultrasonic ECHO inputs use resistor dividers before reaching the ESP32. Nominal values: 10 kOhm upper resistor and 18 kOhm lower resistor, yielding approximately 3.21 V from a 5 V input. Add 100 nF optional filter footprints on Hall inputs if noise requires them.

## Reverse and brake outputs

GPIO27, GPIO14, GPIO33, and GPIO32 drive four low-side N-channel MOSFET stages. Each stage presents an open-drain output to the motor-controller reverse/brake wire and sinks it to common ground when active. Use 100 Ohm gate resistors and 100 kOhm gate pull-downs so reverse and brake remain OFF during reset.

## Throttle outputs

GPIO25 and GPIO26 are routed to the left and right throttle signal connectors through 1 kOhm series resistors. Provide optional RC filter footprints (1 kOhm + 1 uF) so firmware may use either the ESP32 DAC output directly or a smoothed waveform. Motor-controller throttle ground must connect to PCB GND.

## 5.1 V regulator section

The schematic reserves the TI TPS54560 60 V / 5 A buck topology and follows the TPS54560EVM-515 architecture as the reference implementation. The final PCB layout must keep the switch node compact, use a solid ground plane, large input/output copper, thermal vias under the exposed pad, and keep sensor traces away from the switching node.

Recommended protection at battery input:

- replaceable fuse or resettable fuse footprint
- reverse-polarity protection footprint
- 60 V+ TVS footprint
- bulk input capacitor rated at least 63 V

## PCB layout requirements

- 2-layer minimum, 2 oz copper preferred if the Pi 5 V rail is routed on-board.
- Keep the buck converter and Pi power connector together at one edge.
- Keep Hall/ultrasonic inputs on the opposite side from the buck switch node.
- Use wide pours for BAT+, +5V1, and GND.
- Add four M3 mounting holes.
- Put connector names and pin functions on silkscreen.
- Add test points for BAT+, +5V1, +3V3, GND, both throttle outputs, and each Hall input.

## Release status

This directory defines the updated electrical design and schematic source. Do not send Gerbers to fabrication until the KiCad ERC/DRC pass, the exact ESP32 DevKit footprint is checked against the physical board, the chosen JLCPCB-available buck components are verified, and the 5 V rail is load-tested at Raspberry Pi current.