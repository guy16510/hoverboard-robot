# ESP32 Robot Carrier PCB

Carrier PCB for a 30-pin ESP32 DevKit V1, robot sensors, dual motor-controller low-voltage I/O, and Raspberry Pi/ESP32 low-voltage power distribution.

## Power architecture

- Input to this PCB is **regulated 5 V only**. Do not connect the 10S battery directly to this board.
- Use an external 36-42 V to 5 V buck converter sized for the Raspberry Pi and peripherals.
- The external 5 V rail feeds the Raspberry Pi, ESP32 VIN, ultrasonic modules, Hall sensors, and servo connector.
- MPU6050 is powered from ESP32 3.3 V.
- All low-voltage grounds share a common ground plane.
- Motor phase/current paths do NOT pass through this PCB.
- Motor-controller throttle +5 V reference wires must NOT be tied to this board's 5 V rail.

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

- J1 5V IN: +5V / GND, from external regulated buck converter
- J2 PI 5V OUT: +5V / GND, high-current connector
- J3 LEFT MOTOR CTRL: THROTTLE / REVERSE / BRAKE / GND
- J4 RIGHT MOTOR CTRL: THROTTLE / REVERSE / BRAKE / GND
- J5 RIGHT HALL: +5V / HALL_A / HALL_B / HALL_C / GND
- J6 LEFT HALL: +5V / HALL_A / HALL_B / HALL_C / GND
- J7 FRONT ULTRASONIC: +5V / TRIG / ECHO / GND
- J8 LEFT ULTRASONIC: +5V / TRIG / ECHO / GND
- J9 RIGHT ULTRASONIC: +5V / TRIG / ECHO / GND
- J10 MPU6050: +3V3 / SDA / SCL / GND
- J11 SERVO: +5V / SIGNAL / GND
- J12 SPARE POWER: +5V / +3V3 / GND

## Input protection / level shifting

All six Hall inputs and all three ultrasonic ECHO inputs use resistor dividers before reaching the ESP32. Nominal values are 10 kOhm upper and 18 kOhm lower, yielding approximately 3.21 V from a 5 V input. Optional 100 nF Hall-filter footprints may be fitted if noise requires them.

## Reverse and brake outputs

GPIO27, GPIO14, GPIO33, and GPIO32 drive four low-side N-channel MOSFET stages. Each stage presents an open-drain output to the motor-controller reverse/brake wire and sinks it to common ground when active. Use 100 Ohm gate resistors and 100 kOhm gate pull-downs so reverse and brake remain OFF during reset.

## Throttle outputs

GPIO25 and GPIO26 are routed to the left and right throttle signal connectors through 1 kOhm series resistors. Optional 1 uF RC filter footprints are provided. Motor-controller throttle ground must connect to PCB GND.

## PCB layout requirements

- 2-layer board is sufficient.
- Use a wide +5V plane/trace and solid GND plane for Raspberry Pi/servo current.
- Put J1 external 5V input and J2 Pi output close together.
- Keep Hall and ultrasonic signal traces away from servo/high-current 5V routing.
- Add four M3 mounting holes.
- Put connector names and pin functions on silkscreen.
- Add test points for +5V, +3V3, GND, both throttle outputs, and each Hall input.
- Clearly silk-screen **5V ONLY - NO BATTERY VOLTAGE** next to J1.

## Release status

The high-voltage 36-42 V input and onboard buck converter have been removed. This carrier is low-voltage only. The schematic still requires final footprint assignment, physical ESP32 header-spacing verification, PCB placement/routing, ERC/DRC, and Gerber/drill generation before fabrication. Do not order from JLCPCB until those steps are complete.
