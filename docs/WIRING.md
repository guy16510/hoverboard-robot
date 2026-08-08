# Robot wiring, WinXu 36/48 V 350 W 18 A controllers + ESP32

This repository now assumes two independent WinXu 36/48 V 350 W 18 A brushless controllers, one controller per hoverboard wheel, plus one ESP32 DevKit / ESP32-WROOM-32.

The old GAUSSTOP controller wiring and firmware are intentionally gone.

## Controller reference image

This is the wiring diagram from the eBay listing for the WinXu 36/48 V 350 W 18 A controller family:

![WinXu 36/48 V 350 W controller wiring diagram](https://i.ebayimg.com/images/g/fH4AAOSwOzhmYp1z/s-l1600.webp)

Source listing: https://www.ebay.com/itm/116065871852

Important: WinXu sells several harness variants under the 350 W / 18 A family. The label on the actual wire or connector wins over wire color. Do not guess that an unlabeled black 2-pin plug is reverse.

## ESP32 pin map

| Function | ESP32 GPIO | Notes |
|---|---:|---|
| Left throttle | 25 | DAC1, analog throttle signal |
| Right throttle | 26 | DAC2, analog throttle signal |
| Left reverse | 27 | Drives an optocoupler or dry-contact relay only |
| Right reverse | 14 | Drives an optocoupler or dry-contact relay only |
| Left low brake | 33 | Drives an optocoupler or dry-contact relay only |
| Right low brake | 32 | Drives an optocoupler or dry-contact relay only |
| Front ultrasonic trigger | 16 | HC-SR04 TRIG |
| Front ultrasonic echo | 34 | Input only, level shift to 3.3 V |
| Left ultrasonic trigger | 17 | HC-SR04 TRIG |
| Left ultrasonic echo | 35 | Input only, level shift to 3.3 V |
| Right ultrasonic trigger | 18 | HC-SR04 TRIG |
| Right ultrasonic echo | 39 | Input only, level shift to 3.3 V |
| Common logic ground | GND | ESP32, throttle grounds, ultrasonic grounds |

GPIO 25 and 26 are the ESP32 DAC outputs. The firmware uses real DAC voltage, not PWM, so no PWM-to-analog RC filter is required.

```text
ESP32                     LEFT CONTROLLER
GPIO25  --1k--+---------- Throttle signal
              |
             10k
              |
GND ----------+---------- Throttle ground
GPIO27 -> isolator ------ Reverse switch pair, only if labeled Reverse
GPIO33 -> isolator ------ Low-brake switch pair

ESP32                     RIGHT CONTROLLER
GPIO26  --1k--+---------- Throttle signal
              |
             10k
              |
GND ----------+---------- Throttle ground
GPIO14 -> isolator ------ Reverse switch pair, only if labeled Reverse
GPIO32 -> isolator ------ Low-brake switch pair
```

The 10 kOhm throttle pull-down is intentional. It holds the controller throttle signal near zero while the ESP32 is unpowered, resetting, or before firmware configures the DAC pin.

## One motor controller

Repeat this section for left and right controllers.

### Battery and motor

1. Controller thick red goes to battery positive through an appropriately sized fuse.
2. Controller thick black goes to battery negative.
3. Controller thick yellow, green and blue motor phase wires go to the wheel phase wires. Start color-to-color.
4. Connect the motor Hall connector. Typical Hall wiring is red +5 V, black ground, and yellow/green/blue Hall signals.
5. Connect the controller power-lock / key wire exactly as labeled by the controller documentation. For a robot, put the controller enable path behind a physical emergency-stop or keyed power switch rather than depending on software alone.
6. Run the controller self-learning procedure with the wheel lifted before using ESP32 throttle control. Disconnect the self-learning connector after learning.

### Throttle

Use the controller connector labeled `Throttle`.

Typical throttle wiring is:

| Controller throttle lead | Connection |
|---|---|
| +5 V, normally red | Leave disconnected from ESP32 |
| GND, normally black | ESP32 GND |
| Signal, often green or white | Left GPIO25 or right GPIO26 through a 1 kOhm series resistor, plus 10 kOhm signal-to-GND pull-down |

Do not connect the controller's 5 V throttle supply to an ESP32 GPIO or to the ESP32 3.3 V rail.

The controller family expects roughly a 1.1 V to 4.2 V throttle signal. An ESP32 DAC can only generate about 0 V to 3.3 V. The firmware intentionally limits drive voltage to about 3.15 V, so maximum controller output is intentionally below absolute full throttle. That is a good starting point for this robot.

### Low-level brake

Use the connector physically labeled `Low brake` or `Low level brake`.

The controller brake input is a switch circuit. The firmware assumes closing the brake pair cuts motor power. Do not connect either controller brake lead directly to an ESP32 GPIO.

Wire each brake pair through an isolated dry-contact relay or an optocoupler circuit:

```text
ESP32 GPIO33 (left)  -> isolator -> LEFT controller low-brake pair
ESP32 GPIO32 (right) -> isolator -> RIGHT controller low-brake pair
```

The firmware asserts both brakes whenever it is disarmed, whenever the Pi command lease expires, and while changing motor direction. A physical emergency-stop or controller power-lock remains required because software braking cannot protect against every reset, wiring fault, or failed ESP32.

### Reverse

Runtime reverse is supported in firmware on GPIO27 and GPIO14, but it only works if your exact controller has a connector explicitly labeled `Reverse`.

```text
ESP32 GPIO27 -> isolator -> LEFT controller reverse switch pair
ESP32 GPIO14 -> isolator -> RIGHT controller reverse switch pair
```

Do not substitute the self-learning connector for runtime reverse. Self-learning is an installation procedure, not a direction command.

Some WinXu 350 W harness variants do not expose runtime reverse. If the actual harness does not have a labeled reverse input, set `WINXU_RUNTIME_REVERSE=0` in the PlatformIO build flags before testing. With that option, negative commands stop instead of accidentally applying forward throttle.

## Mirrored hoverboard wheels

The default firmware assumes the wheels are mounted as mirror images:

```text
WINXU_LEFT_MOTOR_SIGN=1
WINXU_RIGHT_MOTOR_SIGN=-1
```

With the robot lifted, command a very small positive Donkeycar throttle. Both tire contact patches must move in the robot's forward direction. If one wheel is wrong, change that motor sign in `platformio.ini`. Do not compensate by randomly swapping Hall wires after the controller has successfully completed self-learning.

## Three ultrasonic sensors

The firmware exposes three sensors as front, left and right.

### HC-SR04-style wiring

For each sensor:

```text
HC-SR04 VCC  -> regulated 5 V
HC-SR04 GND  -> ESP32 GND
HC-SR04 TRIG -> assigned ESP32 trigger GPIO
HC-SR04 ECHO -> voltage divider -> assigned ESP32 echo GPIO
```

A common HC-SR04 echo is approximately 5 V. ESP32 GPIO is not 5 V tolerant. Use this divider on every echo line:

```text
HC-SR04 ECHO ---- 10 kOhm ----+---- ESP32 ECHO GPIO
                              |
                            15 kOhm
                              |
                             GND
```

That reduces a 5 V echo to about 3.0 V.

Sensor assignments:

| Position | TRIG | ECHO |
|---|---:|---:|
| Front | GPIO16 | GPIO34 |
| Left | GPIO17 | GPIO35 |
| Right | GPIO18 | GPIO39 |

The firmware pings one sensor at a time to reduce ultrasonic crosstalk and reports each distance in millimeters over the existing Pi/ESP32 binary serial connection.

## Donkeycar signals

The Raspberry Pi adapter exposes the three ranges directly in Donkeycar memory:

```text
ultrasonic/front_m
ultrasonic/left_m
ultrasonic/right_m
```

The same values are included in `/api/state` on the robot dashboard and in the JSON run log. A value of `None` means the ESP32 has no valid recent reading.

## First powered test

1. Put both drive wheels off the ground and physically restrain the chassis.
2. Keep the controller power-lock / emergency-stop open while checking all low-voltage wiring.
3. Power the ESP32 and Pi first. Confirm the ESP32 is connected over USB serial.
4. Power the motor controllers.
5. Perform controller self-learning one wheel at a time.
6. Verify zero Donkeycar throttle produces zero wheel movement.
7. Verify a small positive command moves both wheels in robot-forward direction.
8. Verify zero command applies the low-level brake.
9. Only if the actual harness has a labeled reverse connector, test a small negative command and confirm both wheels reverse.
10. Verify the dashboard reports sensible front, left and right ultrasonic distances before lowering the robot to the ground.

Do not test full throttle until direction, braking, serial lease timeout and ultrasonic readings have all been verified with the wheels lifted.
