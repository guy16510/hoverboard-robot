EESchema Schematic File Version 4
LIBS:power
LIBS:device
LIBS:Connector_Generic
LIBS:Transistor_FET
LIBS:Regulator_Switching
EELAYER 29 0
EELAYER END
$Descr A3 16535 11693
Sheet 1 1
Title "ESP32 Robot Carrier - Power, Sensors, Motor Control"
Date "2026-08-08"
Rev "0.1"
Comp "Hoverboard Robot"
Comment1 "36-42V input, 5.1V/5A Pi rail, ESP32 carrier"
Comment2 "Hall/ECHO level shifting, reverse/brake open-drain outputs"
Comment3 "Do not route motor phase current through this PCB"
Comment4 "Pre-fabrication ERC/DRC and footprint verification required"
$EndDescr
Text Notes 850 700 0    120  ~ 24
POWER INPUT / 5.1V BUCK
Text Notes 5550 700 0    120  ~ 24
ESP32 DEVKIT V1 30-PIN CARRIER
Text Notes 10500 700 0    120  ~ 24
ROBOT I/O CONNECTORS
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 1
P 1350 1350
F 0 "J1" H 1268 1567 50  0000 C CNN
F 1 "BATTERY_36_42V" H 1268 1476 50 0000 C CNN
	1    1350 1350
	-1 0 0 -1
$EndComp
Text Label 1700 1350 0 50 ~ 0
BAT+
Text Label 1700 1450 0 50 ~ 0
GND
Wire Wire Line
	1550 1350 2050 1350
Wire Wire Line
	1550 1450 2050 1450
$Comp
L Device:Fuse F1
U 1 1 2
P 2450 1350
F 0 "F1" V 2253 1350 50 0000 C CNN
F 1 "5A_INPUT_FUSE" V 2344 1350 50 0000 C CNN
	1    2450 1350
	0 1 1 0
$EndComp
Wire Wire Line
	2050 1350 2300 1350
Text Label 2800 1350 0 50 ~ 0
BAT_FUSED
Wire Wire Line
	2600 1350 3200 1350
$Comp
L Device:D_TVS D1
U 1 1 3
P 3000 1750
F 0 "D1" V 2954 1830 50 0000 L CNN
F 1 "60V_TVS" V 3045 1830 50 0000 L CNN
	1    3000 1750
	0 1 1 0
$EndComp
Wire Wire Line
	3000 1600 3000 1350
Text Label 3000 2050 0 50 ~ 0
GND
Wire Wire Line
	3000 1900 3000 2050
Text Notes 950 2400 0 60 ~ 12
TPS54560-class 60V / 5A buck section, target 5.1V output.
Text Notes 950 2500 0 60 ~ 12
Use TI reference topology, compact switch loop, 63V input capacitors,
Text Notes 950 2600 0 60 ~ 12
thermal vias under exposed pad, large BAT+/5V/GND copper pours.
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 4
P 1450 3250
F 0 "J2" H 1368 3467 50 0000 C CNN
F 1 "PI_5V_OUT" H 1368 3376 50 0000 C CNN
	1    1450 3250
	-1 0 0 -1
$EndComp
Text Label 1800 3250 0 50 ~ 0
+5V1
Text Label 1800 3350 0 50 ~ 0
GND
Wire Wire Line
	1650 3250 2150 3250
Wire Wire Line
	1650 3350 2150 3350
$Comp
L Connector_Generic:Conn_01x15 J13
U 1 1 5
P 6100 2100
F 0 "J13" H 6180 2142 50 0000 L CNN
F 1 "ESP32_LEFT_HEADER" H 6180 2051 50 0000 L CNN
	1    6100 2100
	1 0 0 -1
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J14
U 1 1 6
P 7800 2100
F 0 "J14" H 7880 2142 50 0000 L CNN
F 1 "ESP32_RIGHT_HEADER" H 7880 2051 50 0000 L CNN
	1    7800 2100
	-1 0 0 -1
$EndComp
Text Notes 5850 3900 0 70 ~ 14
ESP32 plugs into two female 1x15 sockets.
Text Notes 5850 4000 0 70 ~ 14
VIN <- +5V1, GND <- common GND, 3V3 exported to MPU6050 only.
Text Label 5600 4550 2 50 ~ 0
GPIO25_LEFT_THROTTLE
Text Label 5600 4700 2 50 ~ 0
GPIO26_RIGHT_THROTTLE
Text Label 5600 4850 2 50 ~ 0
GPIO27_LEFT_REVERSE
Text Label 5600 5000 2 50 ~ 0
GPIO14_RIGHT_REVERSE
Text Label 5600 5150 2 50 ~ 0
GPIO33_LEFT_BRAKE
Text Label 5600 5300 2 50 ~ 0
GPIO32_RIGHT_BRAKE
Text Label 5600 5450 2 50 ~ 0
GPIO19_R_HALL_A
Text Label 5600 5600 2 50 ~ 0
GPIO21_R_HALL_B
Text Label 5600 5750 2 50 ~ 0
GPIO22_R_HALL_C
Text Label 7600 4550 0 50 ~ 0
GPIO16_L_HALL_A
Text Label 7600 4700 0 50 ~ 0
GPIO17_L_HALL_B
Text Label 7600 4850 0 50 ~ 0
GPIO36_L_HALL_C
Text Label 7600 5000 0 50 ~ 0
GPIO5_FRONT_TRIG
Text Label 7600 5150 0 50 ~ 0
GPIO34_FRONT_ECHO
Text Label 7600 5300 0 50 ~ 0
GPIO15_LEFT_TRIG
Text Label 7600 5450 0 50 ~ 0
GPIO35_LEFT_ECHO
Text Label 7600 5600 0 50 ~ 0
GPIO18_RIGHT_TRIG
Text Label 7600 5750 0 50 ~ 0
GPIO39_RIGHT_ECHO
Text Label 7600 5900 0 50 ~ 0
GPIO4_MPU_SDA
Text Label 7600 6050 0 50 ~ 0
GPIO23_MPU_SCL
Text Label 7600 6200 0 50 ~ 0
GPIO13_SERVO
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 7
P 11300 1450
F 0 "J3" H 11380 1442 50 0000 L CNN
F 1 "LEFT_MOTOR_CTRL" H 11380 1351 50 0000 L CNN
	1    11300 1450
	1 0 0 -1
$EndComp
Text Label 10650 1350 2 50 ~ 0
LEFT_THROTTLE_OUT
Text Label 10650 1450 2 50 ~ 0
LEFT_REVERSE_OD
Text Label 10650 1550 2 50 ~ 0
LEFT_BRAKE_OD
Text Label 10650 1650 2 50 ~ 0
GND
Wire Wire Line
	11100 1350 10650 1350
Wire Wire Line
	11100 1450 10650 1450
Wire Wire Line
	11100 1550 10650 1550
Wire Wire Line
	11100 1650 10650 1650
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 8
P 14100 1450
F 0 "J4" H 14180 1442 50 0000 L CNN
F 1 "RIGHT_MOTOR_CTRL" H 14180 1351 50 0000 L CNN
	1    14100 1450
	1 0 0 -1
$EndComp
Text Label 13450 1350 2 50 ~ 0
RIGHT_THROTTLE_OUT
Text Label 13450 1450 2 50 ~ 0
RIGHT_REVERSE_OD
Text Label 13450 1550 2 50 ~ 0
RIGHT_BRAKE_OD
Text Label 13450 1650 2 50 ~ 0
GND
Wire Wire Line
	13900 1350 13450 1350
Wire Wire Line
	13900 1450 13450 1450
Wire Wire Line
	13900 1550 13450 1550
Wire Wire Line
	13900 1650 13450 1650
Text Notes 10600 2050 0 60 ~ 12
Motor controller +5V throttle reference is intentionally NOT connected.
Text Notes 10600 2150 0 60 ~ 12
Only throttle signal, reverse, brake, and controller ground land here.
$Comp
L Connector_Generic:Conn_01x05 J5
U 1 1 9
P 11300 3000
F 0 "J5" H 11380 3042 50 0000 L CNN
F 1 "RIGHT_HALL" H 11380 2951 50 0000 L CNN
	1    11300 3000
	1 0 0 -1
$EndComp
Text Label 10650 2800 2 50 ~ 0
+5V1
Text Label 10650 2900 2 50 ~ 0
R_HALL_A_RAW
Text Label 10650 3000 2 50 ~ 0
R_HALL_B_RAW
Text Label 10650 3100 2 50 ~ 0
R_HALL_C_RAW
Text Label 10650 3200 2 50 ~ 0
GND
Wire Wire Line
	11100 2800 10650 2800
Wire Wire Line
	11100 2900 10650 2900
Wire Wire Line
	11100 3000 10650 3000
Wire Wire Line
	11100 3100 10650 3100
Wire Wire Line
	11100 3200 10650 3200
$Comp
L Connector_Generic:Conn_01x05 J6
U 1 1 10
P 14100 3000
F 0 "J6" H 14180 3042 50 0000 L CNN
F 1 "LEFT_HALL" H 14180 2951 50 0000 L CNN
	1    14100 3000
	1 0 0 -1
$EndComp
Text Label 13450 2800 2 50 ~ 0
+5V1
Text Label 13450 2900 2 50 ~ 0
L_HALL_A_RAW
Text Label 13450 3000 2 50 ~ 0
L_HALL_B_RAW
Text Label 13450 3100 2 50 ~ 0
L_HALL_C_RAW
Text Label 13450 3200 2 50 ~ 0
GND
Wire Wire Line
	13900 2800 13450 2800
Wire Wire Line
	13900 2900 13450 2900
Wire Wire Line
	13900 3000 13450 3000
Wire Wire Line
	13900 3100 13450 3100
Wire Wire Line
	13900 3200 13450 3200
Text Notes 10550 3550 0 60 ~ 12
Each HALL_*_RAW input -> 10k series/top resistor -> ESP32 node,
Text Notes 10550 3650 0 60 ~ 12
with 18k from ESP32 node to GND. 5V input becomes ~3.21V.
$Comp
L Connector_Generic:Conn_01x04 J7
U 1 1 11
P 11300 4550
F 0 "J7" H 11380 4542 50 0000 L CNN
F 1 "FRONT_ULTRASONIC" H 11380 4451 50 0000 L CNN
	1    11300 4550
	1 0 0 -1
$EndComp
Text Label 10650 4450 2 50 ~ 0
+5V1
Text Label 10650 4550 2 50 ~ 0
GPIO5_FRONT_TRIG
Text Label 10650 4650 2 50 ~ 0
FRONT_ECHO_RAW
Text Label 10650 4750 2 50 ~ 0
GND
Wire Wire Line
	11100 4450 10650 4450
Wire Wire Line
	11100 4550 10650 4550
Wire Wire Line
	11100 4650 10650 4650
Wire Wire Line
	11100 4750 10650 4750
$Comp
L Connector_Generic:Conn_01x04 J8
U 1 1 12
P 14100 4550
F 0 "J8" H 14180 4542 50 0000 L CNN
F 1 "LEFT_ULTRASONIC" H 14180 4451 50 0000 L CNN
	1    14100 4550
	1 0 0 -1
$EndComp
Text Label 13450 4450 2 50 ~ 0
+5V1
Text Label 13450 4550 2 50 ~ 0
GPIO15_LEFT_TRIG
Text Label 13450 4650 2 50 ~ 0
LEFT_ECHO_RAW
Text Label 13450 4750 2 50 ~ 0
GND
Wire Wire Line
	13900 4450 13450 4450
Wire Wire Line
	13900 4550 13450 4550
Wire Wire Line
	13900 4650 13450 4650
Wire Wire Line
	13900 4750 13450 4750
$Comp
L Connector_Generic:Conn_01x04 J9
U 1 1 13
P 11300 5500
F 0 "J9" H 11380 5492 50 0000 L CNN
F 1 "RIGHT_ULTRASONIC" H 11380 5401 50 0000 L CNN
	1    11300 5500
	1 0 0 -1
$EndComp
Text Label 10650 5400 2 50 ~ 0
+5V1
Text Label 10650 5500 2 50 ~ 0
GPIO18_RIGHT_TRIG
Text Label 10650 5600 2 50 ~ 0
RIGHT_ECHO_RAW
Text Label 10650 5700 2 50 ~ 0
GND
Wire Wire Line
	11100 5400 10650 5400
Wire Wire Line
	11100 5500 10650 5500
Wire Wire Line
	11100 5600 10650 5600
Wire Wire Line
	11100 5700 10650 5700
Text Notes 10550 6050 0 60 ~ 12
Each ECHO_RAW input uses the same 10k/18k divider before GPIO34/35/39.
$Comp
L Connector_Generic:Conn_01x04 J10
U 1 1 14
P 14100 5500
F 0 "J10" H 14180 5492 50 0000 L CNN
F 1 "MPU6050" H 14180 5401 50 0000 L CNN
	1    14100 5500
	1 0 0 -1
$EndComp
Text Label 13450 5400 2 50 ~ 0
+3V3
Text Label 13450 5500 2 50 ~ 0
GPIO4_MPU_SDA
Text Label 13450 5600 2 50 ~ 0
GPIO23_MPU_SCL
Text Label 13450 5700 2 50 ~ 0
GND
Wire Wire Line
	13900 5400 13450 5400
Wire Wire Line
	13900 5500 13450 5500
Wire Wire Line
	13900 5600 13450 5600
Wire Wire Line
	13900 5700 13450 5700
$Comp
L Connector_Generic:Conn_01x03 J11
U 1 1 15
P 11300 6800
F 0 "J11" H 11380 6842 50 0000 L CNN
F 1 "SERVO" H 11380 6751 50 0000 L CNN
	1    11300 6800
	1 0 0 -1
$EndComp
Text Label 10650 6700 2 50 ~ 0
+5V1
Text Label 10650 6800 2 50 ~ 0
GPIO13_SERVO
Text Label 10650 6900 2 50 ~ 0
GND
Wire Wire Line
	11100 6700 10650 6700
Wire Wire Line
	11100 6800 10650 6800
Wire Wire Line
	11100 6900 10650 6900
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 16
P 14100 6800
F 0 "J12" H 14180 6842 50 0000 L CNN
F 1 "SPARE_POWER" H 14180 6751 50 0000 L CNN
	1    14100 6800
	1 0 0 -1
$EndComp
Text Label 13450 6700 2 50 ~ 0
+5V1
Text Label 13450 6800 2 50 ~ 0
+3V3
Text Label 13450 6900 2 50 ~ 0
GND
Wire Wire Line
	13900 6700 13450 6700
Wire Wire Line
	13900 6800 13450 6800
Wire Wire Line
	13900 6900 13450 6900
Text Notes 900 7300 0 120 ~ 24
OUTPUT CONDITIONING
Text Notes 900 7550 0 60 ~ 12
GPIO25 -> 1k -> LEFT_THROTTLE_OUT, optional 1uF to GND
Text Notes 900 7650 0 60 ~ 12
GPIO26 -> 1k -> RIGHT_THROTTLE_OUT, optional 1uF to GND
Text Notes 900 7850 0 60 ~ 12
GPIO27 -> 100R gate -> Q1 N-MOSFET, 100k gate pulldown -> LEFT_REVERSE_OD
Text Notes 900 7950 0 60 ~ 12
GPIO14 -> 100R gate -> Q2 N-MOSFET, 100k gate pulldown -> RIGHT_REVERSE_OD
Text Notes 900 8050 0 60 ~ 12
GPIO33 -> 100R gate -> Q3 N-MOSFET, 100k gate pulldown -> LEFT_BRAKE_OD
Text Notes 900 8150 0 60 ~ 12
GPIO32 -> 100R gate -> Q4 N-MOSFET, 100k gate pulldown -> RIGHT_BRAKE_OD
Text Notes 900 8450 0 60 ~ 12
Open-drain MOSFET drains connect only to controller reverse/brake inputs;
Text Notes 900 8550 0 60 ~ 12
sources connect to common GND. Default reset state is OFF.
Text Notes 900 9000 0 90 ~ 18
IMPORTANT: battery voltage appears only in protected power section.
Text Notes 900 9150 0 90 ~ 18
Never connect BAT+ to ESP32, Pi, sensors, throttle, Hall, reverse, or brake pins.
$EndSCHEMATC
