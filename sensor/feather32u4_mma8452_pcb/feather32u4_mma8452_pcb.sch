EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:MiscellaneousDevices
LIBS:nRF24L01+
LIBS:feather32u4_mma8452_pcb-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ADAFRUIT_FEATHER U2
U 1 1 5843530F
P 7150 2200
F 0 "U2" V 7350 1650 60  0000 C CNN
F 1 "ADAFRUIT_FEATHER" V 7350 2450 60  0000 C CNN
F 2 "AdafruitFeather:ADAFRUIT_FEATHER" H 7300 2000 60  0001 C CNN
F 3 "" H 7300 2000 60  0000 C CNN
	1    7150 2200
	1    0    0    -1  
$EndComp
$Comp
L ADAFRUIT_FEATHER U2
U 2 1 584353EE
P 7150 4400
F 0 "U2" V 7350 3850 60  0000 C CNN
F 1 "ADAFRUIT_FEATHER" V 7350 4650 60  0000 C CNN
F 2 "AdafruitFeather:ADAFRUIT_FEATHER" H 7300 4200 60  0001 C CNN
F 3 "" H 7300 4200 60  0000 C CNN
	2    7150 4400
	1    0    0    -1  
$EndComp
$Comp
L nRF24L01+ U1
U 1 1 5843542B
P 4600 4850
F 0 "U1" H 4600 4550 50  0000 C CNN
F 1 "nRF24L01+" H 4600 5150 50  0000 C CNN
F 2 "AdafruitFeather:Socket_nRF24L01" H 4600 4950 50  0001 C CNN
F 3 "DOCUMENTATION" H 4600 4800 50  0001 C CNN
	1    4600 4850
	1    0    0    -1  
$EndComp
$Comp
L LED_RABG D1
U 1 1 58435490
P 4350 2550
F 0 "D1" H 4425 2900 50  0000 C CNN
F 1 "LED_RBG" H 4375 2200 50  0000 C CNN
F 2 "LEDs:LED-RGB-5MM_Common_Cathode" H 4300 2500 50  0001 C CNN
F 3 "" H 4300 2500 50  0000 C CNN
	1    4350 2550
	-1   0    0    1   
$EndComp
$Comp
L SPST SW1
U 1 1 58435568
P 4550 1750
F 0 "SW1" H 4550 1850 50  0000 C CNN
F 1 "SPST" H 4550 1650 50  0000 C CNN
F 2 "Switches:DPST_TE_SSJ12R04" H 4550 1750 50  0001 C CNN
F 3 "" H 4550 1750 50  0000 C CNN
	1    4550 1750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P1
U 1 1 5843563B
P 4250 3400
F 0 "P1" H 4250 3750 50  0000 C CNN
F 1 "CONN_01X06" V 4350 3400 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_S06B-XH-A_06x2.50mm_Angled" H 4250 3400 50  0001 C CNN
F 3 "" H 4250 3400 50  0000 C CNN
	1    4250 3400
	-1   0    0    -1  
$EndComp
Text GLabel 4050 1750 0    60   Output ~ 0
GND
Text GLabel 6600 3950 0    60   Output ~ 0
GND
Text GLabel 6600 3750 0    60   Output ~ 0
3V3
Text GLabel 3850 4800 0    60   Input ~ 0
3V3
Text GLabel 3850 4650 0    60   Output ~ 0
GND
Text GLabel 4050 2550 0    60   Input ~ 0
3V3
$Comp
L R R1
U 1 1 58435CEB
P 4800 2350
F 0 "R1" V 4880 2350 50  0000 C CNN
F 1 "R" V 4800 2350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 4730 2350 50  0001 C CNN
F 3 "" H 4800 2350 50  0000 C CNN
	1    4800 2350
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 58435D7B
P 4800 2550
F 0 "R2" V 4880 2550 50  0000 C CNN
F 1 "R" V 4800 2550 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 4730 2550 50  0001 C CNN
F 3 "" H 4800 2550 50  0000 C CNN
	1    4800 2550
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 58435DA5
P 4800 2750
F 0 "R3" V 4880 2750 50  0000 C CNN
F 1 "R" V 4800 2750 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 4730 2750 50  0001 C CNN
F 3 "" H 4800 2750 50  0000 C CNN
	1    4800 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	6900 2250 5350 2250
Wire Wire Line
	5350 2250 5350 2750
Wire Wire Line
	5350 2750 4950 2750
Wire Wire Line
	6900 2150 5250 2150
Wire Wire Line
	5250 2150 5250 2550
Wire Wire Line
	5250 2550 4950 2550
Wire Wire Line
	6900 2050 5150 2050
Wire Wire Line
	5150 2050 5150 2350
Wire Wire Line
	5150 2350 4950 2350
Text GLabel 4450 3150 2    60   Input ~ 0
3V3
Wire Wire Line
	6900 3750 6600 3750
Wire Wire Line
	6900 3950 6600 3950
Wire Wire Line
	6900 2750 6000 2750
Wire Wire Line
	6900 2650 5900 2650
Wire Wire Line
	6900 4950 6000 4950
Wire Wire Line
	6900 5050 5900 5050
Text GLabel 4450 3650 2    60   Output ~ 0
GND
Wire Wire Line
	6900 4650 5350 4650
Wire Wire Line
	5600 4750 6900 4750
Wire Wire Line
	5600 4750 5600 4800
Wire Wire Line
	5600 4800 5350 4800
Wire Wire Line
	6900 4850 5700 4850
Wire Wire Line
	5700 4850 5700 4950
Wire Wire Line
	5700 4950 5350 4950
Text GLabel 6600 2550 0    60   BiDi ~ 0
CE
Text GLabel 6600 2400 0    60   BiDi ~ 0
CSN
Wire Wire Line
	6900 2550 6600 2550
Text GLabel 3850 5100 0    60   BiDi ~ 0
CSN
Text GLabel 3850 4950 0    60   BiDi ~ 0
CE
Wire Wire Line
	6900 2450 6750 2450
Wire Wire Line
	6750 2450 6750 2400
Wire Wire Line
	6750 2400 6600 2400
Wire Wire Line
	5050 1750 6900 1750
$Comp
L CONN_01X06 P2
U 1 1 58443C81
P 4250 4100
F 0 "P2" H 4250 4450 50  0000 C CNN
F 1 "CONN_01X06" V 4350 4100 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_S06B-XH-A_06x2.50mm_Angled" H 4250 4100 50  0001 C CNN
F 3 "" H 4250 4100 50  0000 C CNN
	1    4250 4100
	-1   0    0    -1  
$EndComp
Text GLabel 4450 3850 2    60   Input ~ 0
3V3
Text GLabel 4450 4350 2    60   Output ~ 0
GND
Wire Wire Line
	6000 2750 6000 3250
Wire Wire Line
	6000 3250 4450 3250
Wire Wire Line
	4750 3250 4750 3950
Wire Wire Line
	4750 3950 4450 3950
Connection ~ 4750 3250
Wire Wire Line
	5900 2650 5900 3350
Wire Wire Line
	5900 3350 4450 3350
Wire Wire Line
	4850 3350 4850 4050
Wire Wire Line
	4850 4050 4450 4050
Connection ~ 4850 3350
Wire Wire Line
	6000 4950 6000 3450
Wire Wire Line
	6000 3450 4450 3450
Wire Wire Line
	4950 3450 4950 4150
Wire Wire Line
	4950 4150 4450 4150
Connection ~ 4950 3450
Wire Wire Line
	5900 5050 5900 3550
Wire Wire Line
	5900 3550 4450 3550
Wire Wire Line
	5050 3550 5050 4250
Wire Wire Line
	5050 4250 4450 4250
Connection ~ 5050 3550
$EndSCHEMATC