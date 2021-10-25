EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:LED_Small_ALT D1
U 1 1 632C5E3A
P 4700 2950
F 0 "D1" V 4746 2880 50  0000 R CNN
F 1 "LED" V 4655 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 4700 2950 50  0001 C CNN
F 3 "~" V 4700 2950 50  0001 C CNN
	1    4700 2950
	0    -1   -1   0   
$EndComp
$Comp
L MCU_Microchip_ATtiny:ATtiny84A-MMH U1
U 1 1 632C737D
P 2750 3050
F 0 "U1" H 2207 3096 50  0000 R CNN
F 1 "ATtiny84A-MMH" H 2207 3005 50  0000 R CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_3x3mm_P0.45mm_EP1.6x1.6mm" H 2750 3050 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc8183.pdf" H 2750 3050 50  0001 C CNN
	1    2750 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small_ALT D2
U 1 1 632C8AE6
P 5050 2950
F 0 "D2" V 5096 2880 50  0000 R CNN
F 1 "LED" V 5005 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 5050 2950 50  0001 C CNN
F 3 "~" V 5050 2950 50  0001 C CNN
	1    5050 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D3
U 1 1 632C91BF
P 5350 2950
F 0 "D3" V 5396 2880 50  0000 R CNN
F 1 "LED" V 5305 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 5350 2950 50  0001 C CNN
F 3 "~" V 5350 2950 50  0001 C CNN
	1    5350 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D4
U 1 1 632C9726
P 5650 2950
F 0 "D4" V 5696 2880 50  0000 R CNN
F 1 "LED" V 5605 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 5650 2950 50  0001 C CNN
F 3 "~" V 5650 2950 50  0001 C CNN
	1    5650 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3350 2450 4700 2450
Wire Wire Line
	4700 2450 4700 2850
Wire Wire Line
	3350 2550 5050 2550
Wire Wire Line
	5050 2550 5050 2850
Wire Wire Line
	3350 2650 5350 2650
Wire Wire Line
	5350 2650 5350 2850
Wire Wire Line
	3350 2750 5650 2750
Wire Wire Line
	5650 2750 5650 2850
$Comp
L Device:LED_Small_ALT D5
U 1 1 632D355A
P 6100 2950
F 0 "D5" V 6146 2880 50  0000 R CNN
F 1 "LED" V 6055 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 6100 2950 50  0001 C CNN
F 3 "~" V 6100 2950 50  0001 C CNN
	1    6100 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D6
U 1 1 632D3612
P 6450 2950
F 0 "D6" V 6496 2880 50  0000 R CNN
F 1 "LED" V 6405 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 6450 2950 50  0001 C CNN
F 3 "~" V 6450 2950 50  0001 C CNN
	1    6450 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D7
U 1 1 632D361C
P 6750 2950
F 0 "D7" V 6796 2880 50  0000 R CNN
F 1 "LED" V 6705 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 6750 2950 50  0001 C CNN
F 3 "~" V 6750 2950 50  0001 C CNN
	1    6750 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D8
U 1 1 632D3626
P 7050 2950
F 0 "D8" V 7096 2880 50  0000 R CNN
F 1 "LED" V 7005 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 7050 2950 50  0001 C CNN
F 3 "~" V 7050 2950 50  0001 C CNN
	1    7050 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4700 2450 6100 2450
Wire Wire Line
	6100 2450 6100 2850
Connection ~ 4700 2450
Wire Wire Line
	5050 2550 6450 2550
Wire Wire Line
	6450 2550 6450 2850
Connection ~ 5050 2550
Wire Wire Line
	5350 2650 6750 2650
Wire Wire Line
	6750 2650 6750 2850
Connection ~ 5350 2650
Wire Wire Line
	5650 2750 7050 2750
Wire Wire Line
	7050 2750 7050 2850
Connection ~ 5650 2750
$Comp
L Device:LED_Small_ALT D9
U 1 1 632D6DFA
P 7650 2950
F 0 "D9" V 7696 2880 50  0000 R CNN
F 1 "LED" V 7605 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 7650 2950 50  0001 C CNN
F 3 "~" V 7650 2950 50  0001 C CNN
	1    7650 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D10
U 1 1 632D6F12
P 8000 2950
F 0 "D10" V 8046 2880 50  0000 R CNN
F 1 "LED" V 7955 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 8000 2950 50  0001 C CNN
F 3 "~" V 8000 2950 50  0001 C CNN
	1    8000 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small_ALT D11
U 1 1 632D6F1C
P 8300 2950
F 0 "D11" V 8346 2880 50  0000 R CNN
F 1 "LED" V 8255 2880 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 8300 2950 50  0001 C CNN
F 3 "~" V 8300 2950 50  0001 C CNN
	1    8300 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 2450 7650 2450
Wire Wire Line
	7650 2450 7650 2850
Connection ~ 6100 2450
Wire Wire Line
	6450 2550 8000 2550
Wire Wire Line
	8000 2550 8000 2850
Connection ~ 6450 2550
Wire Wire Line
	6750 2650 8300 2650
Wire Wire Line
	8300 2650 8300 2850
Connection ~ 6750 2650
Wire Wire Line
	5650 3050 5650 3150
Wire Wire Line
	5650 3150 5350 3150
Wire Wire Line
	4700 3150 4700 3050
Wire Wire Line
	5050 3050 5050 3150
Connection ~ 5050 3150
Wire Wire Line
	5050 3150 4700 3150
Wire Wire Line
	5350 3050 5350 3150
Connection ~ 5350 3150
Wire Wire Line
	5350 3150 5050 3150
Wire Wire Line
	4700 3150 4450 3150
Wire Wire Line
	4450 3150 4450 2850
Wire Wire Line
	3350 2850 4450 2850
Connection ~ 4700 3150
Wire Wire Line
	6100 3050 6100 3250
Wire Wire Line
	6100 3250 6450 3250
Wire Wire Line
	6450 3250 6450 3050
Wire Wire Line
	6450 3250 6750 3250
Wire Wire Line
	6750 3250 6750 3050
Connection ~ 6450 3250
Wire Wire Line
	6750 3250 7050 3250
Wire Wire Line
	7050 3250 7050 3050
Connection ~ 6750 3250
Wire Wire Line
	6100 3250 4350 3250
Wire Wire Line
	4350 3250 4350 2950
Wire Wire Line
	3350 2950 4350 2950
Connection ~ 6100 3250
Wire Wire Line
	7650 3050 7650 3350
Wire Wire Line
	7650 3350 8000 3350
Wire Wire Line
	8000 3350 8000 3050
Wire Wire Line
	8000 3350 8300 3350
Wire Wire Line
	8300 3350 8300 3050
Connection ~ 8000 3350
Wire Wire Line
	7650 3350 4250 3350
Wire Wire Line
	4250 3350 4250 3050
Wire Wire Line
	3350 3050 4250 3050
Connection ~ 7650 3350
$Comp
L amulet:TTP223-BA6 U2
U 1 1 60C243DD
P 6000 4450
F 0 "U2" H 6000 4915 50  0000 C CNN
F 1 "TTP223-BA6" H 6000 4824 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 6000 4450 50  0001 C CNN
F 3 "" H 6000 4450 50  0001 C CNN
	1    6000 4450
	1    0    0    -1  
$EndComp
$Comp
L amulet:VEML6075 U3
U 1 1 60C249BB
P 5600 1500
F 0 "U3" H 5600 1825 50  0000 C CNN
F 1 "VEML6075" H 5600 1734 50  0000 C CNN
F 2 "amulet:VEML6075" H 5500 1500 50  0001 C CNN
F 3 "" H 5500 1500 50  0001 C CNN
	1    5600 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60C250B2
P 2750 4050
F 0 "#PWR0101" H 2750 3800 50  0001 C CNN
F 1 "GND" H 2755 3877 50  0000 C CNN
F 2 "" H 2750 4050 50  0001 C CNN
F 3 "" H 2750 4050 50  0001 C CNN
	1    2750 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 4050 2750 3950
$Comp
L power:GND #PWR0102
U 1 1 60C27051
P 5000 1400
F 0 "#PWR0102" H 5000 1150 50  0001 C CNN
F 1 "GND" V 5005 1272 50  0000 R CNN
F 2 "" H 5000 1400 50  0001 C CNN
F 3 "" H 5000 1400 50  0001 C CNN
	1    5000 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 1400 5250 1400
$Comp
L power:GND #PWR0103
U 1 1 60C28A28
P 5450 4400
F 0 "#PWR0103" H 5450 4150 50  0001 C CNN
F 1 "GND" V 5455 4272 50  0000 R CNN
F 2 "" H 5450 4400 50  0001 C CNN
F 3 "" H 5450 4400 50  0001 C CNN
	1    5450 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 4400 5650 4400
Wire Wire Line
	3350 3550 4700 3550
Wire Wire Line
	4700 3550 4700 4250
Wire Wire Line
	4700 4250 5650 4250
Wire Wire Line
	3350 3350 3800 3350
Wire Wire Line
	3350 3450 3800 3450
Text Label 3800 3350 2    50   ~ 0
SCL
Text Label 3800 3450 2    50   ~ 0
SDA
Text Label 4950 1600 0    50   ~ 0
SDA
Wire Wire Line
	4950 1600 5250 1600
Wire Wire Line
	5950 1600 6250 1600
Text Label 6250 1600 2    50   ~ 0
SCL
Wire Wire Line
	3350 3150 4050 3150
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60C32A1F
P 7200 1400
F 0 "#FLG0101" H 7200 1475 50  0001 C CNN
F 1 "PWR_FLAG" V 7200 1528 50  0000 L CNN
F 2 "" H 7200 1400 50  0001 C CNN
F 3 "~" H 7200 1400 50  0001 C CNN
	1    7200 1400
	0    1    1    0   
$EndComp
Text Label 5050 4250 0    50   ~ 0
BUTTON
Text Label 4050 3150 2    50   ~ 0
VEML6075_Power
Wire Wire Line
	5950 1400 6950 1400
Text Label 6700 1400 2    50   ~ 0
VEML6075_Power
$Comp
L Device:C_Small C1
U 1 1 60C380AD
P 3050 2000
F 0 "C1" V 2821 2000 50  0000 C CNN
F 1 "100n" V 2912 2000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3050 2000 50  0001 C CNN
F 3 "~" H 3050 2000 50  0001 C CNN
	1    3050 2000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 60C3C22B
P 3350 2000
F 0 "#PWR03" H 3350 1750 50  0001 C CNN
F 1 "GND" V 3355 1872 50  0000 R CNN
F 2 "" H 3350 2000 50  0001 C CNN
F 3 "" H 3350 2000 50  0001 C CNN
	1    3350 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3150 2000 3350 2000
Wire Wire Line
	2950 2000 2750 2000
Wire Wire Line
	2750 2000 2750 2150
$Comp
L Device:C_Small C2
U 1 1 60C3F27A
P 6950 1600
F 0 "C2" H 6858 1554 50  0000 R CNN
F 1 "100n" H 6858 1645 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6950 1600 50  0001 C CNN
F 3 "~" H 6950 1600 50  0001 C CNN
	1    6950 1600
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 60C42B9B
P 6950 1800
F 0 "#PWR01" H 6950 1550 50  0001 C CNN
F 1 "GND" H 6955 1627 50  0000 C CNN
F 2 "" H 6950 1800 50  0001 C CNN
F 3 "" H 6950 1800 50  0001 C CNN
	1    6950 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1800 6950 1700
Wire Wire Line
	6950 1500 6950 1400
Connection ~ 6950 1400
Wire Wire Line
	6950 1400 7200 1400
$Comp
L Device:C_Small C3
U 1 1 60C474BC
P 6950 4600
F 0 "C3" H 6858 4554 50  0000 R CNN
F 1 "100n" H 6858 4645 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6950 4600 50  0001 C CNN
F 3 "~" H 6950 4600 50  0001 C CNN
	1    6950 4600
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 60C476AE
P 6950 4800
F 0 "#PWR07" H 6950 4550 50  0001 C CNN
F 1 "GND" H 6955 4627 50  0000 C CNN
F 2 "" H 6950 4800 50  0001 C CNN
F 3 "" H 6950 4800 50  0001 C CNN
	1    6950 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 4800 6950 4700
NoConn ~ 6350 4250
Wire Wire Line
	6350 4550 6550 4550
Wire Wire Line
	6550 4550 6550 4400
Wire Wire Line
	6550 4400 6350 4400
$Comp
L power:VDD #PWR02
U 1 1 60C504E3
P 2750 1900
F 0 "#PWR02" H 2750 1750 50  0001 C CNN
F 1 "VDD" H 2765 2073 50  0000 C CNN
F 2 "" H 2750 1900 50  0001 C CNN
F 3 "" H 2750 1900 50  0001 C CNN
	1    2750 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1900 2750 2000
Connection ~ 2750 2000
$Comp
L power:VDD #PWR05
U 1 1 60C526B8
P 6950 4250
F 0 "#PWR05" H 6950 4100 50  0001 C CNN
F 1 "VDD" H 6965 4423 50  0000 C CNN
F 2 "" H 6950 4250 50  0001 C CNN
F 3 "" H 6950 4250 50  0001 C CNN
	1    6950 4250
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR04
U 1 1 60C5303B
P 8600 4200
F 0 "#PWR04" H 8600 4050 50  0001 C CNN
F 1 "VDD" V 8615 4327 50  0000 L CNN
F 2 "" H 8600 4200 50  0001 C CNN
F 3 "" H 8600 4200 50  0001 C CNN
	1    8600 4200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 60C532B2
P 8600 4600
F 0 "#PWR06" H 8600 4350 50  0001 C CNN
F 1 "GND" V 8605 4472 50  0000 R CNN
F 2 "" H 8600 4600 50  0001 C CNN
F 3 "" H 8600 4600 50  0001 C CNN
	1    8600 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 4400 6950 4400
Wire Wire Line
	6950 4400 6950 4250
Connection ~ 6550 4400
Wire Wire Line
	6950 4400 6950 4500
Connection ~ 6950 4400
$Comp
L Connector:TestPoint TP3
U 1 1 60C5A94A
P 5150 4950
F 0 "TP3" V 5345 5022 50  0000 C CNN
F 1 "CapSense" V 5254 5022 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 5350 4950 50  0001 C CNN
F 3 "~" H 5350 4950 50  0001 C CNN
	1    5150 4950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5150 4950 5450 4950
Wire Wire Line
	5450 4950 5450 4550
Wire Wire Line
	5450 4550 5650 4550
$Comp
L Connector:TestPoint TP1
U 1 1 60C5D33F
P 9000 4200
F 0 "TP1" V 8954 4388 50  0000 L CNN
F 1 "Case" V 9045 4388 50  0000 L CNN
F 2 "amulet:amulet" H 9200 4200 50  0001 C CNN
F 3 "~" H 9200 4200 50  0001 C CNN
	1    9000 4200
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 60C653A2
P 9000 4600
F 0 "TP2" V 8954 4788 50  0000 L CNN
F 1 "Tab" V 9045 4788 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9200 4600 50  0001 C CNN
F 3 "~" H 9200 4600 50  0001 C CNN
	1    9000 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	8600 4200 9000 4200
Wire Wire Line
	9000 4600 8600 4600
Text Label 5450 4550 2    50   ~ 0
CapSense
$Comp
L Device:C_Small C4
U 1 1 60CB1032
P 5450 5250
F 0 "C4" H 5358 5204 50  0000 R CNN
F 1 "30pF" H 5358 5295 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5450 5250 50  0001 C CNN
F 3 "~" H 5450 5250 50  0001 C CNN
	1    5450 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	5450 5150 5450 4950
Connection ~ 5450 4950
$Comp
L power:GND #PWR08
U 1 1 60CB33A8
P 5450 5450
F 0 "#PWR08" H 5450 5200 50  0001 C CNN
F 1 "GND" H 5455 5277 50  0000 C CNN
F 2 "" H 5450 5450 50  0001 C CNN
F 3 "" H 5450 5450 50  0001 C CNN
	1    5450 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 5450 5450 5350
$Comp
L Connector:TestPoint TP6
U 1 1 60DA43B2
P 2950 4900
F 0 "TP6" H 3000 4850 50  0000 R CNN
F 1 "VDD" V 2950 5100 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3150 4900 50  0001 C CNN
F 3 "~" H 3150 4900 50  0001 C CNN
	1    2950 4900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP7
U 1 1 60DA6958
P 3100 4900
F 0 "TP7" H 3150 4850 50  0000 R CNN
F 1 "SCK" V 3100 5100 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3300 4900 50  0001 C CNN
F 3 "~" H 3300 4900 50  0001 C CNN
	1    3100 4900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP8
U 1 1 60DA6F5D
P 3250 4900
F 0 "TP8" H 3300 4850 50  0000 R CNN
F 1 "MISO" V 3250 5100 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3450 4900 50  0001 C CNN
F 3 "~" H 3450 4900 50  0001 C CNN
	1    3250 4900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP9
U 1 1 60DA71A9
P 3400 4900
F 0 "TP9" H 3450 4850 50  0000 R CNN
F 1 "MOSI" V 3400 5100 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3600 4900 50  0001 C CNN
F 3 "~" H 3600 4900 50  0001 C CNN
	1    3400 4900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 60DA72CA
P 2650 4900
F 0 "TP4" H 2700 4850 50  0000 R CNN
F 1 "RST" V 2650 5100 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 2850 4900 50  0001 C CNN
F 3 "~" H 2850 4900 50  0001 C CNN
	1    2650 4900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP5
U 1 1 60DA7664
P 2800 4900
F 0 "TP5" H 2850 4850 50  0000 R CNN
F 1 "GND" V 2800 5100 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3000 4900 50  0001 C CNN
F 3 "~" H 3000 4900 50  0001 C CNN
	1    2800 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	2650 4900 2650 4400
Wire Wire Line
	2650 4400 3500 4400
Wire Wire Line
	3500 4400 3500 3650
Wire Wire Line
	3500 3650 3350 3650
$Comp
L power:GND #PWR0104
U 1 1 60DB1062
P 2800 4700
F 0 "#PWR0104" H 2800 4450 50  0001 C CNN
F 1 "GND" H 2805 4527 50  0000 C CNN
F 2 "" H 2800 4700 50  0001 C CNN
F 3 "" H 2800 4700 50  0001 C CNN
	1    2800 4700
	-1   0    0    1   
$EndComp
Wire Wire Line
	2800 4700 2800 4900
$Comp
L power:VDD #PWR0105
U 1 1 60DB38E6
P 2950 4700
F 0 "#PWR0105" H 2950 4550 50  0001 C CNN
F 1 "VDD" H 2965 4873 50  0000 C CNN
F 2 "" H 2950 4700 50  0001 C CNN
F 3 "" H 2950 4700 50  0001 C CNN
	1    2950 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4700 2950 4900
Text Label 3700 2850 2    50   ~ 0
PA4
Text Label 3700 2950 2    50   ~ 0
PA5
Text Label 3700 3050 2    50   ~ 0
PA6
Text Label 3050 4650 0    50   ~ 0
PA4
Wire Wire Line
	3050 4650 3100 4650
Wire Wire Line
	3100 4650 3100 4900
Text Label 3200 4700 0    50   ~ 0
PA5
Wire Wire Line
	3200 4700 3250 4700
Wire Wire Line
	3250 4700 3250 4900
Text Label 3350 4750 0    50   ~ 0
PA6
Wire Wire Line
	3350 4750 3400 4750
Wire Wire Line
	3400 4750 3400 4900
$EndSCHEMATC
