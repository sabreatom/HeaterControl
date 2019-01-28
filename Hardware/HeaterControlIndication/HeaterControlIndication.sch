EESchema Schematic File Version 4
LIBS:HeaterControlIndication-cache
EELAYER 26 0
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
L MCU_ST_STM32F0:STM32F030K6Tx U2
U 1 1 5C4BB1EE
P 7575 3500
F 0 "U2" H 7700 2525 50  0000 C CNN
F 1 "STM32F030K6Tx" H 7950 2450 50  0000 C CNN
F 2 "Package_QFP:LQFP-32_7x7mm_P0.8mm" H 7075 2600 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 7575 3500 50  0001 C CNN
	1    7575 3500
	1    0    0    -1  
$EndComp
$Comp
L Relay:G5LE-1 K1
U 1 1 5C4BB50C
P 2175 3225
F 0 "K1" H 1650 3325 50  0000 L CNN
F 1 "G5LE-1" H 1450 3250 50  0000 L CNN
F 2 "Relay_THT:Relay_SPDT_Omron-G5LE-1" H 2625 3175 50  0001 L CNN
F 3 "http://www.omron.com/ecb/products/pdf/en-g5le.pdf" H 2175 2825 50  0001 C CNN
	1    2175 3225
	-1   0    0    -1  
$EndComp
$Comp
L Converter_ACDC:IRM-02-5 PS1
U 1 1 5C4BB5F7
P 2525 1475
F 0 "PS1" H 2525 1800 50  0000 C CNN
F 1 "IRM-02-5" H 2525 1709 50  0000 C CNN
F 2 "Converter_ACDC:Converter_ACDC_MeanWell_IRM-02-xx_THT" H 2525 1175 50  0001 C CNN
F 3 "http://www.meanwell.com/productPdf.aspx?i=675" H 2925 1125 50  0001 C CNN
	1    2525 1475
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J2
U 1 1 5C4BB745
P 875 1475
F 0 "J2" H 850 1250 50  0000 L CNN
F 1 "Screw_Terminal_01x03" H 100 1175 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-3_P5.08mm" H 875 1475 50  0001 C CNN
F 3 "~" H 875 1475 50  0001 C CNN
	1    875  1475
	-1   0    0    -1  
$EndComp
$Comp
L Sensor_Current:ACS712xLCTR-05B U3
U 1 1 5C4BB815
P 2125 5375
F 0 "U3" H 2500 5600 50  0000 C CNN
F 1 "ACS712xLCTR-05B" H 2800 5525 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2225 5025 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/ACS712-Datasheet.ashx?la=en" H 2125 5375 50  0001 C CNN
	1    2125 5375
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP601-xOT U4
U 1 1 5C4BBD35
P 4850 5475
F 0 "U4" H 4800 5300 50  0000 L CNN
F 1 "MCP601-xOT" H 4800 5225 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4750 5275 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 4850 5675 50  0001 C CNN
	1    4850 5475
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q1
U 1 1 5C4BBEB9
P 9750 1950
F 0 "Q1" H 9955 1996 50  0000 L CNN
F 1 "2N7002" H 9955 1905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9950 1875 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7002.pdf" H 9750 1950 50  0001 L CNN
	1    9750 1950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 5C4BC100
P 3500 4125
F 0 "Q2" H 3705 4171 50  0000 L CNN
F 1 "2N7002" H 3705 4080 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3700 4050 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7002.pdf" H 3500 4125 50  0001 L CNN
	1    3500 4125
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MCP1700-3302E_SOT89 U1
U 1 1 5C4BC8D0
P 4050 1375
F 0 "U1" H 4050 1617 50  0000 C CNN
F 1 "MCP1700-3302E_SOT89" H 4050 1526 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-89-3" H 4050 1575 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826C.pdf" H 4050 1325 50  0001 C CNN
	1    4050 1375
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x04 J1
U 1 1 5C4BCD26
P 10175 1400
F 0 "J1" H 10255 1392 50  0000 L CNN
F 1 "282834-4" H 10255 1301 50  0000 L CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-4_1x04_P2.54mm_Horizontal" H 10175 1400 50  0001 C CNN
F 3 "~" H 10175 1400 50  0001 C CNN
	1    10175 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse_Small F1
U 1 1 5C4C87FF
P 1450 1375
F 0 "F1" H 1450 1560 50  0000 C CNN
F 1 "FX0457" H 1450 1469 50  0000 C CNN
F 2 "Fuse:Fuseholder_Cylinder-5x20mm_Bulgin_FX0457_Horizontal_Closed" H 1450 1375 50  0001 C CNN
F 3 "~" H 1450 1375 50  0001 C CNN
	1    1450 1375
	1    0    0    -1  
$EndComp
Wire Wire Line
	1075 1375 1350 1375
Wire Wire Line
	1550 1375 1875 1375
Wire Wire Line
	1075 1475 2025 1475
Wire Wire Line
	2025 1475 2025 1575
Wire Wire Line
	2025 1575 2125 1575
$Comp
L power:GND #PWR05
U 1 1 5C4C8B1D
P 3000 1650
F 0 "#PWR05" H 3000 1400 50  0001 C CNN
F 1 "GND" H 3005 1477 50  0000 C CNN
F 2 "" H 3000 1650 50  0001 C CNN
F 3 "" H 3000 1650 50  0001 C CNN
	1    3000 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5C4C8B57
P 4050 1825
F 0 "#PWR07" H 4050 1575 50  0001 C CNN
F 1 "GND" H 4055 1652 50  0000 C CNN
F 2 "" H 4050 1825 50  0001 C CNN
F 3 "" H 4050 1825 50  0001 C CNN
	1    4050 1825
	1    0    0    -1  
$EndComp
Wire Wire Line
	2925 1575 3000 1575
Wire Wire Line
	3000 1575 3000 1650
Wire Wire Line
	4050 1675 4050 1775
$Comp
L Device:CP C1
U 1 1 5C4C8CB4
P 3325 1575
F 0 "C1" H 3443 1621 50  0000 L CNN
F 1 "10u" H 3443 1530 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3363 1425 50  0001 C CNN
F 3 "~" H 3325 1575 50  0001 C CNN
	1    3325 1575
	1    0    0    -1  
$EndComp
Wire Wire Line
	3325 1725 3325 1775
Wire Wire Line
	3325 1775 4050 1775
Connection ~ 4050 1775
Wire Wire Line
	4050 1775 4050 1825
Wire Wire Line
	2925 1375 3325 1375
Wire Wire Line
	3325 1425 3325 1375
Connection ~ 3325 1375
Wire Wire Line
	3325 1375 3750 1375
$Comp
L power:+5V #PWR02
U 1 1 5C4C8F0C
P 3325 1300
F 0 "#PWR02" H 3325 1150 50  0001 C CNN
F 1 "+5V" H 3340 1473 50  0000 C CNN
F 2 "" H 3325 1300 50  0001 C CNN
F 3 "" H 3325 1300 50  0001 C CNN
	1    3325 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3325 1300 3325 1375
$Comp
L Device:CP C2
U 1 1 5C4C92C3
P 4750 1575
F 0 "C2" H 4868 1621 50  0000 L CNN
F 1 "10u" H 4868 1530 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4788 1425 50  0001 C CNN
F 3 "~" H 4750 1575 50  0001 C CNN
	1    4750 1575
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 1375 4750 1375
Wire Wire Line
	4750 1375 4750 1425
Wire Wire Line
	4750 1725 4750 1775
Wire Wire Line
	4750 1775 4050 1775
$Comp
L power:+3.3V #PWR03
U 1 1 5C4CA81A
P 4750 1300
F 0 "#PWR03" H 4750 1150 50  0001 C CNN
F 1 "+3.3V" H 4765 1473 50  0000 C CNN
F 2 "" H 4750 1300 50  0001 C CNN
F 3 "" H 4750 1300 50  0001 C CNN
	1    4750 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1300 4750 1375
Connection ~ 4750 1375
Wire Wire Line
	1875 2925 1875 1375
Connection ~ 1875 1375
Wire Wire Line
	1875 1375 2125 1375
Wire Wire Line
	1975 3525 1975 3725
Wire Wire Line
	1975 3725 1550 3725
Wire Wire Line
	1550 3725 1550 5175
Wire Wire Line
	1550 5175 1725 5175
Wire Wire Line
	1725 5575 1400 5575
Wire Wire Line
	1400 5575 1400 2050
Wire Wire Line
	1400 2050 1750 2050
Wire Wire Line
	1750 2050 1750 1575
Wire Wire Line
	1750 1575 1075 1575
$Comp
L power:GND #PWR014
U 1 1 5C4CD140
P 3400 4550
F 0 "#PWR014" H 3400 4300 50  0001 C CNN
F 1 "GND" H 3405 4377 50  0000 C CNN
F 2 "" H 3400 4550 50  0001 C CNN
F 3 "" H 3400 4550 50  0001 C CNN
	1    3400 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5C4CD1A8
P 3775 4300
F 0 "R4" H 3834 4346 50  0000 L CNN
F 1 "100k" H 3834 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3775 4300 50  0001 C CNN
F 3 "~" H 3775 4300 50  0001 C CNN
	1    3775 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4550 3400 4475
Wire Wire Line
	3775 4200 3775 4125
Wire Wire Line
	3775 4125 3700 4125
Wire Wire Line
	3775 4400 3775 4475
Wire Wire Line
	3775 4475 3400 4475
Connection ~ 3400 4475
Wire Wire Line
	3400 4475 3400 4325
Wire Wire Line
	2375 3525 2375 3725
Wire Wire Line
	2375 3725 3100 3725
Wire Wire Line
	3400 3725 3400 3925
$Comp
L power:+5V #PWR012
U 1 1 5C4CF771
P 2375 2675
F 0 "#PWR012" H 2375 2525 50  0001 C CNN
F 1 "+5V" H 2390 2848 50  0000 C CNN
F 2 "" H 2375 2675 50  0001 C CNN
F 3 "" H 2375 2675 50  0001 C CNN
	1    2375 2675
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAR42FILM D1
U 1 1 5C4CFA6F
P 3100 3250
F 0 "D1" V 3054 3391 50  0000 L CNN
F 1 "BAR42FILM" V 3145 3391 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3100 3075 50  0001 C CNN
F 3 "www.st.com/resource/en/datasheet/bar42.pdf" H 3100 3250 50  0001 C CNN
	1    3100 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	2375 2675 2375 2800
Wire Wire Line
	3100 3400 3100 3725
Connection ~ 3100 3725
Wire Wire Line
	3100 3725 3400 3725
Wire Wire Line
	3100 3100 3100 2800
Wire Wire Line
	3100 2800 2375 2800
Connection ~ 2375 2800
Wire Wire Line
	2375 2800 2375 2925
$Comp
L power:GND #PWR024
U 1 1 5C4D217A
P 2125 6050
F 0 "#PWR024" H 2125 5800 50  0001 C CNN
F 1 "GND" H 2130 5877 50  0000 C CNN
F 2 "" H 2125 6050 50  0001 C CNN
F 3 "" H 2125 6050 50  0001 C CNN
	1    2125 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2125 5775 2125 5925
NoConn ~ 2075 2925
$Comp
L Device:C_Small C9
U 1 1 5C4D51DC
P 2650 5700
F 0 "C9" H 2742 5746 50  0000 L CNN
F 1 "1n" H 2742 5655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2650 5700 50  0001 C CNN
F 3 "~" H 2650 5700 50  0001 C CNN
	1    2650 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 5475 2650 5475
Wire Wire Line
	2650 5475 2650 5600
Wire Wire Line
	2650 5800 2650 5925
Wire Wire Line
	2650 5925 2125 5925
Connection ~ 2125 5925
Wire Wire Line
	2125 5925 2125 6050
$Comp
L Device:C_Small C6
U 1 1 5C4D68F2
P 2325 4775
F 0 "C6" V 2096 4775 50  0000 C CNN
F 1 "100n" V 2187 4775 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2325 4775 50  0001 C CNN
F 3 "~" H 2325 4775 50  0001 C CNN
	1    2325 4775
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5C4D76B5
P 2550 4825
F 0 "#PWR017" H 2550 4575 50  0001 C CNN
F 1 "GND" H 2555 4652 50  0000 C CNN
F 2 "" H 2550 4825 50  0001 C CNN
F 3 "" H 2550 4825 50  0001 C CNN
	1    2550 4825
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 5C4D77CE
P 2125 4600
F 0 "#PWR015" H 2125 4450 50  0001 C CNN
F 1 "+5V" H 2140 4773 50  0000 C CNN
F 2 "" H 2125 4600 50  0001 C CNN
F 3 "" H 2125 4600 50  0001 C CNN
	1    2125 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2125 4600 2125 4775
Wire Wire Line
	2225 4775 2125 4775
Connection ~ 2125 4775
Wire Wire Line
	2125 4775 2125 4975
Wire Wire Line
	2425 4775 2550 4775
Wire Wire Line
	2550 4775 2550 4825
$Comp
L Device:R_Small R6
U 1 1 5C4DA589
P 3150 5700
F 0 "R6" H 3209 5746 50  0000 L CNN
F 1 "26k7" H 3209 5655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3150 5700 50  0001 C CNN
F 3 "~" H 3150 5700 50  0001 C CNN
	1    3150 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5C4DA62C
P 2925 5375
F 0 "R5" V 2875 5200 50  0000 C CNN
F 1 "13k7" V 2875 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2925 5375 50  0001 C CNN
F 3 "~" H 2925 5375 50  0001 C CNN
	1    2925 5375
	0    1    1    0   
$EndComp
Wire Wire Line
	3150 5800 3150 5925
Wire Wire Line
	3150 5925 2650 5925
Connection ~ 2650 5925
Wire Wire Line
	2525 5375 2825 5375
Wire Wire Line
	3025 5375 3150 5375
Wire Wire Line
	3150 5375 3150 5600
$Comp
L Device:C_Small C10
U 1 1 5C4DF82A
P 3500 5700
F 0 "C10" H 3592 5746 50  0000 L CNN
F 1 "1n" H 3592 5655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3500 5700 50  0001 C CNN
F 3 "~" H 3500 5700 50  0001 C CNN
	1    3500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5800 3500 5925
Wire Wire Line
	3500 5925 3150 5925
Connection ~ 3150 5925
Wire Wire Line
	3500 5600 3500 5375
Wire Wire Line
	3500 5375 3150 5375
Connection ~ 3150 5375
$Comp
L Device:C_Small C7
U 1 1 5C4E1E16
P 4950 5000
F 0 "C7" V 4721 5000 50  0000 C CNN
F 1 "100n" V 4812 5000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4950 5000 50  0001 C CNN
F 3 "~" H 4950 5000 50  0001 C CNN
	1    4950 5000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5C4E1EB4
P 4750 5875
F 0 "#PWR023" H 4750 5625 50  0001 C CNN
F 1 "GND" H 4755 5702 50  0000 C CNN
F 2 "" H 4750 5875 50  0001 C CNN
F 3 "" H 4750 5875 50  0001 C CNN
	1    4750 5875
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5C4E1F01
P 5125 5050
F 0 "#PWR020" H 5125 4800 50  0001 C CNN
F 1 "GND" H 5130 4877 50  0000 C CNN
F 2 "" H 5125 5050 50  0001 C CNN
F 3 "" H 5125 5050 50  0001 C CNN
	1    5125 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4850 4750 5000
Wire Wire Line
	4850 5000 4750 5000
Connection ~ 4750 5000
Wire Wire Line
	4750 5000 4750 5175
Wire Wire Line
	5050 5000 5125 5000
Wire Wire Line
	5125 5000 5125 5050
Wire Wire Line
	4750 5875 4750 5775
Wire Wire Line
	4550 5375 3500 5375
Connection ~ 3500 5375
Wire Wire Line
	5150 5475 5500 5475
Wire Wire Line
	5500 5475 5500 6300
Wire Wire Line
	5500 6300 4350 6300
Wire Wire Line
	4350 6300 4350 5575
Wire Wire Line
	4350 5575 4550 5575
$Comp
L power:GND #PWR016
U 1 1 5C4EAF8A
P 7475 4650
F 0 "#PWR016" H 7475 4400 50  0001 C CNN
F 1 "GND" H 7480 4477 50  0000 C CNN
F 2 "" H 7475 4650 50  0001 C CNN
F 3 "" H 7475 4650 50  0001 C CNN
	1    7475 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7475 4500 7475 4575
Wire Wire Line
	7575 4500 7575 4575
Wire Wire Line
	7575 4575 7475 4575
Connection ~ 7475 4575
Wire Wire Line
	7475 4575 7475 4650
$Comp
L Connector_Generic:Conn_01x06 J4
U 1 1 5C4EFBAE
P 9850 5725
F 0 "J4" H 9930 5717 50  0000 L CNN
F 1 "SWD" H 9930 5626 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9850 5725 50  0001 C CNN
F 3 "~" H 9850 5725 50  0001 C CNN
	1    9850 5725
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR06
U 1 1 5C4EFF67
P 7675 1675
F 0 "#PWR06" H 7675 1525 50  0001 C CNN
F 1 "+3.3V" H 7690 1848 50  0000 C CNN
F 2 "" H 7675 1675 50  0001 C CNN
F 3 "" H 7675 1675 50  0001 C CNN
	1    7675 1675
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5C4F0274
P 7850 2275
F 0 "C5" V 7621 2275 50  0000 C CNN
F 1 "100n" V 7712 2275 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7850 2275 50  0001 C CNN
F 3 "~" H 7850 2275 50  0001 C CNN
	1    7850 2275
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5C4F0491
P 8000 2350
F 0 "#PWR09" H 8000 2100 50  0001 C CNN
F 1 "GND" H 8005 2177 50  0000 C CNN
F 2 "" H 8000 2350 50  0001 C CNN
F 3 "" H 8000 2350 50  0001 C CNN
	1    8000 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5C4F05E9
P 7850 1925
F 0 "C3" V 7621 1925 50  0000 C CNN
F 1 "100n" V 7712 1925 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7850 1925 50  0001 C CNN
F 3 "~" H 7850 1925 50  0001 C CNN
	1    7850 1925
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5C4F06CB
P 7275 2275
F 0 "C4" V 7046 2275 50  0000 C CNN
F 1 "100n" V 7137 2275 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7275 2275 50  0001 C CNN
F 3 "~" H 7275 2275 50  0001 C CNN
	1    7275 2275
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5C4F076A
P 7100 2350
F 0 "#PWR08" H 7100 2100 50  0001 C CNN
F 1 "GND" H 7105 2177 50  0000 C CNN
F 2 "" H 7100 2350 50  0001 C CNN
F 3 "" H 7100 2350 50  0001 C CNN
	1    7100 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7675 1675 7675 1925
Wire Wire Line
	7750 2275 7675 2275
Connection ~ 7675 2275
Wire Wire Line
	7675 2275 7675 2600
Wire Wire Line
	7750 1925 7675 1925
Connection ~ 7675 1925
Wire Wire Line
	7675 1925 7675 2275
Wire Wire Line
	7950 1925 8000 1925
Wire Wire Line
	8000 1925 8000 2275
Wire Wire Line
	7950 2275 8000 2275
Connection ~ 8000 2275
Wire Wire Line
	8000 2275 8000 2350
Wire Wire Line
	7175 2275 7100 2275
Wire Wire Line
	7100 2275 7100 2350
Wire Wire Line
	7375 2275 7475 2275
Wire Wire Line
	7475 2600 7475 2275
Connection ~ 7475 2275
Wire Wire Line
	7475 2275 7575 2275
Wire Wire Line
	7575 2600 7575 2275
Connection ~ 7575 2275
Wire Wire Line
	7575 2275 7675 2275
Text Label 5575 5475 0    50   ~ 0
CURR_ADC
Wire Wire Line
	8075 2900 8475 2900
Text Label 8075 2900 0    50   ~ 0
CURR_ADC
Wire Wire Line
	5500 5475 6025 5475
Connection ~ 5500 5475
Wire Wire Line
	6975 3400 4425 3400
Wire Wire Line
	4425 3400 4425 4125
Wire Wire Line
	4425 4125 3775 4125
Connection ~ 3775 4125
$Comp
L power:GND #PWR011
U 1 1 5C50BD35
P 9850 2450
F 0 "#PWR011" H 9850 2200 50  0001 C CNN
F 1 "GND" H 9855 2277 50  0000 C CNN
F 2 "" H 9850 2450 50  0001 C CNN
F 3 "" H 9850 2450 50  0001 C CNN
	1    9850 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5C50C10F
P 9500 2175
F 0 "R2" H 9559 2221 50  0000 L CNN
F 1 "100k" H 9559 2130 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9500 2175 50  0001 C CNN
F 3 "~" H 9500 2175 50  0001 C CNN
	1    9500 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1750 9850 1600
Wire Wire Line
	9850 1600 9975 1600
Wire Wire Line
	9500 2075 9500 1950
Wire Wire Line
	9500 1950 9550 1950
Wire Wire Line
	9850 2150 9850 2375
Wire Wire Line
	9500 2275 9500 2375
Wire Wire Line
	9500 2375 9850 2375
Connection ~ 9850 2375
Wire Wire Line
	9850 2375 9850 2450
$Comp
L power:+5V #PWR04
U 1 1 5C51BA92
P 9150 1450
F 0 "#PWR04" H 9150 1300 50  0001 C CNN
F 1 "+5V" H 9165 1623 50  0000 C CNN
F 2 "" H 9150 1450 50  0001 C CNN
F 3 "" H 9150 1450 50  0001 C CNN
	1    9150 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5C51BE5F
P 9575 1500
F 0 "R1" V 9525 1325 50  0000 C CNN
F 1 "600R" V 9525 1675 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9575 1500 50  0001 C CNN
F 3 "~" H 9575 1500 50  0001 C CNN
	1    9575 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	9150 1450 9150 1500
Wire Wire Line
	9150 1500 9475 1500
Wire Wire Line
	9675 1500 9975 1500
$Comp
L power:GND #PWR01
U 1 1 5C521BFF
P 10025 925
F 0 "#PWR01" H 10025 675 50  0001 C CNN
F 1 "GND" H 10030 752 50  0000 C CNN
F 2 "" H 10025 925 50  0001 C CNN
F 3 "" H 10025 925 50  0001 C CNN
	1    10025 925 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9975 1300 9875 1300
Wire Wire Line
	9875 1300 9875 825 
Wire Wire Line
	9875 825  10025 825 
Wire Wire Line
	10025 825  10025 925 
Wire Wire Line
	9975 1400 9450 1400
Text Label 9500 1400 0    50   ~ 0
PANEL_BUT
Wire Wire Line
	9500 1950 8975 1950
Connection ~ 9500 1950
Text Label 9025 1950 0    50   ~ 0
PANEL_LED
Text Label 6525 3900 0    50   ~ 0
PANEL_LED
Text Label 8125 4300 0    50   ~ 0
PANEL_BUT
$Comp
L Device:C_Small C8
U 1 1 5C53684C
P 9750 5100
F 0 "C8" V 9521 5100 50  0000 C CNN
F 1 "100n" V 9612 5100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9750 5100 50  0001 C CNN
F 3 "~" H 9750 5100 50  0001 C CNN
	1    9750 5100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5C536B26
P 9925 5150
F 0 "#PWR021" H 9925 4900 50  0001 C CNN
F 1 "GND" H 9930 4977 50  0000 C CNN
F 2 "" H 9925 5150 50  0001 C CNN
F 3 "" H 9925 5150 50  0001 C CNN
	1    9925 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 5100 9925 5100
Wire Wire Line
	9925 5100 9925 5150
$Comp
L power:+3.3V #PWR019
U 1 1 5C53A23B
P 9550 4850
F 0 "#PWR019" H 9550 4700 50  0001 C CNN
F 1 "+3.3V" H 9565 5023 50  0000 C CNN
F 2 "" H 9550 4850 50  0001 C CNN
F 3 "" H 9550 4850 50  0001 C CNN
	1    9550 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 4850 9550 5100
Wire Wire Line
	9550 5525 9650 5525
Wire Wire Line
	9650 5100 9550 5100
Connection ~ 9550 5100
Wire Wire Line
	9550 5100 9550 5525
Wire Wire Line
	9650 5625 9000 5625
Wire Wire Line
	9650 5825 9000 5825
$Comp
L power:GND #PWR022
U 1 1 5C548527
P 8600 5800
F 0 "#PWR022" H 8600 5550 50  0001 C CNN
F 1 "GND" H 8605 5627 50  0000 C CNN
F 2 "" H 8600 5800 50  0001 C CNN
F 3 "" H 8600 5800 50  0001 C CNN
	1    8600 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 5725 8600 5725
Wire Wire Line
	8600 5725 8600 5800
Text Label 9050 5625 0    50   ~ 0
SWCLK
Text Label 9050 5825 0    50   ~ 0
SWDIO
Wire Wire Line
	9650 5925 9000 5925
Text Label 9050 5925 0    50   ~ 0
nRST
$Comp
L Device:R_Small R3
U 1 1 5C550115
P 6625 2600
F 0 "R3" H 6684 2646 50  0000 L CNN
F 1 "100k" H 6684 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6625 2600 50  0001 C CNN
F 3 "~" H 6625 2600 50  0001 C CNN
	1    6625 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5C550216
P 6800 3075
F 0 "#PWR013" H 6800 2825 50  0001 C CNN
F 1 "GND" H 6805 2902 50  0000 C CNN
F 2 "" H 6800 3075 50  0001 C CNN
F 3 "" H 6800 3075 50  0001 C CNN
	1    6800 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	6975 3000 6800 3000
Wire Wire Line
	6800 3000 6800 3075
$Comp
L power:+3.3V #PWR010
U 1 1 5C5542CD
P 6625 2375
F 0 "#PWR010" H 6625 2225 50  0001 C CNN
F 1 "+3.3V" H 6640 2548 50  0000 C CNN
F 2 "" H 6625 2375 50  0001 C CNN
F 3 "" H 6625 2375 50  0001 C CNN
	1    6625 2375
	1    0    0    -1  
$EndComp
Wire Wire Line
	6625 2500 6625 2375
Wire Wire Line
	6625 2700 6625 2800
Wire Wire Line
	6625 2800 6975 2800
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5C55C5B1
P 5950 2800
F 0 "J3" H 5870 3017 50  0000 C CNN
F 1 "RST_JP" H 5870 2926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5950 2800 50  0001 C CNN
F 3 "~" H 5950 2800 50  0001 C CNN
	1    5950 2800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6150 2800 6625 2800
Connection ~ 6625 2800
Wire Wire Line
	6150 2900 6800 2900
Wire Wire Line
	6800 2900 6800 3000
Connection ~ 6800 3000
Text Label 6250 2800 0    50   ~ 0
nRST
Text Label 8225 4100 0    50   ~ 0
SWDIO
Text Label 8225 4200 0    50   ~ 0
SWCLK
Wire Wire Line
	8075 4100 8500 4100
Wire Wire Line
	8075 4200 8500 4200
NoConn ~ 9650 6025
$Comp
L Mechanical:MountingHole MH1
U 1 1 5C577989
P 5325 6875
F 0 "MH1" H 5425 6921 50  0000 L CNN
F 1 "MountingHole" H 5425 6830 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 5325 6875 50  0001 C CNN
F 3 "~" H 5325 6875 50  0001 C CNN
	1    5325 6875
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole MH3
U 1 1 5C577BC8
P 5325 7175
F 0 "MH3" H 5425 7221 50  0000 L CNN
F 1 "MountingHole" H 5425 7130 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 5325 7175 50  0001 C CNN
F 3 "~" H 5325 7175 50  0001 C CNN
	1    5325 7175
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole MH2
U 1 1 5C577C60
P 6125 6875
F 0 "MH2" H 6225 6921 50  0000 L CNN
F 1 "MountingHole" H 6225 6830 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 6125 6875 50  0001 C CNN
F 3 "~" H 6125 6875 50  0001 C CNN
	1    6125 6875
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole MH4
U 1 1 5C577CFB
P 6125 7175
F 0 "MH4" H 6225 7221 50  0000 L CNN
F 1 "MountingHole" H 6225 7130 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 6125 7175 50  0001 C CNN
F 3 "~" H 6125 7175 50  0001 C CNN
	1    6125 7175
	1    0    0    -1  
$EndComp
NoConn ~ 6975 4000
NoConn ~ 6975 4100
NoConn ~ 6975 4200
NoConn ~ 6975 4300
NoConn ~ 6975 3500
NoConn ~ 8075 4000
NoConn ~ 8075 3900
NoConn ~ 8075 3800
NoConn ~ 8075 3700
NoConn ~ 8075 3600
NoConn ~ 8075 3500
NoConn ~ 8075 3400
NoConn ~ 8075 3300
NoConn ~ 8075 3200
NoConn ~ 8075 3100
NoConn ~ 8075 3000
NoConn ~ 8075 2800
Text Label 1150 1475 0    50   ~ 0
HV_N
Text Label 1150 1575 0    50   ~ 0
HV_LOAD
Text Label 1150 1375 0    50   ~ 0
HV_L
Text Label 1575 1375 0    50   ~ 0
HV_L_F
Text Label 1600 3725 0    50   ~ 0
HV_ROUT
$Comp
L power:+3.3V #PWR0101
U 1 1 5C5D490C
P 4750 4850
F 0 "#PWR0101" H 4750 4700 50  0001 C CNN
F 1 "+3.3V" H 4765 5023 50  0000 C CNN
F 2 "" H 4750 4850 50  0001 C CNN
F 3 "" H 4750 4850 50  0001 C CNN
	1    4750 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8075 4300 8550 4300
Wire Wire Line
	6975 3900 6500 3900
NoConn ~ 6975 3800
NoConn ~ 6975 3700
$EndSCHEMATC
