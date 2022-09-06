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
L Sensor_Magnetic:A1302ELHLT-T U1
U 1 1 61D9CBE1
P 4200 3700
F 0 "U1" H 3970 3746 50  0000 R CNN
F 1 "A1302ELHLT-T" H 3970 3655 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 4200 3350 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1301-2-Datasheet.ashx" H 4100 3700 50  0001 C CNN
	1    4200 3700
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Magnetic:A1302ELHLT-T U2
U 1 1 61D9D574
P 5800 3700
F 0 "U2" H 5570 3746 50  0000 R CNN
F 1 "A1302ELHLT-T" H 5570 3655 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23W" H 5800 3350 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A1301-2-Datasheet.ashx" H 5700 3700 50  0001 C CNN
	1    5800 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 61DA9D41
P 4100 4200
F 0 "#PWR0101" H 4100 3950 50  0001 C CNN
F 1 "GND" H 4105 4027 50  0000 C CNN
F 2 "" H 4100 4200 50  0001 C CNN
F 3 "" H 4100 4200 50  0001 C CNN
	1    4100 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61DAA12F
P 5700 4200
F 0 "#PWR0102" H 5700 3950 50  0001 C CNN
F 1 "GND" H 5705 4027 50  0000 C CNN
F 2 "" H 5700 4200 50  0001 C CNN
F 3 "" H 5700 4200 50  0001 C CNN
	1    5700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4100 4100 4150
Wire Wire Line
	5700 4100 5700 4150
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 61DB80A0
P 5000 2600
F 0 "H1" H 5100 2649 50  0000 L CNN
F 1 "MountingHole_Pad" H 5100 2558 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_5x10mm" H 5000 2600 50  0001 C CNN
F 3 "~" H 5000 2600 50  0001 C CNN
	1    5000 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3300 5000 3300
Wire Wire Line
	5000 3300 5000 2700
Wire Wire Line
	5700 3300 5700 2700
Wire Wire Line
	5700 2700 5000 2700
Connection ~ 5000 2700
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 61DBB087
P 6400 3600
F 0 "H4" H 6500 3649 50  0000 L CNN
F 1 "MountingHole_Pad" H 6500 3558 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_5x10mm" H 6400 3600 50  0001 C CNN
F 3 "~" H 6400 3600 50  0001 C CNN
	1    6400 3600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 61DBB8E4
P 4700 3600
F 0 "H2" H 4800 3649 50  0000 L CNN
F 1 "MountingHole_Pad" H 4800 3558 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_5x10mm" H 4700 3600 50  0001 C CNN
F 3 "~" H 4700 3600 50  0001 C CNN
	1    4700 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3700 4500 3700
Wire Wire Line
	6400 3700 6100 3700
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 61DBBF93
P 4750 4050
F 0 "H3" H 4850 4099 50  0000 L CNN
F 1 "MountingHole_Pad" H 4850 4008 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_5x10mm" H 4750 4050 50  0001 C CNN
F 3 "~" H 4750 4050 50  0001 C CNN
	1    4750 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4150 4100 4150
Connection ~ 4100 4150
Wire Wire Line
	4100 4150 4100 4200
Wire Wire Line
	4750 4150 5700 4150
Connection ~ 4750 4150
Connection ~ 5700 4150
Wire Wire Line
	5700 4150 5700 4200
$EndSCHEMATC
