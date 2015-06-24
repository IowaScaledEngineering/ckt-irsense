v 20130925 2
C 5350 3650 1 0 0 relay-SPDT-1.sym
C 5050 3750 1 90 0 diode-1.sym
{
T 4450 4150 5 10 0 0 90 0 1
device=DIODE
T 4550 4200 5 10 1 1 90 3 1
refdes=1N914
}
N 5350 4650 4850 4650 4
N 5350 3750 4850 3750 4
N 5150 4650 5150 5350 4
{
T 5150 5450 5 10 1 1 0 3 1
netname=+7.5V
}
N 6550 4200 6900 4200 4
N 6900 4200 6900 5100 4
N 6900 5100 5150 5100 4
N 5150 1100 5150 3750 4
C 7250 2950 1 90 0 resistor-1.sym
{
T 6850 3250 5 10 0 0 90 0 1
device=RESISTOR
T 6950 3150 5 10 1 1 90 0 1
refdes=Rlimit
}
C 7350 1950 1 90 0 led-3.sym
{
T 6700 2900 5 10 0 0 90 0 1
device=LED
T 6800 2250 5 10 1 1 90 0 1
refdes=RED
}
N 7150 2850 7150 2950 4
C 7050 1550 1 0 0 gnd-1.sym
N 7150 1850 7150 1950 4
N 7150 3850 7150 4000 4
N 7150 4000 6550 4000 4
N 6550 4400 7400 4400 4
C 7400 4000 1 0 0 spdt.sym
C 9000 2950 1 90 0 resistor-1.sym
{
T 8600 3250 5 10 0 0 90 0 1
device=RESISTOR
T 8700 3150 5 10 1 1 90 0 1
refdes=Rlimit
}
C 9100 1950 1 90 0 led-3.sym
{
T 8450 2900 5 10 0 0 90 0 1
device=LED
T 8550 2100 5 10 1 1 90 0 1
refdes=YELLOW
}
N 8900 2850 8900 2950 4
C 8800 1550 1 0 0 gnd-1.sym
N 8900 1850 8900 1950 4
C 10000 2950 1 90 0 resistor-1.sym
{
T 9600 3250 5 10 0 0 90 0 1
device=RESISTOR
T 9700 3150 5 10 1 1 90 0 1
refdes=Rlimit
}
C 10100 1950 1 90 0 led-3.sym
{
T 9450 2900 5 10 0 0 90 0 1
device=LED
T 9550 2150 5 10 1 1 90 0 1
refdes=GREEN
}
N 9900 2850 9900 2950 4
C 9800 1550 1 0 0 gnd-1.sym
N 9900 1850 9900 1950 4
N 8900 3850 8900 4200 4
N 8900 4200 8600 4200 4
N 8600 4600 9900 4600 4
N 9900 4600 9900 3850 4
B 1150 2100 3000 300 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
B 1050 2000 100 500 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
B 950 2150 100 200 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
N 3900 2300 3900 4950 4
{
T 3850 2450 5 10 1 1 90 0 1
netname=RED
}
N 3900 4950 5150 4950 4
N 3600 2200 3600 1600 4
{
T 3550 2050 5 10 1 1 90 6 1
netname=BLK
}
C 3500 1300 1 0 0 gnd-1.sym
N 2850 2150 2850 1100 4
N 2850 1100 5150 1100 4
N 2850 2350 2850 3100 4
N 2150 2150 2150 1350 4
T 6350 4450 9 10 1 0 0 0 1
NC
T 6350 3950 9 10 1 0 0 2 1
NO
T 8000 4900 9 10 1 0 0 3 1
Tortoise
T 2800 2050 5 10 1 1 90 6 1
netname=WHT
T 2800 2450 5 10 1 1 90 0 1
netname=BLU
T 2100 2050 5 10 1 1 90 6 1
netname=YEL
T 950 2600 9 10 1 0 0 0 1
CKT-IRSENSE