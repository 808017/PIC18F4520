Step-by-Step Explanation of the Code //\\ [Y..M..] //\\
                                 ┈╭━━━━━━━━━━━╮┈
                                 ┈┃╭━━━╮┊╭━━━╮┃┈
                                 ╭┫┃┈▇┈┃┊┃┈▇┈ ┣
                                 ┃┃╰━━━╯┊╰━━━╯┃┃
                                 ╰┫╭━╮╰━━━╯╭━╮┣╯
                                 ┈┃┃┣┳┳┳┳┳┳┳┫┃┃┈
                                 ┈┃┃╰┻┻┻┻┻┻┻╯┃┃┈
                                 ┈╰━━━━━━━━━━━╯┈
.......................................................................
This program is written in C18 Compiler for the PIC18F4520 microcontroller to 
control a 4-wire stepper motor using PORTB. The motor rotates in clockwise (CW) 
and anticlockwise (CCW) directions. The L293D motor driver is used to interface 
the motor with the microcontroller.
========================================================================================
Step 1: Header File Inclusion
......................
#include <p18f4520.h>
.....................
This includes the header file specific to the PIC18F4520, allowing access to special 
function registers
=======================================================================================
Step 2: Configuration Bits
.................................................................................
#pragma config OSC = HS  // High-Speed Oscillator
#pragma config WDT = OFF // Watchdog Timer disabled
#pragma config LVP = OFF // Low Voltage Programming disabled
#pragma config PBADEN = OFF // PORTB as digital I/O
................................................................................
OSC = HS → Configures the microcontroller to use a High-Speed Crystal Oscillator.
WDT = OFF → Disables the Watchdog Timer (prevents unintentional resets).
LVP = OFF → Disables Low-Voltage Programming Mode (ensures normal operation).
PBADEN = OFF → Ensures PORTB is digital I/O instead of an analog input.
======================================================================================
Step 3: Define System Frequency
...........................................
#define _XTAL_FREQ 8000000  // 8MHz Crystal
...........................................
The _XTAL_FREQ macro is used for generating delays using the __delay_ms() function.
======================================================================================
Step 4: Delay Function
.....................................
void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++) {
        for(j = 0; j < 100; j++);
    }
}
....................................
This function creates a software delay.
The nested loop runs ms times, creating an approximate delay.
=====================================================================================
Step 5: Define Step Sequence for Stepper Motor
.......................................................................................
const unsigned char stepSequence[4] = {0b00001001, 0b00000011, 0b00000110, 0b00001100};
.......................................................................................
This array stores the 4-step full-step sequence required to move the motor.
Each binary value represents which coils should be energized.
......................................................................................
Step	RB3	RB2	RB1	RB0	Coil Activation
1	1	0	0	1	Coil A & D
2	0	0	1	1	Coil B & A
3	0	1	1	0	Coil C & B
4	1	1	0	0	Coil D & C
......................................................................................
If we move through the sequence forwards, the motor moves clockwise.
If we move through the sequence backwards, the motor moves anticlockwise.
=======================================================================================
Step 6: Clockwise Rotation Function
.......................................................
void stepper_CW(unsigned int steps) {
    unsigned int i;
    for(i = 0; i < steps; i++) {
        PORTB = stepSequence[i % 4]; // Rotate forward
        delay_ms(200); // Adjust speed
    }
}
.......................................................
This function rotates the motor clockwise.
i % 4 ensures that the array loops through the 4-step sequence.
A delay (delay_ms(200)) controls the motor speed.
======================================================================================
Step 7: Anticlockwise Rotation Function
................................................................
void stepper_CCW(unsigned int steps) {
    unsigned int i;
    for(i = 0; i < steps; i++) {
        PORTB = stepSequence[3 - (i % 4)]; // Rotate backward
        delay_ms(200); // Adjust speed
    }
}
................................................................
This function rotates the motor anticlockwise.
3 - (i % 4) reverses the step sequence.
======================================================================================
Step 8: Main Function
..........................................................
void main() {
    TRISB = 0x00; // Configure PORTB as output
    PORTB = 0x00; // Initialize PORTB

    while(1) {
        stepper_CW(50);  // Rotate Clockwise
        delay_ms(1000);  // Wait 1 second
        stepper_CCW(50); // Rotate Anticlockwise
        delay_ms(1000);  // Wait 1 second
    }
}
.........................................................
Explanation:

TRISB = 0x00; → Configures PORTB as output for stepper motor control.
PORTB = 0x00; → Initializes PORTB to zero (all coils OFF).
while(1) {} → Runs indefinitely.
Calls:
stepper_CW(50); → Rotates the motor clockwise for 50 steps.
delay_ms(1000); → Waits 1 second.
stepper_CCW(50); → Rotates the motor anticlockwise for 50 steps.
delay_ms(1000); → Waits 1 second.
======================================================================================
L293D Motor Driver Explanation
.................................
The L293D is a dual H-Bridge motor driver IC that allows controlling high-power 
motors with low-power microcontroller signals.
.................................
Pin Configuration of L293D
Pin	Name	Description
1	EN1	Enables Motor A
2	IN1	Motor A Input 1
3	OUT1	Motor A Output 1
4	GND	Ground
5	GND	Ground
6	OUT2	Motor A Output 2
7	IN2	Motor A Input 2
8	VCC2	Motor Power Supply (9V-12V)
9	EN2	Enables Motor B
10	IN3	Motor B Input 1
11	OUT3	Motor B Output 1
12	GND	Ground
13	GND	Ground
14	OUT4	Motor B Output 2
15	IN4	Motor B Input 2
16	VCC1	Logic Power Supply (5V)
..........................................
Connections to PIC18F4520
.........................................
L293D Pin	Connected To
IN1 (Pin 2)	RB0
IN2 (Pin 7)	RB1
IN3 (Pin 10)	RB2
IN4 (Pin 15)	RB3
EN1 (Pin 1)	5V (Enable Motor A)
EN2 (Pin 9)	5V (Enable Motor B)
VCC1 (Pin 16)	5V
VCC2 (Pin 8)	9V-12V (Motor Power)
GND (Pin 4,5,12,13)	GND
========================================================================================
How L293D Works?

The H-Bridge Circuit inside L293D allows the direction of current to change,
thus controlling the motor rotation.

The Enable Pins (EN1, EN2) must be HIGH (5V) for the motor to run.

The Input Pins (IN1, IN2, IN3, IN4) control the stepper motor sequence.
=======================================================================================
Final Summary

The C18 program controls a 4-wire stepper motor.
PORTB (RB0-RB3) is used to control the step sequence.
L293D motor driver acts as an interface between the microcontroller and the stepper motor.
The motor rotates clockwise and anticlockwise based on the given step sequences.
.......................................................................................
