#include <p18f4520.h>

// Configuration Bits (Modify as needed for your hardware)
#pragma config OSC = HS     // High-speed Oscillator
#pragma config WDT = OFF    // Watchdog Timer Disabled
#pragma config LVP = OFF    // Low Voltage Programming Disabled

// Define relay and switch using proper bit access
#define RELAY  PORTCbits.RC0
#define SWITCH PORTCbits.RC1

void main() {
    // Set up the ports
    TRISCbits.TRISC0 = 0;  // Set RC0 as output (for relay control)
    TRISCbits.TRISC1 = 1;  // Set RC1 as input (for switch)

    // Optional: Ensure PORTC outputs are low initially
    RELAY = 0;

    while(1) {
        if (SWITCH == 1) {  // If switch is pressed
            RELAY = 1;      // Turn ON relay
        } else {
            RELAY = 0;      // Turn OFF relay
        }
    }
}
