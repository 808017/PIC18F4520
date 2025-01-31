#include <p18f4520.h>
#include <delays.h>

// Configuration Bits
#pragma config OSC = HS    // High-Speed Oscillator
#pragma config WDT = OFF   // Disable Watchdog Timer
#pragma config LVP = OFF   // Disable Low Voltage Programming

#define RELAY LATCbits.LATC0  // Use LAT for Output Control

void delay_ms(unsigned int ms);

void main() {
    // Set RC0 as output (Relay control)
    TRISCbits.TRISC0 = 0;  

    while(1) {
        RELAY = 1;      // Turn ON relay
        delay_ms(100); // Wait 1 second (1000ms)

        RELAY = 0;      // Turn OFF relay
        delay_ms(100); // Wait 1 second (1000ms)
    }
}

// Function for Millisecond Delay
void delay_ms(unsigned int ms){
  unsigned int i,j;
   for(i=0;i<ms;i++)
     for(j=0;j<330;j++);
}