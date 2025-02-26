
// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

#include <p18F4520.h>

// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)



#define _XTAL_FREQ 4000000  // 4 MHz Crystal Frequency

// Function Prototypes
void PWM_Init(void);
void SetDutyCycle(unsigned int duty);
void delay_ms(unsigned int milliseconds);

void main() {
    PWM_Init(); // Initialize PWM

    while (1) {
        SetDutyCycle(256); // 25% Duty Cycle
        delay_ms(200);

        SetDutyCycle(512); // 50% Duty Cycle
        delay_ms(200);

        SetDutyCycle(768); // 75% Duty Cycle
        delay_ms(200);

        SetDutyCycle(1023); // 100% Duty Cycle
        delay_ms(200);
    }
}

// Initialize PWM on CCP1 (RC2)
void PWM_Init(void) {
    TRISCbits.TRISC2 = 0; // Set RC2 as Output (PWM Pin)

    // Configure CCP1 in PWM Mode
    CCP1CON = 0b00001100; // PWM mode, P1A active high

    // Set Timer2 for PWM
    T2CON = 0b00000101;   // Timer2 ON, Prescaler = 4

    // Set PWM frequency (Lowered to ~2 kHz for motor control)
    PR2 = 124; 
    
    CCPR1L = 62;
    
    CCP1CONbits.DC1B = 0; // Lower 2 bits of duty cycle
}

// Set PWM Duty Cycle (10-bit resolution)
void SetDutyCycle(unsigned int duty) {
    if (duty > 1023) duty = 1023; // Limit to 10-bit value
    CCPR1L = duty >> 2;  // Upper 8 bits
    CCP1CONbits.DC1B = duty & 0x03;  // Lower 2 bits
}

// Delay function for C18 (Replaces __delay_ms)
void delay_ms(unsigned int milliseconds) {
    int i, j;
    for (i = 0; i < milliseconds; i++)
        for (j = 0; j < 333; j++);
}