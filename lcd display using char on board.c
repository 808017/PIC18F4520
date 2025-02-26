
// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

#include <p18F4520.h>

// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
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

#define _XTAL_FREQ 1000000

#define lcd_data PORTD      // Data pins connected to PORTD
#define rs PORTCbits.RC1    // Register Select pin
#define en PORTCbits.RC2    // Enable pin

void lcdcmd(unsigned char value);
void lcddata(unsigned char value);
void MSDelay(unsigned int iteam);

void main() {
    // Set PORTD and PORTB as output
    TRISD = 0; 
    TRISC = 0;

    // Initial state
    en = 0;
    MSDelay(250);

    // LCD initialization
    lcdcmd(0x30); // 8-bit mode, 1-line display
    MSDelay(250);
    lcdcmd(0x0E); // Display ON, cursor ON
    MSDelay(15);
    lcdcmd(0x01); // Clear display
    MSDelay(15);
    lcdcmd(0x80); // Set cursor to position
    MSDelay(15);
    lcdcmd(0x0F);
    MSDelay(15);

    // Write data to LCD
    lcddata('A');
    MSDelay(100);
    lcddata('N');
    MSDelay(100);
    lcddata('I');
    MSDelay(100);
    lcddata('R');
    MSDelay(100);
    lcddata('U');
    MSDelay(100);
    lcddata('D');
    MSDelay(100);
    lcddata('H');
    MSDelay(100);
    lcddata('_');
    MSDelay(100);
    lcddata('L');
    MSDelay(100);
    lcddata('O');
    MSDelay(100);
    lcddata('V');
    MSDelay(100);
    lcddata('E');
    MSDelay(100);
    lcddata('_');
    MSDelay(100);
    lcddata('S');
    MSDelay(100);
    lcddata('.');
    MSDelay(100);
    lcddata('.');
    MSDelay(100);
 
 


    lcdcmd(0x30);
    MSDelay(200);

    lcdcmd(0x0E); // Display ON, cursor ON
    MSDelay(200);

    lcdcmd(0x01); // Clear display
    MSDelay(200);

    lcdcmd(0x80);
    MSDelay(15);

    lcddata('.');
    MSDelay(100);
    lcddata('.');
    MSDelay(100);
    lcddata('.');
    MSDelay(100);
    lcddata('.');
    MSDelay(100);
    lcddata('.');
    MSDelay(100); 
    lcddata('.');
    MSDelay(100);
  
   

    lcddata('[');
    MSDelay(400);

    lcddata('Y');
    MSDelay(400);

    lcddata('M');
    MSDelay(400);

 
    lcddata(']');
    MSDelay(400);

    lcddata('.');
    MSDelay(600);
    lcddata('.');
    MSDelay(200);
    lcddata('.');
    MSDelay(200);
    lcddata('.');
    MSDelay(200); 
    lcddata('.');
    MSDelay(200);
    lcddata('.');
    MSDelay(200);
    lcddata('.');
    MSDelay(200);
   
  

   

    while (1); // Infinite loop
}

void lcdcmd(unsigned char value) {
    lcd_data = value; // Send command
    rs = 0;           // Command mode
    en = 1;           // Generate enable pulse
    MSDelay(1);
    en = 0;
}

void lcddata(unsigned char value) {
    lcd_data = value; // Send data
    rs = 1;           // Data mode
    en = 1;           // Generate enable pulse
    MSDelay(1);
    en = 0;
}

void MSDelay(unsigned int iteam) {
    unsigned int i, j;
    for (i = 0; i < iteam; i++) {
        for (j = 0; j < 100; j++); // Simple delay loop
    }
}