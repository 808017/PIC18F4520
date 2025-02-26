
// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

#include <p18F4520.h>
#include <string.h>


// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
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


#define _XTAL_FREQ 4000000


// Keypad connections
#define ROW1 LATBbits.LATB0
#define ROW2 LATBbits.LATB1
#define ROW3 LATBbits.LATB2
#define ROW4 LATBbits.LATB3

#define COL1 PORTBbits.RB4
#define COL2 PORTBbits.RB5
#define COL3 PORTBbits.RB6
#define COL4 PORTBbits.RB7

// LCD connections
#define RS LATCbits.LATC1 // Use separate pins for RS and EN
#define EN LATCbits.LATC2
#define DATA_PORT LATD

// Function prototypes
void initKeypad(void);
void initLCD(void);
void lcd_cmd(unsigned char command);
void lcd_data(unsigned char data);
void displayCharLCD(char c);
char readKeypad(void);
void MSDelay(unsigned int ms);

far rom const char mydata[] = "Key Pressed:";

void main(void) {
    char key;
    unsigned char z = 0;
    
    
   // Send LCD initialization commands
    for (z = 0; z < 4; z++) {
        lcd_cmd(lcd_cmd[z]);
        MSDelay(2);
    }

    // Initialize peripherals
    initKeypad();
    initLCD();

    // Display initial message
    lcd_cmd(0x80);
    MSDelay(2); // Delay to ensure the message is displayed
    
    // Display string
    for (z = 0; mydata[z] != '\0'; z++) {
        lcd_data(mydata[z]);
        MSDelay(5);
    }
     
     lcd_cmd(0xC0); // Move cursor to second line, first column
    
        while (1) {
        key = readKeypad(); // Read the keypad
        if (key != '\0') {  // If a key is pressed
            lcd_cmd(0x06);
            displayCharLCD(key);  // Display the pressed keY
            MSDelay(100); // Debounce delay
        }
    }
}

// Initialize keypad
void initKeypad(void) {
    TRISB = 0xF0; // RB0-RB3 as output (rows), RB4-RB7 as input (columns)
    LATB = 0x0F;  // Set all rows high initially
}

// Initialize LCD
void initLCD(void) {
    TRISD = 0x00; // Set PORTD as output (data pins)
    TRISCbits.TRISC1 = 0; // Set RC0 as output (RS)
    TRISCbits.TRISC2 = 0; // Set RC1 as output (EN)

    MSDelay(20); // Wait for LCD to power up

    lcd_cmd(0x38); // 8-bit mode, 2 lines, 5x7 dots
    MSDelay(5);
    lcd_cmd(0x0C); // Display on, cursor off
    MSDelay(5);
    lcd_cmd(0x01); // Clear display
    MSDelay(5);
    lcd_cmd(0x06); // Increment cursor
    MSDelay(5);
}

// Send command to LCD
void lcd_cmd(unsigned char command) {

    DATA_PORT = command;
    RS = 0; // Command mode
    EN = 1;
    MSDelay(2);
    EN = 0;
    MSDelay(2);
}

// Send data to LCD
void lcd_data(unsigned char data) {
    
    DATA_PORT = data;
    RS = 1; // Data mode
    EN = 1;
    MSDelay(2);
    EN = 0;
    MSDelay(2);
}

// Display a character on LCD
void displayCharLCD(char c) {
    lcd_data(c);
}


// Read keypad
char readKeypad(void) {
    char key = '\0'; // Default to no key pressed

    // Scan each row
    ROW1 = 0; ROW2 = 1; ROW3 = 1; ROW4 = 1;
    if (COL1 == 0) key = '1';
    if (COL2 == 0) key = '2';
    if (COL3 == 0) key = '3';
    if (COL4 == 0) key = 'A';
    MSDelay(2); // Small delay for debouncing

    ROW1 = 1; ROW2 = 0; ROW3 = 1; ROW4 = 1;
    if (COL1 == 0) key = '4';
    if (COL2 == 0) key = '5';
    if (COL3 == 0) key = '6';
    if (COL4 == 0) key = 'B';
    MSDelay(2);

    ROW1 = 1; ROW2 = 1; ROW3 = 0; ROW4 = 1;
    if (COL1 == 0) key = '7';
    if (COL2 == 0) key = '8';
    if (COL3 == 0) key = '9';
    if (COL4 == 0) key = 'C';
    MSDelay(2);

    ROW1 = 1; ROW2 = 1; ROW3 = 1; ROW4 = 0;
    if (COL1 == 0) key = '*';
    if (COL2 == 0) key = '0';
    if (COL3 == 0) key = '#';
    if (COL4 == 0) key = 'D';
    MSDelay(2);

    // Reset all rows
    ROW1 = 1; ROW2 = 1; ROW3 = 1; ROW4 = 1;

    return key;
}

// Delay function
void MSDelay(unsigned int ms) {
    unsigned int i, j;
    for (i=0;i< ms;i++)
        for (j=0;j<135;j++);
}