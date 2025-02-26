
// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

#include <p18F4520.h>
#include <delays.h>
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
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

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

// LCD Pin definitions
#define LCD_RS      0x02    // RD2
#define LCD_EN      0x04    // RD3
#define LCD_D4      0x10    // RD4
#define LCD_D5      0x20    // RD5
#define LCD_D6      0x40    // RD6
#define LCD_D7      0x80    // RD7

void Delay_ms(unsigned int ms)
{
    unsigned int i;
    for(i=0; i<ms; i++)
    {
        Delay1KTCYx(5);  // 5 for 20MHz
    }
}

void Lcd_Port(char a)
{
    if(a & 1)
        PORTD |= LCD_D4;
    else
        PORTD &= ~LCD_D4;

    if(a & 2)
        PORTD |= LCD_D5;
    else
        PORTD &= ~LCD_D5;

    if(a & 4)
        PORTD |= LCD_D6;
    else
        PORTD &= ~LCD_D6;

    if(a & 8)
        PORTD |= LCD_D7;
    else
        PORTD &= ~LCD_D7;
}

void Lcd_Cmd(char a)
{
    PORTC &= ~LCD_RS;    // RS = 0
    Lcd_Port(a>>4);      // Send upper nibble
    
    PORTC |= LCD_EN;     // EN = 1
    Delay_ms(1);
    PORTC &= ~LCD_EN;    // EN = 0
    
    Lcd_Port(a);         // Send lower nibble
    
    PORTC |= LCD_EN;     // EN = 1
    Delay_ms(1);
    PORTC &= ~LCD_EN;    // EN = 0
    
    Delay_ms(5);
}

void Lcd_Clear(void)
{
    Lcd_Cmd(0x01);
    Delay_ms(5);
}

void Lcd_Init(void)
{
    TRISC = 0x00;
    TRISD = 0X00;
    PORTD &= 0x0F;
    Delay_ms(20);
    
    Lcd_Port(0x03);
    PORTC |= LCD_EN;
    Delay_ms(5);
    PORTC &= ~LCD_EN;
    
    Delay_ms(5);
    Lcd_Port(0x03);
    PORTC |= LCD_EN;
    Delay_ms(5);
    PORTC &= ~LCD_EN;
    
    Delay_ms(5);
    Lcd_Port(0x03);
    PORTC |= LCD_EN;
    Delay_ms(5);
    PORTC &= ~LCD_EN;
    
    Delay_ms(5);
    Lcd_Port(0x02);
    PORTC |= LCD_EN;
    Delay_ms(5);
    PORTC &= ~LCD_EN;
    
    Lcd_Cmd(0x28);
    Lcd_Cmd(0x0C);
    Lcd_Cmd(0x06);
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x80);
}

void Lcd_Write_Char(char a)
{
    PORTC |= LCD_RS;
    Lcd_Port(a>>4);
    
    PORTC |= LCD_EN;
    Delay_ms(1);
    PORTC &= ~LCD_EN;
    
    Lcd_Port(a);
    
    PORTC |= LCD_EN;
    Delay_ms(1);
    PORTC &= ~LCD_EN;
    
    Delay_ms(5);
}

void Lcd_Set_Cursor(char row, char col)
{
    char temp;
    if(row == 1)
    {
        temp = 0x80 + col - 1;
        Lcd_Cmd(temp);
    }
    else if(row == 2)
    {
        temp = 0xC0 + col - 1;
        Lcd_Cmd(temp);
    }
}

// ADC Functions
void ADC_Init(void)
{
    ADCON0 = 0x01;    // ADC ON, Channel 0, GO/DONE = 0
    ADCON1 = 0x0E;    // AN0 analog, rest digital, right justified
    ADCON2 = 0x92;    // Right justified, 4Tad, FOSC/32
    TRISA = 0x01;     // RA0 as input
}

unsigned int ADC_Read(void)
{
    ADCON0bits.GO = 1;               // Start conversion
    while(ADCON0bits.GO);            // Wait for conversion
    return ((ADRESH << 8) + ADRESL); // Return result
}

void main(void)
{
    unsigned int adc_value;
    float temp;
    char temp_int, temp_dec;
    
    // Initialize ports
    ADCON1 = 0x0F;     // All digital for now
    TRISC = 0X00;
    TRISD = 0x00;      // PORTD as output
    
    Delay_ms(70);
    
    // Initialize LCD and ADC
    Lcd_Init();
    ADC_Init();
    
    while(1)
    {
        adc_value = ADC_Read();
        temp = (adc_value * 5000.0) / 1024.0;  // Convert to mV
        temp = temp / 10.0;  // Convert to temperature (LM35)
        
        temp_int = (char)temp;  // Integer part
        temp_dec = (char)((temp - temp_int) * 10);  // Decimal part
        
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_Char('R');
        Lcd_Write_Char('O');
        Lcd_Write_Char('O');
        Lcd_Write_Char('M');
        Lcd_Write_Char('_');
        Lcd_Write_Char('T');
        Lcd_Write_Char('E');
        Lcd_Write_Char('M');
        Lcd_Write_Char('P');
        Lcd_Write_Char(':');
        
        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char(temp_int/10 + '0');    // Tens digit
        Lcd_Write_Char(temp_int%10 + '0');    // Ones digit
        Lcd_Write_Char('.');
        Lcd_Write_Char(temp_dec + '0');       // Decimal digit
        Lcd_Write_Char(0xDF);                 // Degree symbol
        Lcd_Write_Char('C');
        
        Delay_ms(300);  // Update every second
    }
}