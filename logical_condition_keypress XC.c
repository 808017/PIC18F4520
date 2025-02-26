
#include <xc.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>



    
#pragma config OSC = INTIO67
#pragma config PWRT = ON        // power up timer enabled
#pragma config BOREN= OFF   // Brown-out Reset enabled in hardware only (SBOREN is disabled) 
#pragma config WDT  = OFF       // WDT disabled
#pragma config MCLRE= ON  
#pragma config LVP  = OFF 
#pragma config PBADEN  = OFF 
#pragma config CPD = ON
#pragma config DEBUG = OFF 
#pragma config CP0 = ON 
#pragma config CP1 = ON 
#pragma config CP2 = ON 
#pragma config CP3 = ON 
#pragma config CPB = ON 

#define key1 PORTBbits.RB0
#define key2 PORTBbits.RB1



#define en PORTCbits.RC3
#define rw PORTCbits.RC2
#define rs PORTCbits.RC1

#define _XTAL_FREQ 4000000


void lcd_string(unsigned char *S);
void lcdinit(void);
void lcdcmd(unsigned char data);
void lcddata(unsigned char data);

///////////////////////////////////////////////////////////////////////////////////

void lcd_string(unsigned char *S)
{
	while(*S)
    {
    	lcddata(*S);
		S++;
		}
}
//////////////////////////////////

////////////////////////////
void lcdcmd(unsigned char data)
{
	PORTD=data;
	rs=0;
    __delay_ms(1);
    en=1;
	__delay_ms(1);
	en=0;
	
}

////////////////////////////////////////////////////
void lcddata(unsigned char data)
{
	PORTD=data;
	rs=1;
    __delay_ms(1);
   	en=1;
	__delay_ms(1);
	en=0;
}
////////////////////////////////////////////////////
void lcdinit(void)
{
	lcdcmd(0x01);
	__delay_ms(40);
	lcdcmd(0x38);
	lcdcmd(0x0C);
  	lcdcmd(0x06);
    lcdcmd(0x80);
    
}

//////////////////////////////////////////////////////////////////

void main(void)
{
OSCCON=0xEF; //EF 4 MHz


TRISB=0X0F;
TRISC=0x00;
TRISD=0x00;
rw=0;
en=0;

__delay_ms(100);

lcdinit();
lcdcmd(0x80);
lcd_string("RIT WELCOMES");
lcdcmd(0xC0);
lcd_string("YOU");
__delay_ms(500);


while(1)
{
    if(key1==0 && key2==0)
    {
    lcdcmd(0x01);
     lcd_string("Both Key Pressed");
     __delay_ms(500);
    }
    else if(key1==0)
    {
    lcdcmd(0x01);
     lcd_string("Key1 Pressed");
     __delay_ms(500);
    }
    else if(key2==0)
    {
    lcdcmd(0x01);
     lcd_string("Key2 Pressed");
     __delay_ms(500);
    }
    else 
    {
    lcdcmd(0x01);
     lcd_string("No Key Pressed");
     __delay_ms(500);
    }
    
    __delay_ms(500);
}        
    
}

