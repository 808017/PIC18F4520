/*
 * File:   newmain.c
 * Author: Dr.ABK
 *
 * Created on 8 September, 2020, 10:38 AM
 */
#include<xc.h>


#pragma config OSC = INTIO67
#pragma config PWRT = ON        // power up timer enabled
#pragma config BOREN= OFF   // Brown-out Reset enabled in hardware only (SBOREN is disabled) 
#pragma config WDT  = OFF       // WDT disabled
#pragma config MCLRE= OFF  
#pragma config LVP  = OFF 
#pragma config PBADEN  = OFF 
#pragma config CPD = ON
#pragma config DEBUG = OFF 
#pragma config CP0 = ON 
#pragma config CP1 = ON 
#pragma config CP2 = ON 
#pragma config CP3 = ON 
#pragma config CPB = ON 


#define _XTAL_FREQ 4000000
/////////////////////////////////////////////

void main(void)
{
OSCCON=0xEF; //EF 4 MHz
TRISB=0XFF;
TRISC=0x00;

while(1)
{
    if(PORTBbits.RB0==0)
    {
        PORTCbits.RC0=0;
    }
    else
    {
        PORTCbits.RC0=1;
    }
__delay_ms(50);
}        

    
}


