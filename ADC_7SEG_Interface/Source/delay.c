/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: delays.c
*
*
* Description: Contains delay functions that utilize the Timer1
*              peripheral.
*
* Known Issues: None
*
********************************************************************/


/********************************************************************
* Include Section
********************************************************************/
#include "p24FJ128GB108.h"
#include "adc_7seg.h"


/********************************************************************
* Function Prototype Section
********************************************************************/ 
void delay_ms(const U16 milliseconds);
void delay_10us(const U16 ten_microseconds);


/********************************************************************
* Function name : delay_ms
*
* milliseconds : number of milliseconds to delay
*
* Created by : LuisMbedder
*
* Description : Delays for the specified number of milliseconds.
*
* Notes : None
********************************************************************/
void delay_ms(const U16 milliseconds)
{
   U16 ms_count = milliseconds;
   //configure timer1 control register
   //bit 15 = 1, timer On bit
   //bit 14 = 0, unimplemented, read as 0
   //bit 13 = 0, continue in idle mode
   //bit 12 = 0, unimplemented, read as 0
   //bit 10 = 0, unimplemented, read as 0
   //bit 9 = 0, unimplemented, read as 0
   //bit 8 = 0, unimplemented, read as 0
   //bit 7 = 0, unimplemented, read as 0
   //bit 6 = 0, gated time accumulation disabled
                //prescale select set to 1:1
   //bit 5 = 0, prescale select bits 1
   //bit 4 = 0, prescale select bits 0
   //bit 3 = 0, unimplemented, read as 0
   //bit 2 = 0, do not synchronize external clock input
   //bit 1 = 0, use internal clock (FOSC/2)
   //bit 0 = 0, unimplemented, read as 0
   T1CON = 0x8000;

   while (ms_count--)
   {
        TMR1 = 0;
#ifdef FOSC_32MHZ
        while (TMR1<16000);
#endif //#ifdef FOSC_32MHZ
#ifdef FOSC_16MHZ
        while (TMR1<8000);
#endif //#ifdef FOSC_16MHZ
   }
}//delay_ms


/********************************************************************
* Function name : delay_10us
*
* milliseconds : number of microseconds to delay
*
* Created by : LuisMbedder
*
* Description : Delays for the specified multiple of 10 microseconds.
*
* Notes : None
********************************************************************/
void delay_10us(const U16 ten_microseconds)
{
   U16 ten_us_count = ten_microseconds;
   //configure timer1 control register
   //bit 15 = 1, timer On bit
   //bit 14 = 0, unimplemented, read as 0
   //bit 13 = 0, continue in idle mode
   //bit 12 = 0, unimplemented, read as 0
   //bit 10 = 0, unimplemented, read as 0
   //bit 9 = 0, unimplemented, read as 0
   //bit 8 = 0, unimplemented, read as 0
   //bit 7 = 0, unimplemented, read as 0
   //bit 6 = 0, gated time accumulation disabled
                //prescale select set to 1:1
   //bit 5 = 0, prescale select bits 1
   //bit 4 = 0, prescale select bits 0
   //bit 3 = 0, unimplemented, read as 0
   //bit 2 = 0, do not synchronize external clock input
   //bit 1 = 0, use internal clock (FOSC/2)
   //bit 0 = 0, unimplemented, read as 0
   T1CON = 0x8000;

   while (ten_us_count--)
   {
        TMR1 = 0;
#ifdef FOSC_32MHZ
        while (TMR1<160);
#endif //#ifdef FOSC_32MHZ
#ifdef FOSC_16MHZ
        while (TMR1<80);
#endif //#ifdef FOSC_16MHZ
   }
}//delay_10us

