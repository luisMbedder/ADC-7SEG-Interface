/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: timer.c
*
* Description: This module contains the functions and variables used
*              to offer millisecond precision timeout capabiltity.
*              The timer3 interrupt is used to increment the 
*              m_millisecond_count once every millisecond.
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
U16 read_ms_count(void);
U16 timeout(const U16 start_count, const U16 ms_delay);



/********************************************************************
* Static Variables
********************************************************************/
static volatile U16 m_millisecond_count = 0;



/********************************************************************
* Function name : _T3Interrupt
*
* Created by : LuisMbedder
*
* Description : Timer3 interrupts every 1 ms and increments the
*               m_millisecond_count.
*
* Notes : None
********************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
   TMR3 = 0x00;//Clear contents of the timer register
   T3CON = 0x00;//Stops the Timer3 and reset control reg.

   if (m_millisecond_count < 0xFFFF)
      m_millisecond_count++;
   else
      m_millisecond_count = 0;

   IFS0bits.T3IF = 0;//Reset Timer3 interrupt flag and return from ISR
   T3CONbits.TON = 1;//Start Timer3 with prescaler settings at 1:1
} //_T3Interrupt


/********************************************************************
* Function name : read_ms_count
*
* returns : The current m_millisecond_count value.
*
* Created by : LuisMbedder
*
* Description : This function simply returns the current millisecond
*               count.
*
* Notes : None
********************************************************************/
U16 read_ms_count(void)
{
   U16 return_count = m_millisecond_count;
   return (return_count);
}//read_ms_count


/********************************************************************
* Function name : timeout
*
* start_count : The start millisecond count.
* ms_delay : The aount of time (in milliseconds) that must pass
*            before the timeout occurs.
*
* returns : TRUE if the amount of time passed sine start_count is
*           greater than ms_delay.
*
* Created by : LuisMbedder
*
* Description : This function compares the amount of time that has
*               passed since start_count and returns TRUE if that
*               time is greater than ms_delay.  It is capable of
*               handling m_millisecond_count overflow.
*
* Notes : None
********************************************************************/
U16 timeout(const U16 start_count, const U16 ms_delay)
{
   U16 current_count = m_millisecond_count;
   U32 timout_count = ( (U32)start_count + (U32)ms_delay );

   if ( (U32)current_count > timout_count )
      return (TRUE);
   else
   {   
      //if the time of sampling (which should be in the past) 
      // is greater than the current time,
      // assume overflow of the counter variable occurred
      if(start_count > current_count)
      {
         //is the delta between the sampling time and the current time
         // greater than the requested timeout ticks?
         if( ((0xffff - start_count) + current_count) > ms_delay)			
            return (TRUE);
         else
            return (FALSE);
      }
      else
         return (FALSE);
   }
}//timeout




