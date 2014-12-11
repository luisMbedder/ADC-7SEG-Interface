/********************************************************************
* Project: ADC_7SEG_Interface
* File: main.c
*
* Created By: LuisMbedder
*
* Description: Main module to interface an AD7799 24-bit
*			   ADC via SPI bus and a 4-digit 7-segment display 
*			   using the I2C bus. The 7-segment display is used to 
*			   to display the converted analog signal. 
*
* Known Issues: None
*
* Original Compiler: C30 Ver. 3.30
* Original IDE: MPLAB Ver. 8.70
* Built on Windows 7 machine on an LV24-33 development board 
* from Mikroelektronica.
*
********************************************************************/

/********************************************************************
* Latest Mem Sizes v1.0: XXXX prog words.  XXX RAM bytes.
********************************************************************/


/********************************************************************
* Include Section
********************************************************************/
#include "p24FJ128GB108.h"
#include "GenericTypeDefs.h"
#include "adc_7seg.h"
#include "7seg_display.h"
#include "delay.h"
#include "init.h"
#include "timer.h"



/********************************************************************
* Configuration Bit Macros
********************************************************************/
#define USE_WATCHDOG

_CONFIG1(  JTAGEN_OFF       //JTAG Off
         & GCP_OFF          //General Segment Program Memory Code Protection Off
         //& GCP_ON           //Code protection is enabled for the entire program memory space
         & GWRP_OFF         //General Segment Code Flash Write Protection Off
         //& GWRP_ON          //General Segment Code Flash Write Protection On
         & BKBUG_OFF        //Background Debugger Disabled
         //& BKBUG_ON         //Background Debugger Enabled
         & ICS_PGx1         //Emulator functions are shared with PGEC1/PGED1
#ifdef USE_WATCHDOG
         & FWDTEN_ON        //Watchdog Timer is enabled
#endif
#ifndef USE_WATCHDOG
         & FWDTEN_OFF       //Watchdog Timer is disabled
#endif
         & WINDIS_OFF       //Standard Watchdog Timer enabled (windowed timer is disabled)
         //& WINDIS_ON        //Windowed Watchdog Timer enabled; FWDTEN must be ON
         & FWPSA_PR32       //WDT Prescaler ratio of 1:32 for a WDT resolution of 1 ms
         & WDTPS_PS4096    //Watchdog Timer Postscaler 1:4,096 for a WDT timeout of 4 seconds
        )
         
_CONFIG2(  IESO_OFF         //IESO mode (Two-Speed Start-up) disabled
           //IESO_ON          //IESO mode (Two-Speed Start-up) enabled
         & PLLDIV_DIV2      //Oscillator input divided by 2 (8MHz input) for USB 96 MHz PLL Prescaler
         & PLL_96MHZ_ON     //PLL enabled (required for all USB operations)
         //& FNOSC_FRCPLL     //Fast RC oscillator w/ divide and PLL
         & FNOSC_PRIPLL     //Primary oscillator (XT, HS, EC) w/ PLL
         & FCKSM_CSECME     //Clock switching and Fail-Safe Clock Monitor are enabled
         //& FCKSM_CSDCMD     //Clock switching and Fail-Safe Clock Monitor are disabled
         & OSCIOFNC_ON      //OSCO/CLKO/RC15 functions as port I/O (RC15) 
         & IOL1WAY_ON       //Only one write RP Registers permitted
         & DISUVREG_OFF     //Internal USB 3.3V Regulator is disabled
         //& POSCMOD_NONE     //Primary Oscillator disabled
         & POSCMOD_XT       //XT oscillator
        )

_CONFIG3(  WPEND_WPSTARTMEM //Write Protect from page 0 to WPFP
         //WPEND_WPENDMEM   //Write Protect from WPFP to the last page of memory
         & WPCFG_WPCFGDIS   //Last page (at the top of program memory) and Flash Configuration Words are not protected
         //& WPCFG_WPCFGEN    //Last page and Flash Configuration Words are code protected
         & WPDIS_WPDIS      //Segmented code protection disabled
         //& WPDIS_WPEN       //Segmented code protection enabled; protected segment defined by WPEND, WPCFG and WPFPx Configuration bits
         & WPFP_WPFP511     //Protected Code Segment Boundary Page bits
        )


/********************************************************************
* Module Interface Function Prototype Section
********************************************************************/
RESET_CONDITION_T reset_condition(void);
ADC_ERROR_T voltage_error(void);
U16 max_loop_time(void);


/********************************************************************
* Module Functions Prototype Section
********************************************************************/
static RESET_CONDITION_T check_for_power_on_reset();
static void process_voltage_led(void);
static void process_adc(void);
static void calc_loop_time(const U16 start_tick, const U16 end_tick);


/********************************************************************
* Microchip USB related Function Prototype Section
********************************************************************/
static void initialize_system(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();


/********************************************************************
* Static Variables
********************************************************************/
RESET_CONDITION_T m_reset_condition = POWER_ON_RESET;
ADC_ERROR_T m_voltage_error = NO_ADC_ERROR;
I2C_LOW_LEVEL_ERROR_T m_display_error = NO_I2C_ERROR;
static U16 m_max_loop_time = 0;

/********************************************************************
* Microchip USB related Variables
********************************************************************/



#pragma code
/********************************************************************
* Function name : main
*
* returns : The microcontroller reset state as enum type 
*           RESET_CONDITION_T.
*
* Created by : LuisMbedder
*
* Description : The main function initializes the system and loops
*               infinitely processing the adc and updating the 7-segment
*				displays. 
*
* Notes : None
********************************************************************/
int main(void)
{ 
//   ClrWdt();
   m_reset_condition = check_for_power_on_reset(); 
//   ClrWdt();  
   initialize_system();

   m_max_loop_time = 0;
 
      while (1)
      {
         process_adc();
         

      }

}//main


/********************************************************************
* Function name : reset_condition
*
* returns : The microcontroller reset state as enum type 
*           RESET_CONDITION_T.
*
* Created by : LuisMbedder
*
* Description : This function returns the reset state of the 
*               microcontroller.  It can only be read once, after
*               which errors are cleared.
*
* Notes : None
********************************************************************/
RESET_CONDITION_T reset_condition(void)
{
   RESET_CONDITION_T return_val = m_reset_condition;

   //clear any reset error conditions so that they are only reported
   // to the front board once.  This will prevent the same error from
   // being displayed multiple times if the front board is to reset
   // for some reason at runtime.
   m_reset_condition = POWER_ON_RESET;

   return (return_val);
}//reset_condition


/********************************************************************
* Function name : voltage_error
*
* returns : The current voltage error as enum type ADC_ERROR_T.
*
* Created by : LuisMbedder
*
* Description : This function returns the voltage error state 
*               returned from the adc module.
*
* Notes : None
********************************************************************/
ADC_ERROR_T voltage_error(void)
{
   ADC_ERROR_T voltage_error = m_voltage_error ;
   return (voltage_error);
}//voltage_error

/********************************************************************
* Function name : max_loop_time
*
* returns : The maximum software loop time
*
* Created by : LuisMbedder
*
* Description : This function returns the maximum software loop time.
*
* Notes : None
********************************************************************/
U16 max_loop_time(void)
{
   return m_max_loop_time;
}


/********************************************************************
* Function name : check_for_power_on_reset
*
* returns : The power on reset condition as enum type 
*           RESET_CONDITION_T.
*
* Created by : LuisMbedder
*
* Description : This function returns the power on reset state and
*               then clears the reset flags.  This means its return
*               value is only valid one time.
*
* Notes : None
********************************************************************/
static RESET_CONDITION_T check_for_power_on_reset(void)
{
   RESET_CONDITION_T reset_condition = POWER_ON_RESET;
   U16 rcon_read = 0;

   rcon_read = RCON;

                   //FEDCBA9876543210
   if (rcon_read & 0b0000001000000000) //Configuration Mismatch Reset
      reset_condition = CONFIGURATION_MISMATCH_RESET;  
                        //FEDCBA9876543210
   else if (rcon_read & 0b0000000000000011) //Power on Reset
      reset_condition = POWER_ON_RESET;
                        //FEDCBA9876543210
   else if (rcon_read & 0b0000000001000000) //Software Reset
      reset_condition = SOFTWARE_RESET;
                        //FEDCBA9876543210
   else if (rcon_read & 0b0000000000000010) //Brown-Out Reset
      reset_condition = BROWN_OUT_RESET;
                        //FEDCBA9876543210
   else if (rcon_read & 0b0000000000010000) //Watchdog Time-out Reset
      reset_condition = WATCHDOG_RESET;  
                        //FEDCBA9876543210
   else if (rcon_read & 0b1000000000000000) //Trap Event Reset
      reset_condition = TRAP_EVENT_RESET;  
                        //FEDCBA9876543210
   else if (rcon_read & 0b0100000000000000) //Illegal Opcode Reset
      reset_condition = ILLEGAL_OPCODE_RESET;  

   //clear all reset flags
   RCON = 0x00;

   return (reset_condition);
}//check_for_power_on_reset


/********************************************************************
* Function name : initialize_system
*
* Created by : LuisMbedder
*
* Description : This function returns initializes the system by
*               configuring the microcontroller and calling the
*               required init functions.
*
* Notes : None
********************************************************************/
static void initialize_system(void)
{
   configure_processor();
   m_voltage_error = init_adc();
   m_display_error = init_display();
}//initialize_system


/********************************************************************
* Function name : process_adc
*
* Created by : LuisMbedder
*
* Description : This function acts as the main loop of the
*               controller.  It is paced by the ADC.  The
*               basic sequence of operations is to sample and process
*               the analog inputs and update the 7-segment displays. 
*
* Notes : None
********************************************************************/
static void process_adc(void)
{
   ClrWdt(); //must kick wathdog at least once every 4 seconds

   static U16 startup_delay_count = 0;
   static U16 loop_time_delay_count = 0;
   static U16 start_tick = 0;
   static U16 end_tick = 0;
  

   //the adc drives the loop timing
   //it samples voltage 242 times a second for a loop time around 4.1ms
   end_tick = TMR5;
   if (loop_time_delay_count >= 1000U)
      calc_loop_time(start_tick, end_tick);

   m_voltage_error = wait_for_voltage_sample_to_complete(); //this holds on the ADC, which drives the loop time

   start_tick = TMR5;

   //sample voltage signal 
   m_voltage_error = read_adc();

   if (startup_delay_count < STARTUP_DELAY_COUNT)
      startup_delay_count++;

   if (loop_time_delay_count < 1000U)
      loop_time_delay_count++;
}//process_adc


/********************************************************************
* Function name : calc_loop_time
*
* start_tick : timer sample from loop start
* end_tick : timer sample from loop end
*
* Created by : LuisMbedder
*
* Description : Calculates the loop time and updates m_max_loop_time
*               if the time is greater than the previous max loop
*               time.
*
* Notes : None
********************************************************************/
static void calc_loop_time(const U16 start_tick, const U16 end_tick)
{
   U16 loop_time = 0;

   if (end_tick > start_tick)
      loop_time = (end_tick - start_tick);
   else //a timer rollever occured
      loop_time = ((0xFFFF - start_tick) + end_tick);

   if (loop_time > m_max_loop_time)
      m_max_loop_time = loop_time;
}


