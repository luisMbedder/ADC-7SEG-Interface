/********************************************************************
* Project: ADC_7SEG_Interface
* File: adc_7seg.h
*
* Description: Contains central Defines used by the software.
*
* Known Issues: None
*
********************************************************************/
#ifndef ADC_7SEG_H
#define ADC_7SEG_H

#include "error_codes.h"



/********************************************************************
* Enums
********************************************************************/
typedef enum RESET_CONDITION_T
{
   POWER_ON_RESET = 0,
   SOFTWARE_RESET,
   BROWN_OUT_RESET,
   WATCHDOG_RESET,
   TRAP_EVENT_RESET,
   ILLEGAL_OPCODE_RESET,
   CONFIGURATION_MISMATCH_RESET
} RESET_CONDITION_T;


/********************************************************************
* Defines
********************************************************************/

//type definitions
typedef unsigned char U8;
typedef unsigned int U16;
typedef unsigned long U32;
typedef unsigned long long U64;
typedef signed char S8;
typedef signed int S16;
typedef signed long S32;
typedef signed long long S64;

#define TRUE 1U
#define FALSE 0U
#define NONE 0

#define U16_MAX_VALUE 0xFFFF
#define LOOP_TIME_MS 4U
#define FLT_LOOP_TIME_MS 4.13223
#define VOLTAGE_FILTER_DEPTH 16U 

//FOSC speed, define 16 or 32MHz
//#define FOSC_16MHZ
#define FOSC_32MHZ
#if  (!defined (FOSC_32MHZ) && !defined (FOSC_16MHZ))
#error "Invalid FOSC specified in adc_7seg.h" //define 32 or 16 MHZ FOSC above
#endif
//I2C display bus speed
//#define FSCL_1MHZ
#define FSCL1_100KHZ
#if  (!defined (FSCL_1MHZ) && !defined (FSCL1_100KHZ))
#error "Invalid FSCL specified in adc_7seg.h" //define 1MHz or 400Khz FSCL above
#endif

//System definitions
#define STARTUP_DELAY_COUNT 32U


//chip select pins
#define ADC_CS_PIN LATDbits.LATD12              //output, use latch

#define SELECT 0U
#define DESELECT 1U

//AD779X !RDY / MISO pin
#define AD779X_NOT_RDY_PIN PORTDbits.RD3             //input, use port

//I2C Pins (bit toggled only for display reset)
#define SCL_PIN LATAbits.LATA14                      //output, use latch
#define SDA_PIN LATAbits.LATA15                      //output, use latch


/********************************************************************
* Extern function prototypes
********************************************************************/
RESET_CONDITION_T reset_condition(void);
ADC_ERROR_T voltage_error(void);
U16 max_loop_time(void);
extern U8 phase;

#endif//ADC_7SEG.H
