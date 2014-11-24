/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: adc.h
*
* Description: Header file for the ADC related functions.
*
* Known Issues: None
*
********************************************************************/
#ifndef ADC_H
#define ADC_H

#include "error_codes.h"


/********************************************************************
* Enums
********************************************************************/
typedef enum AD779X_STATE_T
{
   AD779X_RESET,
   AD779X_INIT = 10, 
   AD779X_ZERO, 
   AD779X_CAL, 
   AD779X_RUN,  
   AD779X_RESET_COMPLETE,
   AD779X_ERROR = 20 //same as TENSION_ADC_COMM_ERROR
} AD779X_STATE_T;

enum AD779X_REGISTERS 
{
   COMMS_REG = 12, 
   STATUS_REG, 
   MODE_REG, 
   CONFIG_REG, 
   DATA_REG, 
   ID_REG, 
   IO_REG, 
   OFFSET_REG, 
   FULL_SCALE_REG
};

/********************************************************************
* Defines
********************************************************************/
// Design min span for +/-15mV.  Unit will not accept CAL requests
// that don't meet this condition.
//
// Vref to ADC = +EXC * 0.4054
//  5V exc: Vref = 2.027V  (Nominal, subject to HW tolerance)
// 10V exc: Vref = 4.054V  (Nominal, subject to HW tolerance)
//
// In bipolar ADC configuration: (N = #bits), (Gain = Inamp gain = 8V/V),
//  (AIN = Analog voltage after input protection divider)
// Code = (2^(N-1))*((AIN*(Gain/Vref))+1)
// 16 bit ADC:  1 Code = 7.732uV   at ADC input pins
//   Input HW has 0.375V/V front end divider for protection.
// 16 bit ADC:  1 Code = 20.620uV  at terminal input
//    So: 15mV input would be 727.45 codes
// 24 bit ADC:  1 Code = 30.205nV   at ADC input pins
//   Input HW has 0.375V/V front end divider for protection.
// 24 bit ADC:  1 Code = 80.546nV  at terminal input
//    So: 15mV input would be 186228.99 codes
//
// As both 16 and 24 bit ADCs are filtered to 16 bit precision,
//  the 16 bit max span value is used to verify the min span
//  condition.

#define MINIMUM_TENSION_SPAN_BITS 727

#define MAX_ADC_INIT_ATTEMPTS 10U
#define ADC_TIMEOUT_MS 100U



/********************************************************************
* Extern function prototypes
********************************************************************/
ADC_ERROR_T wait_for_voltage_sample_to_complete(void);
ADC_ERROR_T init_adc(void);
ADC_ERROR_T read_adc(void);
U16 filtered_voltage_signal(void);


/********************************************************************
* External Variables
********************************************************************/
extern const U16 tension_filter_array[];

#endif //TENSION_ADC_H
