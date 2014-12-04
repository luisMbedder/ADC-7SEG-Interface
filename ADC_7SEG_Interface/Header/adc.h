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
   AD779X_ERROR = 20 //same as ADC_COMM_ERROR
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


#define MAX_ADC_INIT_ATTEMPTS 10U
#define ADC_TIMEOUT_MS 100U



/********************************************************************
* Extern function prototypes
********************************************************************/
ADC_ERROR_T wait_for_voltage_sample_to_complete(void);
ADC_ERROR_T init_adc(void);
ADC_ERROR_T read_adc(void);
U16 filtered_voltage_signal(void);


#endif //ADC_H
