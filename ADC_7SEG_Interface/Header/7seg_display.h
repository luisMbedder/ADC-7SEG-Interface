/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: 7seg_display.h
*
* Description: Header file for 7 segment display related functions.
*
* Known Issues: None.
*
********************************************************************/

#ifndef ADC_7SEG_DISP_H
#define ADC_7SEG_DISP_H

#define MAX_I2C_DELAY_COUNT 10000U
#define DISPLAY_TIMEOUT_MS 100U

#define DECIMAL_POINT 0x80 //turns on decimal pt


typedef enum I2C_LOW_LEVEL_ERROR_T
{
   NO_I2C_ERROR = 0,
   I2C_NACK ,
   I2C_COLLISION,
   I2C_BUSY,
   I2C_TIMEOUT
} I2C_LOW_LEVEL_ERROR_T;

typedef enum bool
{ 
	false=0,
 	true 
} bool;

//extern functions
void update_display(float current_voltage);
I2C_LOW_LEVEL_ERROR_T init_display(void);
bool check_adc_error();

#endif //ADC_7SEG_DISP_H

