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

#define MAX_I2C_DELAY_COUNT 10000U
#define DISPLAY_TIMEOUT_MS 100U

typedef int bool;
#define true 1
#define false 0
#define DECIMAL_POINT 0x2E //ascii deciaml


typedef enum I2C_LOW_LEVEL_ERROR_T
{
   NO_I2C_ERROR = 0,
   I2C_NACK ,
   I2C_COLLISION,
   I2C_BUSY,
   I2C_TIMEOUT
} I2C_LOW_LEVEL_ERROR_T;

