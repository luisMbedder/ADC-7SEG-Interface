/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: error_codes.h
*
* Description: Contains common error code enums.
*
* Known Issues: None
*
********************************************************************/
#ifndef ERROR_CODES_H
#define ERROR_CODES_H


/********************************************************************
* Enums
********************************************************************/

typedef enum ADC_ERROR_T
{
   NO_ADC_ERROR = 0,
   ADC_COMM_ERROR = 20,
   SIGNALS_ADC_ERROR,
   INTERNAL_ADC_ERROR,
} ADC_ERROR_T;

typedef enum ZERO_ERROR_T
{
   ZERO_OK = 0,
   ZERO_RANGE_ERROR = 30
} ZERO_ERROR_T;

typedef enum ZERO_OFFSET_ERROR_T
{
   NO_ZERO_OFFSET_ERROR = 0,
   ZERO_OFFSET_MAX_ERROR = 70
} ZERO_OFFSET_ERROR_T;

#endif //ERROR_CODES_H


