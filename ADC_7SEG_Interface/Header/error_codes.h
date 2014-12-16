/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: error_codes.h
*
* Description: Contains adc error code enums.
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
   ADC_COMM_ERROR,
   SIGNALS_ADC_ERROR,
   INTERNAL_ADC_ERROR,
} ADC_ERROR_T;

#endif //ERROR_CODES_H


