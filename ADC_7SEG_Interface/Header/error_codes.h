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

typedef enum EXCITATION_ERROR_T
{
   NO_EXCITATION_ERROR = 0,
   EXCITATION_SHORT_OR_LOW_IMPEDANCE_ERROR = 10,
   EXCITATION_OPEN_DETECTED_ERROR,
   EXCITATION_UNKOWN_XDCR_TYPE_ERROR
} EXCITATION_ERROR_T;

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

typedef enum CAL_ERROR_T
{
   CAL_OK = 0,
   CAL_WEIGHT_VAL_ERROR = 40,
   CAL_MIN_GAIN_ERROR
} CAL_ERROR_T;

typedef enum CAL_RIDER_ERROR_T
{
   RIDER_CAL_OK = 0,
   RIDER_CAL_MIN_NOT_MET = 50,
} CAL_RIDER_ERROR_T;

typedef enum DATABASE_ERROR_T
{
   NO_DATABASE_ERROR = 0,
   DATABASE_REPAIRED_DATA,
   DATABASE_EEPROM_COMM_ERROR = 60,
   DATABASE_CORRUPTED_DATA_ERROR,
   DATABASE_BOUNDS_ERROR,
   DATABASE_WRITE_RULES_ERROR,
   DATABASE_TEN_CAL_NOT_COMPLETE_ERROR,
   DATABASE_RIDER_CAL_NOT_COMPLETE_ERROR
} DATABASE_ERROR_T;

typedef enum ZERO_OFFSET_ERROR_T
{
   NO_ZERO_OFFSET_ERROR = 0,
   ZERO_OFFSET_MAX_ERROR = 70
} ZERO_OFFSET_ERROR_T;

typedef enum EXTERNAL_COMM_ERROR_T
{
   NO_EXTERNAL_COMM_ERROR = 0,
   EXTERNAL_COMM_EEPROM_MUTEX_TIMEOUT = 80,
   EXTERNAL_COMM_UIB_UPDATE_TIMEOUT,
   EXTERNAL_COMM_WRITE_TIMEOUT
} EXTERNAL_COMM_ERROR_T;


#endif //ERROR_CODES_H


