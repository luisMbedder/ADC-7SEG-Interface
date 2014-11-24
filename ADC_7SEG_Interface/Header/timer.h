/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: timer.h
*
* Description: Header file for timer related functions.
*
* Known Issues: None
*
********************************************************************/
#ifndef TIMER_H
#define TIMER_H

/********************************************************************
* Extern function prototypes
********************************************************************/
U16 read_ms_count(void);
U16 timeout(const U16 start_count, const U16 ms_delay);



#endif //TIMER_H


