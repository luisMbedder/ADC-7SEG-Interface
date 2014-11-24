//#define THIS_IS_MAIN_APPLICATION
///#include "Main.h"
//#include "Lib.h"

// Declare AppConfig structure and some other supporting stack variables
//APP_CONFIG AppConfig;

// Private helper functions.
#include "p24FJ128GB108.h"
#include "stdlib.h"
#include "string.h"

void __attribute__((interrupt, auto_psv)) _DefaultInterrupt(void)
{
//	DEBUG_PRINT( "!!! Default interrupt handler !!!\r\n" );
	while (1) {
		Nop();
		Nop();
		Nop();
	}
}

void __attribute__((interrupt, auto_psv)) _OscillatorFail(void)
{
//	DEBUG_PRINT( "!!! Oscillator Fail interrupt handler !!!\r\n" );
	while (1) {
		Nop();
		Nop();
		Nop();
	}
}

void __attribute__((interrupt, auto_psv)) _AddressError(void)
{
//	DEBUG_PRINT( "!!! Address Error interrupt handler !!!\r\n" );
	while (1) {
		Nop();
		Nop();
		Nop();
	}
}

void __attribute__((interrupt, auto_psv)) _StackError(void)
{
//	DEBUG_PRINT( "!!! Stack Error interrupt handler !!!\r\n" );
	while (1) {
		Nop();
		Nop();
		Nop();
	}
}

void __attribute__((interrupt, auto_psv)) _MathError(void)
{
//	DEBUG_PRINT( "!!! Math Error interrupt handler !!!\r\n" );
	while (1) {
		Nop();
		Nop();
		Nop();
	}
}


