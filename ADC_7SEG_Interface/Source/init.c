/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: init.c
*
* Description: Contains functions necessary to configure and 
*              initialize the microcontroller.
*
* Known Issues: None
*
********************************************************************/


/********************************************************************
* Include Section
********************************************************************/
#include "p24FJ128GB108.h"
#include "adc_7seg.h"
#include "delay.h"



/********************************************************************
* Function Prototype Section
********************************************************************/ 
void configure_processor(void);
static void configure_io(void);
static void configure_clock(void);
static void configure_adc_spi_port(void);
static void configure_timer5_peripheral(void);
void configure_display_i2c_port(void);


/********************************************************************
* Function name : configure_processor
*
* Created by : LuisMbedder
*
* Description : Configures Micro I/O and Perihperals
*
* Notes : None
********************************************************************/
void configure_processor(void)
{
   configure_clock();
   configure_io();
   configure_timer5_peripheral();
   configure_adc_spi_port();
   configure_display_i2c_port();

}//configure_processor


/********************************************************************
* Function name : configure_clock
*
* Created by : LuisMbedder
*
* Description : Configures Micro Clock registers
*
* Notes : None
********************************************************************/
static void configure_clock(void)
{
   //configure oscillator control register
   OSCCONbits.CLKLOCK = 1; //If FCKSM1 = 1, clock and PLL selections are locked
   
   //configure clock divider register
   CLKDIVbits.RCDIV = 0b000; //FRC Postscaler Select bits = 8MHz (divide-by-1)
#ifdef FOSC_32MHZ
   CLKDIVbits.CPDIV = 0b00; //32MHz (divide-by-1)
#endif //#ifdef FOSC_32MHZ
#ifdef FOSC_16MHZ
   CLKDIVbits.CPDIV = 0b01; //16MHz (divide-by-2)
#endif //#ifdef FOSC_16MHZ

   //configure oscillator tune register
   OSCTUNbits.TUN = 0b000000; //Center frequency, oscillator is running at factory calibrated frequency
}//configure_clock


/********************************************************************
* Function name : configure_io
*
* Created by : LuisMbedder
*
* Description : Initializes output values, configures open-drain and
*               analog pins, configures I/O direction and configures 
*               peripheral pin select pins.
*
* Notes : None
********************************************************************/
static void configure_io(void)
{
   //initialize Port A output pins
   LATA = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port B output pins
   /*
   Port B pins unused, set to output driving ground
   */
   LATB = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port C output pins
   /*
   //No C0, set to 0
   //LATCbits.LATC1 = 0; //Pin 4 unused, set to output driving ground
   //No C2, set to 0
   //LATCbits.LATC3 = 0; //Pin 5 unused, set to output driving ground
   //No C4, set to 0
   //No C5, set to 0
   //No C6, set to 0
   //No C7, set to 0
   //No C8, set to 0
   //No C9, set to 0
   //No C10, set to 0
   //No C11, set to 0
   //C12 - Pin 49 used as OSCI, set to 0
   LATCbits.LATC13 = 0; //Pin 59 unused, set to output driving ground
   LATCbits.LATC14 = 0; //Pin 60 unused, set to output driving ground
   //C15 - Pin 50 used as OSCO, set to 0
   */
   LATC = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port D output pins
   /*
   LATBbits.LATD0 = 0; //Pin 58 unused, set to output driving ground
   //D1 - Pin 61 controlled by SPI1, set to 0
   //D2 - Pin 62 controlled by SPI1, set to 0
   //D3 - Pin 63 controlled by SPI1, set to 0
   LATBbits.LATD4 = 0; //Pin 66 unused, set to output driving ground
   LATBbits.LATD5 = 0; //Pin 67 unused, set to output driving ground
   LATDbits.LATD6 = 0; //Pin 68 unused, set to output driving ground
   LATDbits.LATD7 = 0; //Pin 69 unused, set to output driving ground
   LATDbits.LATD8 = 0; //Pin 54 unused, set to output driving ground
   LATDbits.LATD9 = 0; //I2C pin, initialized to 0
   LATDbits.LATD10 = 0;//I2C pin, initialized to 0
   LATDbits.LATD11 = 0;//Pin 57 unused, set to output driving ground
   ADC_CS_PIN = SELECT; //LATDbits.LATD12 = 0
   LATDbits.LATD13 = 0; //Pin 65 unused, set to output driving ground
   LATDbits.LATD14 = 0;//Pin 37 unused, set to output driving ground
   LATDbits.LATD14 = 0;//Pin 37 unused, set to output driving ground
   */
   LATD = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port E output pins
   /*
    Port E pins unused, set to output driving ground
   */
   LATE = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port F output pins
   /*
	Port F pins unused, set to output driving ground
   */
   LATF = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port G output pins
   /*
	 Port G pins unused, set to output driving ground  
   */
   LATG = 0b0000000000000000;
          //FEDCBA9876543210


   //configure open-drain pins
   /*
   ODCDbits.ODD9 = 1; //Set pin D9 (SDA) to open-drain configuration
   ODCDbits.ODD10 = 1; //Set pin D10 (SCL) to open-drain configuration
   */
   ODCD = 0b0000011000000000;
          //FEDCBA9876543210

   //configure Port A I/O direction

   TRISA = 0b1111111111111111;
           //FEDCBA9876543210

   //configure Port B I/O direction
   /*
   TRISBbits.TRISB0 = 0; //Pin 20 unused, set to output driving ground
   TRISBbits.TRISB1 = 0; //Pin 19 unused, set to output driving ground
   TRISBbits.TRISB2 = 0; //Pin 18 unused, set to output driving ground
   TRISBbits.TRISB3 = 0; //Pin 17 unused, set to output driving ground
   TRISBbits.TRISB4 = 0; //Pin 16 unused, set to output driving ground
   TRISBbits.TRISB5 = 0; //Pin 15 unused, set to output driving ground
   TRISBbits.TRISB6 = 1; //PGEC, Pin 21 
   TRISBbits.TRISB7 = 1; //PGED, Pin 22 
   TRISBbits.TRISB8 = 0; //Pin 27 unused, set to output driving ground
   TRISBbits.TRISB9 = 0; //Pin 27 unused, set to output driving ground
   TRISBbits.TRISB10 = 0; //Pin 29 unused, set to output driving ground
   TRISBbits.TRISB11 = 0; //Pin 30 unused, set to output driving ground
   TRISBbits.TRISB12 = 0; //Pin 33 unused, set to output driving ground
   TRISBbits.TRISB13 = 0; //Pin 34 unused, set to output driving ground
   TRISBbits.TRISB14 = 0; //Pin 35 unused, set to output driving ground
   TRISBbits.TRISB15 = 0; //Pin 36 unused, set to output driving ground
   */
   TRISB = 0b0000000011000000;
           //FEDCBA9876543210

   //configure Port C I/O direction
   /*
   //No C0, set to 0
   TRISCbits.TRISC1 = 0; //Pin 4 unused, set to output driving ground
   //No C2, set to 0
   TRISCbits.TRISC3 = 0; //Pin 3 unused, set to output driving ground
   //No C4, set to 0
   //No C5, set to 0
   //No C6, set to 0
   //No C7, set to 0
   //No C8, set to 0
   //No C9, set to 0
   //No C10, set to 0
   //No C11, set to 0
   TRISCbits.TRISC12 = 0; //C12 - Pin 49 used as OSCI, set to 0
   TRISCbits.TRISC13 = 0; //Pin 59 unused, set to output driving ground
   TRISCbits.TRISC14 = 0; //Pin 60 unused, set to output driving ground
   TRISCbits.TRISC15 = 0; //C15 - Pin 50 used as OSCO, set to 0
   */
   TRISC = 0b0000000000000000;
           //FEDCBA9876543210

   //configure Port D I/O direction
   /*
   TRISDbits.TRISD0 = 0; //Pin 58 unused, set to output driving ground
   TRISDbits.TRISD1 = 0; //SCLK_XDCR
   TRISDbits.TRISD2 = 0; //MOSI_XDCR
   TRISDbits.TRISD3 = 1; //MISO_XDCR
   TRISDbits.TRISD4 = 0; //Pin 66 unused, set to output driving ground
   TRISDbits.TRISD5 = 0; //Pin 67 unused, set to output driving ground
   TRISDbits.TRISD6 = 0; //Pin 68 unused, set to output driving ground
   TRISDbits.TRISD7 = 0; //Pin 69 unused, set to output driving ground
   TRISDbits.TRISD8 = 0; //Pin 54 unused, set to output driving ground
   TRISDbits.TRISD9 = 0; //I2C
   TRISDbits.TRISD10 = 0; //I2C
   TRISDbits.TRISD11 = 0; //Pin 57 unused, set to output driving ground
   TRISDbits.TRISD12 = 0; //Set pin D12 (ADC_CS_PIN) to output
   TRISDbits.TRISD13 = 0; //Pin 65 unused, set to output driving ground
   TRISDbits.TRISD14 = 0; //Pin 37 unused, set to output driving ground
   TRISDbits.TRISD15 = 0; //Pin   unused, set to output driving ground
   */
   TRISD = 0b0000000000001000;
           //FEDCBA9876543210

   //configure Port E I/O direction
   /*
	Port E pins unused, set to output driving ground
   */
   TRISE = 0b0000000000000000;
           //FEDCBA9876543210

   //configure Port F I/O direction
   /*
	Port F pins unused, set to output driving ground
   */
   TRISF = 0b0000000000000000;
           //FEDCBA9876543210


   //configure Port G I/O direction
   /*
	 Port G pins unused, set to output driving ground
   */
   TRISG = 0b0000000000000000;
           //FEDCBA9876543210


   //configure analog pins
   /*
	Set all analog pins in digital mode, unused 
   */
   AD1PCFGL = 0b1111111111111111;
              //FEDCBA9876543210


   //configure peripheral pin select pins
   __builtin_write_OSCCONL(OSCCON & 0xBF); //clear bit 6 of OSCCONL to unlock Pin Re-map
   //configure input functions
   RPINR20bits.SDI1R = 22; //assign SPI1 Data Input (SDI1) to RP22 (Pin 63)

   //configure output functions
   RPOR12bits.RP24R = 8; //8 represents SPI1 SCK Out (SCK1OUT), assigned to RP24 (Pin 61)
   RPOR11bits.RP23R = 7; //7 represents SPI1 Data Out (SDO1), assigned to RP23 (Pin 62)

   __builtin_write_OSCCONL(OSCCON | 0x40); //set bit 6 of OSCCONL to lock Pin Re-map and prevent
                                           // reconfiguration of peripheral pin select pins until reset
}//configure_io


/********************************************************************
* Function name : configure_adc_spi_port
*
* Created by : LuisMbedder
*
* Description : Configures SPI1 module for communication with the
*               ADC (AD7799)
*
* Notes : None
********************************************************************/
static void configure_adc_spi_port(void) 
{
   //configure SPI1 control register 1
   SPI1CON1bits.DISSCK = 0; //internal SPI clock is enabled
   SPI1CON1bits.DISSDO = 0; //SDO pin is utilized
   SPI1CON1bits.MODE16 = 0; //communication is byte wide (8-bits)
   SPI1CON1bits.SMP = 0; //Input data sampled at middle of data output time
   SPI1CON1bits.CKE = 0; //Serial output data changes on transition from Idle clock state to active clock state (see CKP)
   SPI1CON1bits.SSEN = 0; //!SS pin not used by module; pin controlled by port function
   SPI1CON1bits.CKP = 1; //Idle state for clock is a high level; active state is a low level
   SPI1CON1bits.MSTEN = 1; //Master Mode enabled


   //SPI clock speed, Fsck = FCY / (Primary Prescaler * Secondary Prescaler)
   //Fsck programmed to 250kHz
#ifdef FOSC_32MHZ //FCY = 16MHz
   SPI1CON1bits.SPRE = 0b111; //Secondary Prescale Bits set for 1:1 ratio
   SPI1CON1bits.PPRE = 0b00; //Primary Prescale Bits set for 64:1 ratio
#endif //#ifdef FOSC_32MHZ
#ifdef FOSC_16MHZ //FCY = 8MHz
   SPI1CON1bits.SPRE = 0b110; //Secondary Prescale Bits set for 2:1 ratio
   SPI1CON1bits.PPRE = 0b01; //Primary Prescale Bits set for 16:1 ratio
#endif //#ifdef FOSC_16MHZ

   //configure SPI1 control register 2
   SPI1CON2bits.FRMEN = 0; //Framed support disabled
 
   //Clear the SPIROV bit (SPIxSTAT<6>)
   SPI1STATbits.SPIROV = 0; //Clear the Receive Overflow Flag bit

   //SPI1CON2bits.SPIBEN = 1; //Enhanced Buffer enabled

   //Enable operation by setting the SPIEN bit (SPIxSTAT<15>)
   SPI1STATbits.SPIEN = 1; //Enable SPI1 module   
}//configure_adc_spi_port


/********************************************************************
* Function name : configure_display_i2c_port
*
* Created by : LuisMbedder
*
* Description : Configures the I2C2 module for communication with the
*               7-segment led displays. 
*
* Notes : This function is not declared static so it can be used
*         externally to manually reset the i2c peripheral and bus if
*         an error condition is encountered.
********************************************************************/
void configure_display_i2c_port(void)
{
   //turn off port to allow manual reinitialization of the i2c bus
   I2C1CONbits.I2CEN = 0; //disable I2C module
   LATDbits.LATD9 = 0; //I2C pin, initialized to 0
   LATDbits.LATD10 = 0; //I2C pin, initialized to 0

   U16 i = 0;

   //set the baud rate, FSCL
   //FSCL = FCY / (I2C2BRG + 1 + FCY/10,000,000)
   //set the baud rate, FSCL
   //FSCL = FCY / (I2C2BRG + 1 + FCY/10,000,000)
#ifdef FOSC_32MHZ
#ifdef FSCL1_100KHZ
   I2C1BRG = 157U; //100kHz
#endif //#ifdef FSCL1_100KHZ
#ifdef FSCL_1MHZ
   I2C1BRG = 13U; //1.026MHz
#endif //#ifdef FSCL_1MHZ
#endif //#ifdef FOSC_32MHZ

#ifdef FOSC_16MHZ
#ifdef FSCL1_100KHZ
   I2C1BRG = 18U; //404kHz
#endif //#ifdef FSCL1_100KHZ
#ifdef FSCL_1MHZ
   I2C1BRG = 6U; //1.026MHz
#endif //#ifdef FSCL_1MHZ
#endif //#ifdef FOSC_16MHZ

   //configure the I2C control register 1
   I2C1CONbits.I2CEN = 1; //enables I2C module and configures port pins
   I2C1CONbits.I2CSIDL = 0; //continues operation in idle mode
   I2C1CONbits.IPMIEN = 0; //Intelligent Platform Management Interface is disabled
#ifdef FSCL1_100KHZ
   I2C1CONbits.DISSLW = 0; //Slew rate control enabled
#endif //#ifdef FSCL1_100KHZ
#ifdef FSCL_1MHZ
   I2C1CONbits.DISSLW = 1; //Slew rate control disabled
#endif //#ifdef FSCL_1MHZ

}//configure_display_i2c_port


/********************************************************************
* Function name : configure_timer5_peripheral
*
* Created by : Luis Marquez
*
* Description : Configures the timer5 peripheral.  This is used for
*               monitoring the loop time.
*
* Notes : None
********************************************************************/
static void configure_timer5_peripheral(void)
{
   T5CON = 0x00;//Stops the Timer5 and reset control reg.
   TMR5 = 0x00;//Clear contents of the timer register

   T5CONbits.TCKPS = 0b11;//1:256 prescale
   T5CONbits.TON = 1;//Start Timer5
}






