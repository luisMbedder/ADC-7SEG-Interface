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
   /*
   LATAbits.LATA14 = 0; //I2C pin, initialized to 0
   LATAbits.LATA15 = 0; //I2C pin, initialized to 0
   */
   LATA = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port B output pins
   /*
   LATBbits.LATB0 = 0; //Pin 20 used as analog input, set to 0
   LATBbits.LATB1 = 0; //Pin 19 used as analog input, set to 0
   LATBbits.LATB2 = 0; //Pin 18 used as analog input, set to 0
   LATBbits.LATB3 = 0; //Pin 17 unused, set to output driving ground
   LATBbits.LATB4 = 0; //Pin 16 unused, set to output driving ground
   LATBbits.LATB5 = 0; //Pin 15 unused, set to output driving ground 
   LATBbits.LATB6 = 0; //PGEC, Pin 21 unused, set to 0
   LATBbits.LATB7 = 0; //PGED, Pin 22 unused, set to 0
   LATBbits.LATB8 = 0; //Pin 27 unused, set to output driving ground
   HEARTBEAT_LED_PIN = HEARTBEAT_LED_OFF; //LATBbits.LATB9 = 0
   LATBbits.LATB10 = 0; //Pin 29 unused, set to output driving ground
   LATBbits.LATB11 = 0; //Pin 30 unused, set to output driving ground
   LATBbits.LATB12 = 0; //Pin 33 unused, set to output driving ground
   LATBbits.LATB13 = 0; //Pin 34 unused, set to output driving ground
   LATBbits.LATB14 = 0; //Pin 35 unused, set to output driving ground
   LATBbits.LATB15 = 0; //Pin 36 unused, set to output driving ground
   */
   LATB = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port C output pins
   /*
   //No C0, set to 0
   LATCH_TENSION_OUT_DAC_PIN_LEFT = RESET_LATCH; //LATCbits.LATC1 = 1
   //No C2, set to 0
   LATCH_TENSION_OUT_DAC_PIN_RIGHT = RESET_LATCH; //LATCbits.LATC3 = 1
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
   LATC = 0b0000000000001010;
          //FEDCBA9876543210

   //initialize Port D output pins
   /*
   LATBbits.LATD0 = 0; //Pin 58 unused, set to output driving ground
   //D1 - Pin 61 controlled by SPI1, set to 0
   //D2 - Pin 62 controlled by SPI1, set to 0
   //D3 - Pin 63 controlled by SPI1, set to 0
   LATBbits.LATD4 = 0; //Pin 66 unused, set to output driving ground, interrupt input for adc
   LATBbits.LATD5 = 0; //Pin 67 unused, set to output driving ground
   LATDbits.LATD6 = 0; //Pin 68 unused, set to output driving ground
   LATDbits.LATD7 = 0; //Pin 69 unused, set to output driving ground
   LATDbits.LATD8 = 0; //Pin 54 unused, set to output driving ground
   LATDbits.LATD9 = 0; //I2C pin, initialized to 0
   LATDbits.LATD10 = 0;//I2C pin, initialized to 0
   LATDbits.LATD11 = 0;//Pin 57 unused, set to output driving ground
   XDCR_ADC_CS_PIN = SELECT; //LATDbits.LATD12 = 0
   LATDbits.LATD13 = 0; //Pin 65 unused, set to output driving ground
   LATDbits.LATD14 = 0;//Pin 37 unused, set to output driving ground
   TENSION_OUT_DAC_CS_PIN_RIGHT = DESELECT; //LATDbits.LATD15=1
   */
   LATD = 0b1000000000000000;
          //FEDCBA9876543210

   //initialize Port E output pins
   /*
   //E0 - Pin 76 used as input, set to 0
   //E1 - Pin 77 used as input, set to 0
   //E2 - Pin 78 used as input, set to 0
   //E3 - Pin 79 used as input, set to 0
   //LATEbits.LATE4 = 0;//Pin 80 unused, set to output driving ground
   CONTROL_OUT_ENABLE_PIN = DISABLE_CONTROL_OUT; //LATEbits.LATE5 = 1
   //LATEbits.LATE6 = 0;//I2C pin, initialized to 0
   //LATEbits.LATE7 = 0;//I2C pin, initialized to 0
   //LATEbits.LATE8 = 0;//Pin 13 unused, set to output driving ground
   //LATEbits.LATE9 = 0; //Pin 14 unused, set to 0
   //No E10, set to 0
   //No E11, set to 0
   //No E12, set to 0
   //No E13, set to 0
   //No E14, set to 0
   //No E15, set to 0
   */
   LATE = 0b0000000000100000;
          //FEDCBA9876543210

   //initialize Port F output pins
   /*
   //LATFbits.LATF0 = 0; //Pin 72 unused, set to output driving ground
   //LATFbits.LATF1 = 0; //Pin 73 unused, set to output driving ground
   //LATFbits.LATF2 = 0; //Pin 42 unused, set to output driving ground
   //LATFbits.LATF3 = 0; //Pin 41 unused, set to output driving ground
   //Pin 39 controlled by SPI3, set to 0
   //Pin 40 controlled by SPI3, set to 0
   //No F6, set to 0
   //No F7, set to 0
   //LATFbits.LATF8 = 0; //Pin 43 unused, set to output driving ground
   //No F9, set to 0
   //No F10, set to 0
   //No F11, set to 0
   //No F12, set to 0
   //No F13, set to 0
   //No F14, set to 0
   //No F15, set to 0
   */
   LATF = 0b0000000000000000;
          //FEDCBA9876543210

   //initialize Port G output pins
   /*
   //LATGbits.LATG0 = 0; //Pin 75 unused, set to output driving ground
   //LATGbits.LATG1 = 0; //Pin 74 unused, set to output driving ground
   //LATGbits.LATG2 = 0; //Pin 47 unused, set to output driving ground
   //LATGbits.LATG3 = 0; //Pin 46 unused, set to output driving ground
   //No G4, set to 0
   //No G5, set to 0
   //G6 - Pin 6 controlled by SPI2, set to 0
   //G7 - Pin 7 controlled by SPI2, set to 0
   //TENSION_OUT_DAC_CS_PIN_LEFT = DESELECT; //LATGbits.LATE8=1
   BUFFER_ENABLE_PIN = BUFFER_DISABLED; //LATGbits.LATG9 = 1
   //No G10, set to 0
   //No G11, set to 0
   //No G12, set to 0
   //No G13, set to 0
   //No G14, set to 0
   //No G15, set to 0   
   */
   LATG = 0b0000001100000000;
          //FEDCBA9876543210


   //configure open-drain pins
   /*
   ODCAbits.ODA14 = 1; //Set pin A14 (SCL) to open-drain configuration
   ODCAbits.ODA15 = 1; //Set pin A15 (SDA) to open-drain configuration
   */
   ODCA = 0b1100000000000000;
          //FEDCBA9876543210

   //configure open-drain pins
   /*
   ODCDbits.ODD4 = 1; //Set pin D4 (EXC_5V_10V_SEL_PIN) to open-drain configuration
   ODCDbits.ODD5 = 1; //Set pin D5 (EXC_ON_OFF_PIN) to open-drain configuration
   ODCDbits.ODD9 = 1; //Set pin D9 (SDA) to open-drain configuration
   ODCDbits.ODD10 = 1; //Set pin D10 (SCL) to open-drain configuration
   */
   ODCD = 0b0000011000110000;
          //FEDCBA9876543210

   //configure open-drain pins
   /*
   ODCEbits.ODE6 = 1; //Set pin E6 (SCL) to open-drain configuration
   ODCEbits.ODE7 = 1; //Set pin E7 (SDA) to open-drain configuration
   */
   ODCE = 0b0000000011000000;
          //FEDCBA9876543210


   //configure Port A I/O direction
   /*
   TRISAbits.TRISA14 = 0; //SCL
   TRISAbits.TRISA15 = 0; //SDA
   */
   TRISA = 0b0011111111111111;
           //FEDCBA9876543210

   //configure Port B I/O direction
   /*
   TRISBbits.TRISB0 = 0; //Pin 20 unused, set to output driving ground
   TRISBbits.TRISB1 = 0; //Pin 19 unused, set to output driving ground
   TRISBbits.TRISB2 = 0; //Pin 18 unused, set to output driving ground
   TRISBbits.TRISB3 = 0; //Pin 17 unused, set to output driving ground
   TRISBbits.TRISB4 = 0; //Pin 16 unused, set to output driving ground
   TRISBbits.TRISB5 = 0; //Pin 15 unused, set to output driving ground
   TRISBbits.TRISB6 = 1; //PGEC, Pin 21 unused
   TRISBbits.TRISB7 = 1; //PGED, Pin 22 unused
   TRISBbits.TRISB8 = 0; //Pin 27 unused, set to output driving ground
   TRISBbits.TRISB9 = 0; //Set pin B9 (HEARTBEAT_LED_PIN) to output
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
   TRISCbits.TRISC1 = 0; //Set pin C1 (LATCH_TENSION_OUT_DAC_PIN_LEFT) to output
   //No C2, set to 0
   TRISCbits.TRISC3 = 0; //Set pin C3 (LATCH_TENSION_OUT_DAC_PIN_RIGHT) to output
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
   TRISDbits.TRISD4 = 1; //Pin 66 unused, set to output driving ground, adc interrupt input
   TRISDbits.TRISD5 = 0; //Pin 67 unused, set to output driving ground
   TRISDbits.TRISD6 = 0; //Pin 68 unused, set to output driving ground
   TRISDbits.TRISD7 = 0; //Pin 69 unused, set to output driving ground
   TRISDbits.TRISD8 = 0; //Pin 54 unused, set to output driving ground
   TRISDbits.TRISD9 = 0; //I2C SDA_LT_RT
   TRISDbits.TRISD10 = 0; //I2C SCL_LT_RT
   TRISDbits.TRISD11 = 0; //Pin 57 unused, set to output driving ground
   TRISDbits.TRISD12 = 0; //Set pin D12 (XDCR_ADC_CS_PIN) to output
   TRISDbits.TRISD13 = 0; //Pin 65 unused, set to output driving ground
   TRISDbits.TRISD14 = 0; //Pin 37 unused, set to output driving ground
   TRISDbits.TRISD15 = 0; //Set pin D15 (TENSION_OUT_DAC_CS_PIN_RIGHT) to output
   */
   TRISD = 0b0000000000001000;
           //FEDCBA9876543210

   //configure Port E I/O direction
   /*
   TRISEbits.TRISE0 = 1; //Set pin E0 (MENU_BUTTON) to input
   TRISEbits.TRISE1 = 1; //Set pin E1 (UP_BUTTON) to input
   TRISEbits.TRISE2 = 1; //Set pin E2 (DOWN_BUTTON) to input
   TRISEbits.TRISE3 = 1; //Set pin E3 (SET_BUTTON) to input
   TRISEbits.TRISE4 = 0; //Pin 80 unused, set to output driving ground
   TRISEbits.TRISE5 = 0; //Set pin E5 (CONTROL_OUT_ENABLE_PIN) to output
   TRISEbits.TRISE6 = 0; //Set pin E6 (SCL_TOT) to output
   TRISEbits.TRISE7 = 0; //Set pin E7 (SDA_TOT) to output
   TRISEbits.TRISE8 = 0; //Pin 13 unused, set to output driving ground
   TRISEbits.TRISE9 = 0; //Pin 14 unused, set to output driving ground
   //No E10, set to 0
   //No E11, set to 0
   //No E12, set to 0
   //No E13, set to 0
   //No E14, set to 0
   //No E15, set to 0
   */
   TRISE = 0b0000000000001111;
           //FEDCBA9876543210

   //configure Port F I/O direction
   /*
   TRISFbits.TRISF0 = 0; //Pin 72 unused, set to output driving ground
   TRISFbits.TRISF1 = 0; //Pin 73 unused, set to output driving ground
   TRISFbits.TRISF2 = 0; //Pin 74 unused, set to output driving ground
   TRISFbits.TRISF3 = 0; //Pin 41 unused, set to output driving ground //temp use as INT1 input
   TRISFbits.TRISF4 = 0; //SCLK_RIGHT
   TRISFbits.TRISF5 = 0; //MOSI_RIGHT
   //No F6, set to 0
   //No F7, set to 0
   TRISFbits.TRISF8 = 0; //Pin 43 unused, set to output driving ground
   //No F9, set to 0
   //No F10, set to 0
   //No F11, set to 0
   //No F12, set to 0
   //No F13, set to 0
   //No F14, set to 0
   //No F15, set to 0
   */
   TRISF = 0b0000000000001000;
           //FEDCBA9876543210


   //configure Port G I/O direction
   /*
   TRISGbits.TRISG0 = 0; //Pin 75 unused, set to output driving ground
   TRISGbits.TRISG1 = 0; //Pin 74 unused, set to output driving ground
   TRISGbits.TRISG2 = 0; //Pin 47 unused, set to output driving ground
   TRISGbits.TRISG3 = 0; //Pin 46 unused, set to output driving ground
   //No G4, set to 0
   //No G5, set to 0
   TRISGbits.TRISG6 = 0; //MOSI_LEFT
   TRISGbits.TRISG7 = 0; //SCLK_LEFT
   TRISGbits.TRISG8 = 0; //Set pin G8 (TENSION_OUT_DAC_CS_PIN_LEFT) to output
   TRISGbits.TRISG9 = 0; //Set pin G9 (BUFFER_ENABLE_PIN) to output
   //No G10, set to 0
   //No G11, set to 0
   //No G12, set to 0
   //No G13, set to 0
   //No G14, set to 0
   //No G15, set to 0 
   */
   TRISG = 0b0000000000000000;
           //FEDCBA9876543210


   //configure analog pins
   /*
   AD1PCFGLbits.PCFG0 = 1; //Configure AN0 (Pin 20) in Digital mode, unused
   AD1PCFGLbits.PCFG1 = 1; //Configure AN1 (Pin 19) in Digital mode, unused
   AD1PCFGLbits.PCFG2 = 1; //Configure AN2 (Pin 18) in Digital mode, unused
   AD1PCFGLbits.PCFG3 = 1; //Configure AN3 (Pin 17) in Digital mode, unused
   AD1PCFGLbits.PCFG4 = 1; //Configure AN4 (Pin 16) in Digital mode, unused
   AD1PCFGLbits.PCFG5 = 1; //Configure AN5 (Pin 15) in Digital mode, unused
   AD1PCFGLbits.PCFG6 = 1; //Configure AN6 (Pin 21) in Digital mode, PGEC / unused
   AD1PCFGLbits.PCFG7 = 1; //Configure AN7 (Pin 22) in Digital mode, PGED / unused
   AD1PCFGLbits.PCFG8 = 1; //Configure AN8 (Pin 27) in Digital mode, unused
   AD1PCFGLbits.PCFG9 = 1; //Configure AN9 (Pin 28) in Digital mode, used as HEARTBEAT_LED_PIN
   AD1PCFGLbits.PCFG10 = 1; //Configure AN10 (Pin 29) in Digital mode, unused
   AD1PCFGLbits.PCFG11 = 1; //Configure AN11 (Pin 30) in Digital mode, unused
   AD1PCFGLbits.PCFG12 = 1; //Configure AN12 (Pin 33) in Digital mode, unused 
   AD1PCFGLbits.PCFG13 = 1; //Configure AN13 (Pin 34) in Digital mode, unused 
   AD1PCFGLbits.PCFG14 = 1; //Configure AN14 (Pin 35) in Digital mode, unused 
   AD1PCFGLbits.PCFG15 = 1; //Configure AN15 (Pin 36) in Digital mode, unused 
   */
   AD1PCFGL = 0b1111111111111111;
              //FEDCBA9876543210


   //configure peripheral pin select pins
   __builtin_write_OSCCONL(OSCCON & 0xBF); //clear bit 6 of OSCCONL to unlock Pin Re-map
   //configure input functions
   RPINR20bits.SDI1R = 22; //assign SPI1 Data Input (SDI1) to RP22 (Pin 63)
   RPINR0bits.INT1R = 25;//assign to pin 66 RP25   assign external interrupt 1 to RP25 (Pin66),25 in production board
 //  RPINR28bits.SDI3R = 3; //assign SPI3 Data Input (SDI3) to RP3 (Pin 56)
//   RPINR18bits.U1RXR = 15; //assign UART1 Receive (U1RX) to RP15 (Pin 43)
 //  RPINR19bits.U2RXR = 10; //assign UART2 Receive (U2RX) to RP10 (Pin 39)

   //configure output functions
   RPOR12bits.RP24R = 8; //8 represents SPI1 SCK Out (SCK1OUT), assigned to RP24 (Pin 61)
   RPOR11bits.RP23R = 7; //7 represents SPI1 Data Out (SDO1), assigned to RP23 (Pin 62)
   RPOR13bits.RP26R = 11; //11 represents SPI2 SCK Out (SCK2OUT), assigned to RP26 (Pin 7)
   RPOR10bits.RP21R = 10; //10 represents SPI2 Data Out (SDO2), assigned to RP21 (Pin 6)
   RPOR5bits.RP10R = 33; //33 represents SPI3 SCK Out (SCK3OUT), assigned to RP10 (Pin 39)
   RPOR8bits.RP17R = 32; //32 represents SPI3 Data Out (SDO3), assigned to RP12 (Pin 40)
  // RPOR15bits.RP30R = 3; //3 represents UART1 Transmit (U1TX), assigned to RP30 (Pin 42)
 //  RPOR7bits.RP14R = 5; //5 represents UART2 Transmit (U2TX), assinged to RP14 (Pin 35)

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

/*
   //SPI clock speed, Fsck = FCY / (Primary Prescaler * Secondary Prescaler)
   //Fsck programmed to 500kHz
#ifdef FOSC_32MHZ
   SPI1CON1bits.SPRE = 0b110; //Secondary Prescale Bits set for 2:1 ratio
   SPI1CON1bits.PPRE = 0b01; //Primary Prescale Bits set for 16:1 ratio
#endif //#ifdef FOSC_32MHZ
#ifdef FOSC_16MHZ
   SPI1CON1bits.SPRE = 0b111; //Secondary Prescale Bits set for 1:1 ratio
   SPI1CON1bits.PPRE = 0b00; //Primary Prescale Bits set for 64:1 ratio
#endif //#ifdef FOSC_16MHZ
*/
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
   I2C2CONbits.I2CEN = 0; //disable I2C module
   LATAbits.LATA14 = 0; //I2C pin, initialized to 0
   LATAbits.LATA15 = 0; //I2C pin, initialized to 0

   U16 i = 0;

   ///////////////////////////////////
   //manually reset the eeprom device
   ///////////////////////////////////
   SCL_PIN = 0;
   SDA_PIN = 0;
   delay_10us(10);

   SCL_PIN = 0;
   SDA_PIN = 1;
   delay_10us(2);

   //transmit START command
   SCL_PIN = 1;
   delay_10us(1);
   SDA_PIN = 0;
   delay_10us(1);
   SCL_PIN = 0;
   delay_10us(1);

   //transmit 9 clocks
   SDA_PIN = 1;
   delay_10us(1);
   
   for (i = 0 ; i < 9; i++)
   {
      SCL_PIN = 1;
      delay_10us(2);
      SCL_PIN = 0;
      delay_10us(2);
   }

   //transmit START command
   SCL_PIN = 1;
   delay_10us(1);
   SDA_PIN = 0;
   delay_10us(1);
   SCL_PIN = 0;
   delay_10us(1);

   //transmit stop command
   SCL_PIN = 1;
   delay_10us(1);
   SDA_PIN = 1;
   delay_10us(1);
   SCL_PIN = 0;
   delay_10us(1);

   //end of device reset
   ///////////////////////////////////

   //set the baud rate, FSCL
   //FSCL = FCY / (I2C2BRG + 1 + FCY/10,000,000)
#ifdef FOSC_32MHZ
#ifdef FSCL_400KHZ
   I2C2BRG = 37U; //404kHz
#endif //#ifdef FSCL_400KHZ
#ifdef FSCL_1MHZ
   I2C2BRG = 13U; //1.026MHz
#endif //#ifdef FSCL_1MHZ
#endif //#ifdef FOSC_32MHZ

#ifdef FOSC_16MHZ
#ifdef FSCL_400KHZ
   I2C2BRG = 18U; //404kHz
#endif //#ifdef FSCL_400KHZ
#ifdef FSCL_1MHZ
   I2C2BRG = 6U; //1.026MHz
#endif //#ifdef FSCL_1MHZ
#endif //#ifdef FOSC_16MHZ

   //configure the I2C control register 1
   I2C2CONbits.I2CEN = 1; //enables I2C module and configures port pins
   I2C2CONbits.I2CSIDL = 0; //continues operation in idle mode
   I2C2CONbits.IPMIEN = 0; //Intelligent Platform Management Interface is disabled
#ifdef FSCL_400KHZ
   I2C2CONbits.DISSLW = 0; //Slew rate control enabled
#endif //#ifdef FSCL_400KHZ   
#ifdef FSCL_1MHZ
   I2C2CONbits.DISSLW = 1; //Slew rate control disabled
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






