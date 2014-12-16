/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: 7seg_display.c
*
* Description: This module contains the functions to communicate
*			   to the 4 digit 7 segment displays using the I2C interface. 
*
* The 7-segment HT16K33 driver contains 16 bytes of display memory, mapped to 16 row x 8 column 
* output. Each column can drive an individual 7-segment display; only 0-4 are used for this device.
* Each row drives a segment of the display; only rows 0-6 are used. A 16-bit value is written
* to configure the output of each digit. 
*
*          A               The 16-bit value is designated as follows: 
*        -----             PGFEDCBAxxxxxxxx
*     F |     | B          A-G = individual led segments
*       |     |            P=decimal point
*        --G--             x=dont care
*     E |     | C
*       |     |            For example, to display the number 3 we need
*        -----             to light up(bring high) segments A,B,C,D, and G. The 16-bit field
*          D     		   would look like this: 0100 1111 0000 0000 =0x4F
*									
*		                   Mapping to the display address memory:
*							0x00: digit 0(left most digit) 
*							0x02: digit 1
*							0x04: colon ":"  
*							0x06: digit 3
*							0x08: digit 4      
*
* Known Issues: None
********************************************************************/

#include "p24FJ128GB108.h"
#include "adc_7seg.h"
#include "7seg_display.h"
#include "timer.h"
#include <stdio.h>

I2C_LOW_LEVEL_ERROR_T init_display(void);
static I2C_LOW_LEVEL_ERROR_T MasterWriteI2C1(unsigned char data_out);
static I2C_LOW_LEVEL_ERROR_T IdleI2C1(void);
static I2C_LOW_LEVEL_ERROR_T StartI2C1(void);
static I2C_LOW_LEVEL_ERROR_T StopI2C1(void);
void update_display(float current_voltage);
I2C_LOW_LEVEL_ERROR_T I2C1_WriteByte(U8 busAddr, unsigned char data_out);
I2C_LOW_LEVEL_ERROR_T I2C1_WriteDisplay(U8 busAddr, U16 const* data_out_p);

#define HT16K33 0xE0// I2C bus address for Ht16K33 backpack
#define HT16K33_ON 0x21 // turn device oscillator on
#define HT16K33_STANDBY 0x20 // turn device oscillator off
#define HT16K33_DISPLAYON 0x81 // turn on output pins
#define HT16K33_DISPLAYOFF 0x80 // turn off output pins
#define HT16K33_BLINKON 0x85 // blink rate 1 Hz (-2 for 2 Hz)
#define HT16K33_BLINKOFF 0x81 // same as display on
#define HT16K33_MAX 0xEF // max brightness

//look-up table for 7 segment display numbers
static const U8 numbertable[] = {
0x3F, /* 0 */
0x06, /* 1 */
0x5B, /* 2 */
0x4F, /* 3 */
0x66, /* 4 */
0x6D, /* 5 */
0x7D, /* 6 */
0x07, /* 7 */
0x7F, /* 8 */
0x6F, /* 9 */
0x77, /* a */
0x7C, /* b */
0x39, /* C */
0x5E, /* d */
0x79, /* E */
0x71, /* F */
};

/********************************************************************
* Function name : init_display
*
* Created by : LuisMbedder
*
* Description : Initialzes the adafruit 7 segment display
*
* Return Value:   I2C error code
********************************************************************/
I2C_LOW_LEVEL_ERROR_T init_display(void)
{
    I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;
    static U16 clear_display[8]={0x00};
//static U16 clear_display[8]={0x0206, 0x00,0xDB,0x66};//1,2,colon off,3,4
	I2C1_WriteByte(HT16K33,HT16K33_ON); // turn on device oscillator
	I2C1_WriteByte(HT16K33,HT16K33_DISPLAYON); // turn on display, no blink
	I2C1_WriteByte(HT16K33,HT16K33_MAX); // set max brightness
    return_error = I2C1_WriteDisplay(HT16K33,clear_display);  //clear display memory
	if (return_error != NO_I2C_ERROR)
    {
         return return_error;
    }

    return return_error;
}



/********************************************************************
* Function name : I2C1_Start
*
* U8 busAddr : I2C slave address
* Created by : LuisMbedder
*
* Description : Starts an I2C transmission  
*
* Return Value:   I2C error code
********************************************************************/
static I2C_LOW_LEVEL_ERROR_T I2C1_Start(U8 busAddr){

I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

  	return_error = IdleI2C1(); //wait until I2C Bus is Inactive
   if (return_error != NO_I2C_ERROR)
   {
      return return_error;
   }

	return_error = StartI2C1(); //transmit START command
   	if (return_error != NO_I2C_ERROR)
   	{
      return return_error;
   	}

   	//transmit bus address
   	return_error = MasterWriteI2C1(busAddr);
   	if (return_error != NO_I2C_ERROR)
   	{
      return return_error;
   	}
	return return_error;
}

/********************************************************************
* Function name : I2C1_Stop
*
* U8 busAddr : I2C slave address
* Created by : LuisMbedder
*
* Description : Stops an I2C transmission  
*
* Return Value:   I2C error code
********************************************************************/
static I2C_LOW_LEVEL_ERROR_T I2C1_Stop(void){

    I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;
	return_error = StopI2C1(); //transmit stop command
    return return_error;
}

/********************************************************************
* Function name : I2C1_WriteByte
*
* busAddr : 8-bit I2C slave address
* data_out : data byte to be sent
*
* Created by : LuisMbedder
*
* Description : Transmits a single byte to the external I2C display
*               device.
*
* Return Value:   I2C error code
********************************************************************/
I2C_LOW_LEVEL_ERROR_T I2C1_WriteByte(U8 busAddr, unsigned char data_out){

    I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

	return_error = I2C1_Start(busAddr); //send start command
	return_error = MasterWriteI2C1(data_out);//send data
	return_error = I2C1_Stop(); //release i2c bus
    return return_error;
}

/************************************************************************
* Function Name: I2C1_WriteDisplay
*
* busAddr : 8-bit I2C slave address
* data_out_p : pointer to U16 data to write to the display
*
* Description : writes 16 bytes of data to the display starting
*				at display_RAM_address. This address auto increments
*				after each succesive write and wraps after 16 writes(1 byte per write). 

* Return Value:   I2C error code
*************************************************************************/
I2C_LOW_LEVEL_ERROR_T I2C1_WriteDisplay(U8 busAddr, U16 const* data_out_p){

  I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;
  U8 display_RAM_address = 0;
  U8 data_byte = 0;
  U8 i=0;

  return_error = I2C1_Start(busAddr);//transmit START command
  display_RAM_address = (U8) (((*data_out_p >> 8) & 0x00FF));//starting digit address
  //16 bytes is the max before display address wraps back to 0x00
  for(i=0;i<8;i++)
  {
      data_byte = (U8) (*data_out_p & 0x00FF);
      return_error = MasterWriteI2C1(display_RAM_address);
 	  return_error = MasterWriteI2C1(data_byte);

      if (return_error != NO_I2C_ERROR)
      {
         return return_error;
      }
      data_out_p++;
   }

   return_error = I2C1_Stop();//transmit stop command

   return return_error;

}

/************************************************************************
*    Function Name:  MasterWriteI2C1
*    Description:    This routine is used to write a byte to the I2C bus.
*                    The input parameter data_out is written to the
*                    I2CTRN register. If IWCOL bit is set,write collision
*                    has occured and -1 is returned, else 0 is returned.
*    Parameters:     unsigned char : data_out
*    Return Value:   I2C error code
*************************************************************************/
static I2C_LOW_LEVEL_ERROR_T MasterWriteI2C1(unsigned char data_out)
{
   I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;
   I2C1TRN = data_out; //write byte to I2C transmit register
   if(I2C1STATbits.IWCOL) //collision occurred, communication stopped
   {
      return_error = StopI2C1();
      if (return_error != NO_I2C_ERROR)
      {
         return return_error;
      }
      return_error = IdleI2C1();
      if (return_error != NO_I2C_ERROR)
      {
         return return_error;
      }
      return I2C_COLLISION; //collision was detected, communication stopped
   }
   else
   {
      return_error = IdleI2C1(); //wait till data is transmitted as indicated by TRSTAT
      if (return_error != NO_I2C_ERROR)
      {
         return return_error;
      }
      if(I2C1STATbits.ACKSTAT) //1 = NACK was detected last
      {
         return_error = StopI2C1();
         if (return_error != NO_I2C_ERROR)
         {
            return return_error;
         }
         return_error = IdleI2C1();
         if (return_error != NO_I2C_ERROR)
         {
            return return_error;
         }
         return I2C_NACK; //NACK was detected, communication stopped
      }
      else
      {
         return NO_I2C_ERROR; //successful write
      }
   }
}//MasterWriteI2C1


/********************************************************************
* Function name : update_display
*
* current_voltage : latest voltage conversion to be sent to 7 segment display
*
* Created by : LuisMbedder
*
* Description : This function builds the display data buffer. Each element
*				in the buffer array coressponds to a display digit.
*				 
* Notes : 
********************************************************************/
void update_display(float current_voltage){

     int i = 0;
	 int k=0;
	 int blank_spaces=0;
     char buffer[10];
	 U16 displayBuffer[8]={0};
 
	 if(current_voltage<.1){
		blank_spaces=2;
	 }
     else if(current_voltage<10.0){
		blank_spaces=1;
	 }
	 else{
		blank_spaces=0;
	 }
     sprintf(buffer, "%05.2f", (double)current_voltage);
	/*build display buffer
		displayBuffer[0]=digit 0
		displayBuffer[1]=digit 1
		displayBuffer[2]=colon
		displayBuffer[3]=digit 2
		displayBuffer[4]=digit 3
	*/
     for (i = 0; i < 5; i++){
		if(buffer[i]=='.'){
			displayBuffer[k-1]=displayBuffer[k-1]|DECIMAL_POINT;//add decimal to previous digit
			continue;//go to next index
		}
		if(k==2){//colon index
			displayBuffer[k]=0x00;//set colon off
			k++;
		}
		
		int j =buffer[i]-0x30;//convert ascii to decimal
		displayBuffer[k]=numbertable[j];
		k++;//
		
      }
	 	
	 for(i=0;i<blank_spaces;i++){
		displayBuffer[i]=displayBuffer[i]&0x80; //keep decimal point if its there
	 }

	 I2C1_WriteDisplay(HT16K33,displayBuffer);  //write data to display
	 phase++;
}


/************************************************************************
*    Function Name:  IdleI2C1
*    Description:    This routine generates wait condition until I2C2
*                    bus is Idle.
*    Parameters:     void
*    Return Value:   I2C error code
*************************************************************************/
static I2C_LOW_LEVEL_ERROR_T IdleI2C1(void)
{
   U16 idle_start = read_ms_count();
   /* Wait until I2C Bus is Inactive */
   while(I2C1CONbits.SEN || I2C1CONbits.RSEN || I2C1CONbits.PEN || I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1STATbits.TRSTAT)
   {
//      ClrWdt();
      if (timeout(idle_start, DISPLAY_TIMEOUT_MS))
         return I2C_TIMEOUT;
   }
   return NO_I2C_ERROR;
}//IdleI2C1

/*********************************************************************
*    Function Name:  StartI2C1
*    Description:    This routine generates Start condition
*                    during master mode.
*    Parameters:     void
*    Return Value:   I2C error code
*********************************************************************/
static I2C_LOW_LEVEL_ERROR_T StartI2C1(void)
{
   I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

   I2C1CONbits.SEN = 1;   /* initiate Start on SDA and SCL pins */

   return_error = IdleI2C1();

   return return_error;
}//StartI2C1

/*********************************************************************
*    Function Name:  StopI2C1
*    Description:    This routine generates Stop condition
*                    during master mode.
*    Parameters:     void
*    Return Value:   I2C error code
*********************************************************************/
static I2C_LOW_LEVEL_ERROR_T StopI2C1(void)
{
   I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

   I2C1CONbits.PEN = 1;   /* initiate Stop on SDA and SCL pins */

   return_error = IdleI2C1();

   return return_error;
}//StopI2C1

