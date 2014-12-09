/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: 7seg_display.c
*
* Description: This module contains the functions to communicate
*			   to the 4 digit 7 segment displays using the I2C interface. 
*
* Known Issues: None
*
********************************************************************/

#include "p24FJ128GB108.h"
#include "adc_7seg.h"
#include "7seg_display.h"

static I2C_LOW_LEVEL_ERROR_T MasterWriteI2C1(unsigned char data_out);
static I2C_LOW_LEVEL_ERROR_T IdleI2C1(void);
static I2C_LOW_LEVEL_ERROR_T MasterReadI2C1(U8 * const data_p);
static I2C_LOW_LEVEL_ERROR_T StartI2C1(void);
static I2C_LOW_LEVEL_ERROR_T StopI2C1(void);
static U16 i2c_busy(void);
void I2C_WriteByte(U8 busAddr, unsigned char data_out);
void SS_Init();
void I2C1_WriteDisplay(U8 busAddr, U16 const* data_out_p,const U8 num_of_bytes);

#define HT16K33 0xE0// I2C bus address for Ht16K33 backpack
#define HT16K33_ON 0x21 // turn device oscillator on
#define HT16K33_STANDBY 0x20 // turn device oscillator off
#define HT16K33_DISPLAYON 0x81 // turn on output pins
#define HT16K33_DISPLAYOFF 0x80 // turn off output pins
#define HT16K33_BLINKON 0x85 // blink rate 1 Hz (-2 for 2 Hz)
#define HT16K33_BLINKOFF 0x81 // same as display on
#define HT16K33_DIM 0xE0 // add level (15=max) to byte

void SS_Init()
{

//static U8 display_data[6]={0x00, 0x71, 0x3E, 0x00, 0x79,0x38};
static U16 display_data[6]={0x06, 0x5B, 0x00, 0x4F, 0x66};//1,2,colon off,3,4
//if (!i2c1_busy())
	I2C1_WriteByte(HT16K33,HT16K33_ON); // turn on device oscillator
    delay_10us(10);
	I2C1_WriteByte(HT16K33,HT16K33_DISPLAYON); // turn on display, no blink
	delay_10us(10);
	I2C1_WriteByte(HT16K33,HT16K33_DIM+15 ); // set max brightness
	delay_10us(10);
   I2C1_WriteDisplay(HT16K33,display_data,6);	

}

static U16 i2c1_busy(void)
{
   I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

   return_error = IdleI2C1(); //wait until I2C Bus is Inactive
   if (return_error != NO_I2C_ERROR)
   {
      return TRUE;
   }

   return_error = StartI2C1(); //transmit START command
   if (return_error != NO_I2C_ERROR)
   {
      return TRUE;
   }

   //transmit control byte to write data
   return_error = MasterWriteI2C1(HT16K33);
   if (return_error == NO_I2C_ERROR) //display acknowledged write command successfully
   {
      return_error = StopI2C1(); //transmit stop command
      if (return_error == NO_I2C_ERROR)
         return FALSE;
      else
         return TRUE;
   }
   else //display is busy
   {
      return_error = StopI2C1(); //transmit stop command
      return TRUE;
   }
}//display_busy


///i2c START FUNCTION///
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

}
//stop function///
static I2C_LOW_LEVEL_ERROR_T I2C1_Stop(){
I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

	return_error = StopI2C1(); //transmit stop command
}


void I2C1_WriteByte(U8 busAddr, unsigned char data_out){

I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

I2C1_Start(busAddr);
MasterWriteI2C1(data_out);
I2C1_Stop();

}

/************************************************************************
* Function Name: I2C1_WriteDisplay
*
* Created by : LuisMbedder
*
* Description : function to write data to the 7 segment display
*               This routine is used to write a byte to the I2C bus.
*                    The input parameter data_out is written to the
*                    I2CTRN register. If IWCOL bit is set,write collision
*                    has occured and -1 is returned, else 0 is returned.
*    Parameters:     unsigned char : data_out
*    Return Value:   char
*************************************************************************/
void I2C1_WriteDisplay(U8 busAddr, U16 const* data_out_p,const U8 num_of_bytes){

I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;
U8 transmission_count = 0;
 U8 address_byte = 0;
   U8 data_byte = 0;

I2C1_Start(busAddr);
while(transmission_count < num_of_bytes)
   {
//	U16 data =*data_out_p&0xff;
//		 data=*data_out_p>>8;
address_byte = (U8) (((*data_out_p >> 8) & 0x00FF));
data_byte = (U8) (*data_out_p & 0x00FF);
      return_error = MasterWriteI2C1(address_byte);
 	  return_error = MasterWriteI2C1(data_byte);
      if (return_error != NO_I2C_ERROR)
      {
         return return_error;
      }

      data_out_p++;
      transmission_count++;
   }

I2C1_Stop();

}

/************************************************************************
*    Function Name: I2C1_WriteByte
*    Description:    This routine is used to write a byte to the I2C bus.
*                    The input parameter data_out is written to the
*                    I2CTRN register. If IWCOL bit is set,write collision
*                    has occured and -1 is returned, else 0 is returned.
*    Parameters:     unsigned char : data_out
*    Return Value:   char
*************************************************************************/
/*void I2C1_WriteByte(U8 busAddr, unsigned char data_out){

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

   	//transmit control byte to write data
   	return_error = MasterWriteI2C1(busAddr);
   	if (return_error != NO_I2C_ERROR)
   	{
      return return_error;
   	}
	//	return_error = StopI2C1(); //transmit stop command
//return_error = IdleI2C1(); //wait until I2C Bus is Inactive
//return_error = StartI2C1(); //transmit START command
	return_error = MasterWriteI2C1(data_out);
 	if (return_error != NO_I2C_ERROR)
 	{
    return return_error;
	}

	return_error = StopI2C1(); //transmit stop command
	
	//return return_error;

}
*/

/************************************************************************
*    Function Name:  MasterWriteI2C1
*    Description:    This routine is used to write a byte to the I2C bus.
*                    The input parameter data_out is written to the
*                    I2CTRN register. If IWCOL bit is set,write collision
*                    has occured and -1 is returned, else 0 is returned.
*    Parameters:     unsigned char : data_out
*    Return Value:   char
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
}//MasterWriteI2C2



/************************************************************************
*    Function Name:  IdleI2C1
*    Description:    This routine generates wait condition until I2C2
*                    bus is Idle.
*    Parameters:     void
*    Return Value:   void
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
*    Return Value:   void
*********************************************************************/
static I2C_LOW_LEVEL_ERROR_T StartI2C1(void)
{
   I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

   I2C1CONbits.SEN = 1;   /* initiate Start on SDA and SCL pins */

   return_error = IdleI2C1();

   return return_error;
}//StartI2C1

/******************************************************************************
*    Function Name:  MasterReadI2C2
*    Description:    This routine reads a single byte from the I2C Bus.
*                    To enable master receive, RCEN bit is set.
*                    The RCEN bit is checked until it is cleared. When cleared,
*                    the receive register is full and it's contents are returned.
*    Parameters:     void
*    Return Value:   unsigned char
********************************************************************************/
static I2C_LOW_LEVEL_ERROR_T MasterReadI2C1(U8 * const data_p)
{
   U32 delay_count = 0;

   I2C1CONbits.RCEN = 1;

   while(I2C1CONbits.RCEN) //wait for 8 bits of reception to complete
   {
//      ClrWdt();
      delay_count++;
      if (delay_count > MAX_I2C_DELAY_COUNT)
         return I2C_TIMEOUT;
   }

   I2C1STATbits.I2COV = 0; //clear the receive overflow bit
   *data_p = I2C1RCV;
   return NO_I2C_ERROR;
}//MasterReadI2C2

/*********************************************************************
*    Function Name:  StopI2C1
*    Description:    This routine generates Stop condition
*                    during master mode.
*    Parameters:     void
*    Return Value:   void
*********************************************************************/
static I2C_LOW_LEVEL_ERROR_T StopI2C1(void)
{
   I2C_LOW_LEVEL_ERROR_T return_error = NO_I2C_ERROR;

   I2C1CONbits.PEN = 1;   /* initiate Stop on SDA and SCL pins */

   return_error = IdleI2C1();

   return return_error;
}//StopI2C2