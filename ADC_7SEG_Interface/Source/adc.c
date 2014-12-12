/********************************************************************
* Project: ADC_7SEG_Interface
*
* File: adc.c
*
* Description: FILL_IN_DESCRIPTION
*
* Known Issues: None
*
********************************************************************/


/********************************************************************
* Include Section
********************************************************************/
#include "p24FJ128GB108.h"
#include "adc_7seg.h"
#include "adc.h"
#include "delay.h"
#include "timer.h"
#include "error_codes.h"


/********************************************************************
* Function Prototype Section
********************************************************************/
ADC_ERROR_T wait_for_voltage_sample_to_complete(void);
ADC_ERROR_T init_adc(void);
ADC_ERROR_T read_adc(void);
U16 filtered_signal(void);

static U32 read_ad779x(const U8 reg);
static void write_ad779x(const U8 reg, const U32 data);
static U32 voltage_filter(const U32 sample, const U16 depth);
static float calculate_voltage(void);

/********************************************************************
* Static Variables
********************************************************************/ 
static U16 m_filtered_voltage = 0U;
//static U16 m_voltage_value = 0U;




/********************************************************************
* Function name : filtered_voltage_signal
*
* returns : The 16 most significant bits of the filtered voltage
*           signal.
*
* Created by : LuisMbedder
*
* Description : This function simply returns the 16 most significant
*               bits of the filtered voltage signal.
*
* Notes : None
********************************************************************/
U16 filtered_voltage_signal(void)
{
   U16 return_value = 0;
   
   return_value = m_filtered_voltage;

   return return_value;
}//filtered_voltage_signal


/********************************************************************
* Function name : wait_for_voltage_sample_to_complete
*
* Created by : LuisMbedder
*
* Description : This function waits for the AD779X to finish a 
*               conversion.  The AD779X sets the software loop time.
*
* Notes : None
********************************************************************/
ADC_ERROR_T wait_for_voltage_sample_to_complete(void)
{
   U16 adc_error = NO_ADC_ERROR;
   U16 start_wait = read_ms_count();

   //Wait for the A/D to be ready
   while(AD779X_NOT_RDY_PIN)
   {
      Nop();
      if (timeout(start_wait, ADC_TIMEOUT_MS))
      {
         adc_error = init_adc();
         if (adc_error != NO_ADC_ERROR)
            return ADC_COMM_ERROR;
      }
   }

   return NO_ADC_ERROR;
}//wait_for_voltage_sample_to_complete


/********************************************************************
* Function name : init_adc
* 
* ten_sample : pointer to U32 to hold new voltage sample
*
* returns : voltage adc error, if present
*
* Created by : LuisMbedder
*
* Description : Main A/D state machine.  Resets and initializes
*               the Analog to Digital converter.
*
********************************************************************/
ADC_ERROR_T init_adc(void)
{	
   AD779X_STATE_T ad779x_state = AD779X_RESET;
   volatile U8 dummy_byte = 0; //volatile to protect from optimization
   U16 init_attempt_counts = 0;
   U16 reset_count = 0;
   U8 u8temp;
   U16 u16temp;
   U32 U32temp;

   do
   {
      switch (ad779x_state)
      {
         case AD779X_RESET:
         default:
            init_attempt_counts++;
            if (init_attempt_counts > MAX_ADC_INIT_ATTEMPTS)
            {
               ad779x_state = AD779X_ERROR;
            }

            //the a/d is supposed to reset after 32 1's, send one extra byte 
            // of 1's to ensure it is reset in case some were missed due to noise
            for (reset_count = 0; reset_count < 5; reset_count++)
            {
               while(SPI1STATbits.SPITBF); //wait until Transmit Buffer Full bit is clear
               SPI1BUF = 0xFF;
               while(!SPI1STATbits.SPIRBF); //wait for transfer to complete
               dummy_byte = SPI1BUF; //clear the receive bit
            }

            //wait for the a/d to reset, which it indicates with !DRDY/MISO
            while(AD779X_NOT_RDY_PIN)	
            {
               Nop();
            }

            // According to Analog Devices tech support, wait 500us after reset
            // before addressing the serial device. The data sheet says to wait
            // 500ms in one place and 500us in another place. 500ms is too wrong.
            delay_10us(50);	// delay 500us
         
            ad779x_state = AD779X_INIT;
         //no break
         case AD779X_INIT:	//setup each reg for proper operation
            //read all config registers to ensure they are in
            // the default reset state, if not throw an
            // error so the ad779x can be reset

            //u8temp = (U8)read_ad779x(STATUS_REG);

            u16temp = (U16)read_ad779x(MODE_REG);
            if(u16temp != 0x000Au)
            {
               ad779x_state = AD779X_RESET;
               break;
            }
					
            u16temp = (U16)read_ad779x(CONFIG_REG);
            if(u16temp != 0x0710u)
            {
               ad779x_state = AD779X_RESET;
               break;
            }

            u8temp = (U8)read_ad779x(IO_REG);
            if(u8temp != 0x00u)
            {
               ad779x_state = AD779X_RESET;
               break;
            }
					
            U32temp = read_ad779x(OFFSET_REG);
            if(U32temp != 0x800000u)
            {
               ad779x_state = AD779X_RESET;
               break;
            }
					
            U32temp = read_ad779x(FULL_SCALE_REG);
            if( (U32temp & 0xF000FFu) != 0x500000u )
            {
               ad779x_state = AD779X_RESET;
               break;
            }
				
            //if we have gotten through all of the reads
            //successfully, try to configure the ad779x
            //for desired operation
            //MR11 to MR4 should be 0 for correct operation
            //write_ad779x(MODE_REG, 0x0202u); //242Hz update
            write_ad779x(MODE_REG, 0x0002u); //242Hz update
            u16temp = read_ad779x(MODE_REG);
            if(u16temp != 0x0002u)
            {
               ad779x_state = AD779X_RESET;
               break;
            }
					
               write_ad779x(CONFIG_REG, 0x1031u); //ad7799, an2, ext ref, G=1
               				
            u16temp = read_ad779x(CONFIG_REG);
            //check readback from AD7799 if connected
            if(u16temp != 0x1031u)  //ad7799, an2, ext ref, G=1
            {
               ad779x_state = AD779X_RESET;
               break;
            }
                
            //if everything is set up correctly so far,
            // transition to a zero scale calibration
            ad779x_state = AD779X_ZERO;
         //no break
         case AD779X_ZERO:
            //perform an internal zero calibration
            // Don't set bit MR9
            // Do internal calibration at 50Hz
            // write_ad779x(MODE_REG, 0x8202u); //242Hz update
            write_ad779x(MODE_REG, 0x8005u); //50Hz update
            ad779x_state = AD779X_CAL;			
         //no break;
         case AD779X_CAL:
            //perform an internal full scale calibration,
            // advance to run state, where the a/d will
            // be activated for conversions
            // Don't set bit MR9
            // Do internal calibration at 50Hz
            //write_ad779x(MODE_REG, 0xA202u); //242Hz update
            write_ad779x(MODE_REG, 0xA005u); //50Hz update
			
            ad779x_state = AD779X_RUN;
         //no break;    			
         case AD779X_RUN:
            //set the mode to continuous conversion			
            // write_ad779x(MODE_REG, 0x0202u); //242Hz update
            write_ad779x(MODE_REG, 0x0002u); //242Hz update
			
            u16temp = read_ad779x(MODE_REG);			
            if(u16temp != 0x0002u)  //242Hz
            {
               ad779x_state = AD779X_RESET;
               break;
            }
            else //if readback ok
               ad779x_state = AD779X_RESET_COMPLETE;
         break;
      }
   } while (ad779x_state != AD779X_RESET_COMPLETE && ad779x_state != AD779X_ERROR);

   if (ad779x_state == AD779X_RESET_COMPLETE)
      return NO_ADC_ERROR;
   else
      return ADC_COMM_ERROR;
}//init_adc


/********************************************************************
* Function name : read_adc
*
* returns : The ADC error state.
*
* Created by : LuisMbedder
*
* Description : This function is used to read the ADC,
*               filter and calculate the voltage percentage.
*
* Notes : None
********************************************************************/
ADC_ERROR_T read_adc(void)				
{
   ADC_ERROR_T error = NO_ADC_ERROR;
   U16 u16temp;
   U32 U32temp;
   U32 ten_sample = 0;
   U32 temp_filtered_voltage = 0;
   float voltage;
   //check mode register to verify communication hasn't fallen out
   // of sync due to an electrical event (such as ESD)
   u16temp = read_ad779x(MODE_REG);
   //if(u16temp != 0x0202u)  //242Hz
   if(u16temp != 0x0002u)  //242Hz
   {
      error = init_adc();
      if (error != NO_ADC_ERROR)
         return ADC_COMM_ERROR;
      else
      {
         u16temp = read_ad779x(MODE_REG);
         //if(u16temp != 0x0202u)  //242Hz
         if(u16temp != 0x0002u)  //242Hz
            return ADC_COMM_ERROR;  
      }
   }

   U32temp = read_ad779x(DATA_REG);
			
   //filter the sampled voltage
   ten_sample = U32temp;
   temp_filtered_voltage = voltage_filter(ten_sample, VOLTAGE_FILTER_DEPTH);

   //convert average voltage to a 16 bit value, regardless of ADC used  
   temp_filtered_voltage = (temp_filtered_voltage >> 8);
   m_filtered_voltage = temp_filtered_voltage;
	 voltage=(filtered_voltage_signal()*2.02)/65535;
   return NO_ADC_ERROR;	
}//read_voltage_adc


/********************************************************************
* Function name : read_ad779x
*
* reg : ad779x register to read
*
* returns : data from desired register
*
* Created by : LuisMbedder
*
* Description : Transfers a reading to the micro.
*
* Notes : None
********************************************************************/
static U32 read_ad779x(const U8 reg)
{
   volatile U8 dummy_byte = 0; //volatile to protect from optimization
   U8 comms_reg_write = 0b01000000;
   U8 bytes_to_read = 1;
   U32 result = 0;
   U32 U32temp = 0;
	
   //to read a register, we must first write to the comms reg
   // to setup the read (bits 5, 4, and 3 decode which reg)
   switch(reg)
   {
      default:
      case STATUS_REG:
         comms_reg_write |= 0b00000000;
      break;
		
      case MODE_REG:
         comms_reg_write |= 0b00001000;
         bytes_to_read = 2;
      break;
		
      case CONFIG_REG:
         comms_reg_write |= 0b00010000;
         bytes_to_read = 2;
      break;
		
      case DATA_REG:
         comms_reg_write |= 0b00011000;
         bytes_to_read = 3;
      break;
		
      case ID_REG:
         comms_reg_write |= 0b00100000;
      break;
		
      case IO_REG:
         comms_reg_write |= 0b00101000;
      break;
		
      case OFFSET_REG:
         comms_reg_write |= 0b00110000;
         bytes_to_read = 3;
      break;
		
      case FULL_SCALE_REG:
         comms_reg_write |= 0b00111000;
         bytes_to_read = 3;
      break;
	}
	
   //wait for the ad779x to be ready
	while(AD779X_NOT_RDY_PIN)	
   {
      Nop();
   }
	
   //execute the write to setup a read
   while(SPI1STATbits.SPITBF); //wait until Transmit Buffer Full bit is clear
   SPI1BUF = comms_reg_write;
   while(!SPI1STATbits.SPIRBF); //wait for transfer to complete
   dummy_byte = SPI1BUF; //clear the receive bit
	
   //read out the correct number of bytes per the register size	
   while(bytes_to_read > 0u)
   {
      SPI1BUF = 0x00; //read by transferring all 0's
      while(!SPI1STATbits.SPIRBF); //wait for transfer to complete		
      bytes_to_read--;
      U32temp = SPI1BUF;
		
      if(bytes_to_read == 2u)			
         result |= (U32temp << 16u);
      else if(bytes_to_read == 1u)
         result |= (U32temp << 8u);
      else
         result |= U32temp;
   }
	
   Nop();	
   return(result);	
}//read_ad779x


/********************************************************************
* Function name : write_ad779x
*
* reg : register to write data to
* data : data to write to selected register
*
* Created by : LuisMbedder
*
* Description : Transfers a write from the micro to the ADC.
*
* Notes : 
********************************************************************/
static void write_ad779x(const U8 reg, const U32 data)
{
   volatile U8 dummy_byte = 0; //volatile to protect from optimization
   U8 comms_reg_write = 0b00000000;
   U8 bytes_to_write = 1;
	
   //to write a register, we must first write to the comms reg
   // to setup the read (bits 5, 4, and 3 decode which reg)
   switch(reg)
   {
      default:
      case STATUS_REG:
         comms_reg_write |= 0b00000000;
      break;
		
      case MODE_REG:
         comms_reg_write |= 0b00001000;
         bytes_to_write = 2;
      break;
		
      case CONFIG_REG:
         comms_reg_write |= 0b00010000;
         bytes_to_write = 2;
      break;
		
      case DATA_REG:
         comms_reg_write |= 0b00011000;
         bytes_to_write = 3;
      break;
		
      case ID_REG:
         comms_reg_write |= 0b00100000;
      break;
		
      case IO_REG:
         comms_reg_write |= 0b00101000;
      break;
		
      case OFFSET_REG:
         comms_reg_write |= 0b00110000;
         bytes_to_write = 3;
      break;
		
      case FULL_SCALE_REG:
         comms_reg_write |= 0b00111000;
         bytes_to_write = 3;
      break;
   }
	
   //execute the write to setup the register write
   while(SPI1STATbits.SPITBF); //wait until Transmit Buffer Full bit is clear
   SPI1BUF = comms_reg_write;
   while(!SPI1STATbits.SPIRBF); //wait for transfer to complete
   dummy_byte = SPI1BUF; //clear the receive bit
	
   while(bytes_to_write > 0u)
   {
      bytes_to_write--;
		
      if(bytes_to_write == 2u)			
         SPI1BUF = (U8)((data & 0xFF0000) >> 16u);
      else if(bytes_to_write == 1u)
         SPI1BUF = (U8)((data & 0x00FF00) >> 8u);			
      else
         SPI1BUF = (U8)(data & 0x0000FF);
		
      while(!SPI1STATbits.SPIRBF); //wait for transfer to complete
      dummy_byte = SPI1BUF; //clear the receive bit
   }
}//write_ad779x


/********************************************************************
* Function name : voltage_filter
*
* sample : most recent voltage sample
* depth : sample depth of the filter
*
* returns : current filtered voltage value
*
* Created by : LuisMbedder
*
* Description : This function is the digital voltage filter.  It is 
*               a simple moving average FIR, with integer adjustable 
*               depth. 
*
* Notes : 
********************************************************************/
static U32 voltage_filter(const U32 sample, const U16 depth)
{
   static U32 running_sum = 0U;
   static U32 filter_array[VOLTAGE_FILTER_DEPTH];
   static U16 index = 0U;
   static U16 init = 0U;
   U16 filter_depth = depth;
   static U16 previous_depth = 0U;
   U32 return_val = sample;

   if (filter_depth == 0U) //protect against divide by zero
      filter_depth = 1U;
   if (filter_depth > VOLTAGE_FILTER_DEPTH) //protect array bounds overflow
      filter_depth = VOLTAGE_FILTER_DEPTH;

   //if the filter depth parameter has changed, re-initialize the filter
   if (filter_depth != previous_depth)
   {
      init = 0U;
      previous_depth = filter_depth;
   }
			
   //set the filter array on init to the current sample value
   if(init == 0U)
   {	
      for(index = 0U; index < filter_depth; index++)
      {
         filter_array[index] = sample;
      }
		
      running_sum = (sample * (U32)filter_depth);
      index = 0U;

      init = 1U;
   }
	
   running_sum = running_sum + sample - (U32)filter_array[index];
   filter_array[index] = sample;
   index++;
	
   return_val = running_sum / (U32)filter_depth;
	
   if(index > (filter_depth-1))
   {
      index = 0U;
   }

   return(return_val);
}//voltage_filter


