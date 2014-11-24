/********************************************************************
* Project: SW5
*
* File: module_template.c
*
* Copyright 2010 Dover Flexo Electronics, All Rights Reserved
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
#include "tension_adc.h"
#include "delay.h"
#include "timer.h"
#include "error_codes.h"


/********************************************************************
* Function Prototype Section
********************************************************************/
ADC_ERROR_T wait_for_tension_sample_to_complete(void);
ADC_ERROR_T init_tension_adc(void);
ADC_ERROR_T read_tension_adc(void);
U16 filtered_tension_signal(void);

U16 transducer_tension_percent(void);
U16 tension_sign(void);
ZERO_ERROR_T zero_transducer(void);
CAL_ERROR_T calibrate_transducer(void);

static U32 read_ad779x(const U8 reg);
static void write_ad779x(const U8 reg, const U32 data);
static U32 tension_filter(const U32 sample, const U16 depth);
static S16 calculate_transducer_tension(const U16 tension_adc_count,
                                        const U16 tension_zero_adc_count,
                                        const S32 tension_cal_span_count);


/********************************************************************
* Static Variables
********************************************************************/ 
static U16 m_filtered_tension = 0U;
static U16 m_tension_percent = 0U;
static U16 m_tension_sign = POSITIVE;
static enum ADC_VARIANT m_atod_variant = AD7799;


/********************************************************************
* The global variable tension filter array is used to convert
* the enumerated value used by the gui board to the filter depth
* required by the control board.
********************************************************************/ 
const U16 tension_filter_array[] = {1, 2, 4, 8, 16, 30, 60, 121, 242};



/********************************************************************
* Function name : filtered_tension_signal
*
* returns : The 16 most significant bits of the filtered tension
*           signal.
*
* Created by : Bill Twomey
* Date created : 4/8/2010
*
* Description : This function simply returns the 16 most significant
*               bits of the filtered tension signal.
*
* Notes : None
********************************************************************/
U16 filtered_tension_signal(void)
{
   U16 return_value = 0;
   
   return_value = m_filtered_tension;

   return return_value;
}//filtered_tension_signal


/********************************************************************
* Function name : transducer_tension_percent
*
* returns : Tension as a percentage with -2000 to 12000 representing
*           -20.00% to 120.00%.
*
* Created by : Bill Twomey
* Date created : 4/12/2010
*
* Description : This function simply returns the current tension as
*               a percentage, with with -2000 to 12000 representing
*               -20.00% to 120.00%.  In reality, tension cannot be
*               negative, but this helps the user identify and
*               trouble shoot potential tension errors or issues.
*
* Notes : None
********************************************************************/
U16 transducer_tension_percent(void)
{
   U16 return_value = 0;
   
   return_value = m_tension_percent;

   return return_value;
}//transducer_tension_percent


/********************************************************************
* Function name : tension_sign
*
* returns : Tension sign (POSITIVE or NEGATIVE)
*
* Created by : Bill Twomey
* Date created : 4/12/2010
*
* Description : This function simply returns the sign of the current
*               tension value.
*
* Notes : None
********************************************************************/
U16 tension_sign(void)
{
   U16 return_value = 0;
   
   return_value = m_tension_sign;

   return return_value;
}//tension_sign


/********************************************************************
* Function name : zero_transducer
*
* returns : The zero transducer error condition.
*
* Created by : Bill Twomey
* Date created : 6/8/2010
*
* Description : This function sets the transducer zero value.
*
* Notes : None
********************************************************************/
ZERO_ERROR_T zero_transducer(void)
{
   WRITE_STATE_T write_state = NO_EEPROM_ERROR;
   U16 zero_tension = 0;

   zero_tension = m_filtered_tension;

   if (zero_tension == 0U || zero_tension == 0xFFFF)
      return ZERO_RANGE_ERROR;
   else
   {
      do
      {
//         ClrWdt();
         write_state = write_config_parameter(XDCR_ZERO_TENSION_COUNTS, zero_tension);
         delay_ms(LOOP_TIME_MS);
      } while(write_state < WRITE_FAILED && write_state != WRITE_COMPLETE);

      if (write_state == WRITE_COMPLETE)
         return ZERO_OK;
      else
         return write_state;
   }
}//zero_transducer


/********************************************************************
* Function name : calibrate_transducer
*
* returns : The calibrate transducer error condition.
*
* Created by : Bill Twomey
* Date created : 6/8/2010
*
* Description : This function checks for a valid calibration and
*               sets the span value.
*
* Notes : None
********************************************************************/
CAL_ERROR_T calibrate_transducer(void)
{
   WRITE_STATE_T write_state = NO_EEPROM_ERROR; 
   U16 cal_tension = 0;
   S32 temp_span = 0;
   U32 span = 0;
   U16 span_sign = POSITIVE;
   u_param_cnvt param_cnvt;

   param_cnvt.double_word = 0U;

   cal_tension = m_filtered_tension;
   temp_span = ((S32)cal_tension - (S32)(read_config_parameter(XDCR_ZERO_TENSION_COUNTS)));

   if (temp_span < 0)
   {
      span_sign = NEGATIVE;
      temp_span = -temp_span;
   }

   //protect for divid by 0
   if (read_config_parameter(CALIBRATION_WEIGHT_PERCENT) == 0)
      return CAL_WEIGHT_VAL_ERROR;
   else
      span = (U32)((10000.0f / (float)(read_config_parameter(CALIBRATION_WEIGHT_PERCENT))) * (float)(temp_span) + 0.5f);

   param_cnvt.double_word = span;

   //check span doesn't violate minimum delta / maximum gain condition
   if ((EXC_5V_10V_OUT_PIN == TEN_VOLTS) && (span < MINIMUM_TENSION_SPAN_BITS / 2)) // XR
      return CAL_MIN_GAIN_ERROR;

   if ((EXC_5V_10V_OUT_PIN == FIVE_VOLTS) && (span < MINIMUM_TENSION_SPAN_BITS)) // STD
      return CAL_MIN_GAIN_ERROR;

   //save span counts low
   do
   {
//      ClrWdt();
      write_state = write_config_parameter(XDCR_CAL_SPAN_COUNTS_LOW, param_cnvt.param[0]);
      delay_ms(LOOP_TIME_MS);
   } while(write_state < WRITE_FAILED && write_state != WRITE_COMPLETE);
   if (write_state > WRITE_FAILED)
      return write_state;

   //save span counts high
   do
   {
//      ClrWdt();
      write_state = write_config_parameter(XDCR_CAL_SPAN_COUNTS_HIGH, param_cnvt.param[1]);
      delay_ms(LOOP_TIME_MS);
   } while(write_state < WRITE_FAILED && write_state != WRITE_COMPLETE);
   if (write_state > WRITE_FAILED)
      return write_state;

   //save span sign
   do
   {
//      ClrWdt();
      write_state = write_config_parameter(TENSION_SPAN_SIGN, span_sign);
      delay_ms(LOOP_TIME_MS);
   } while(write_state < WRITE_FAILED && write_state != WRITE_COMPLETE);
   if (write_state > WRITE_FAILED)
      return write_state;

   //clear the Cal Complete flag if needed
   if (read_config_parameter(XDCR_CAL_COMPLETE) == NO)
   {
      do
      {
//         ClrWdt();
         write_state = write_config_parameter(XDCR_CAL_COMPLETE, YES);
         delay_ms(LOOP_TIME_MS);
      } while(write_state < WRITE_FAILED && write_state != WRITE_COMPLETE);
      if (write_state > WRITE_FAILED)
         return write_state;
   }

   return CAL_OK;
}//calibrate_transducer


/********************************************************************
* Function name : wait_for_tension_sample_to_complete
*
* Created by : Bill Twomey
* Date created : 4/16/2010
*
* Description : This function waits for the AD779X to finish a 
*               conversion.  The AD779X sets the software loop time.
*
* Notes : None
********************************************************************/
ADC_ERROR_T wait_for_tension_sample_to_complete(void)
{
   U16 adc_error = NO_ADC_ERROR;
   U16 start_wait = read_ms_count();

   //Wait for the A/D to be ready
   while(AD779X_NOT_RDY_PIN)
   {
      Nop();
      if (timeout(start_wait, ADC_TIMEOUT_MS))
      {
         adc_error = init_tension_adc();
         if (adc_error != NO_ADC_ERROR)
            return TENSION_ADC_COMM_ERROR;
      }
   }

   return NO_ADC_ERROR;
}//wait_for_tension_sample_to_complete


/********************************************************************
* Function name : init_tension_adc
* 
* ten_sample : pointer to U32 to hold new tension sample
*
* returns : tension adc error, if present
*
* Created by : Bill Twomey
* Date created : 3/11/2010
*
* Description : Main A/D state machine.  Resets and initializes
*               the Analog to Digital converter.
*
* Notes : This function is based on code initially used on the Ti17c
*         /Ti18C project, which utilizes the same ADC.
********************************************************************/
ADC_ERROR_T init_tension_adc(void)
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
            if (init_attempt_counts > MAX_TENSION_ADC_INIT_ATTEMPTS)
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
							
            //determine which a/d is connected
            u8temp = (U8)read_ad779x(ID_REG);
            if( (u8temp & 0x0Fu) == 0x09u ) //ad7799
               m_atod_variant = AD7799;
            //else if( (u8temp & 0x0Fu) == 0x08u ) //ad7798
            //   m_atod_variant = AD7798;
            else //no other models accepted currently
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
					
            if(m_atod_variant == AD7799)
               write_ad779x(CONFIG_REG, 0x0331u); //ad7799, an2, ext ref, G=8
            //else if(m_atod_variant == AD7798)
            //   write_ad779x(CONFIG_REG, 0x0331u); //ad7798, an2, ext ref, G=8
               				
            u16temp = read_ad779x(CONFIG_REG);
            //check readback from AD7799 if connected
            if((m_atod_variant == AD7799) && (u16temp != 0x0331u))  //ad7798, an2, ext ref, G=8
            {
               ad779x_state = AD779X_RESET;
               break;
            }
            //check readback from AD7798 if connected
            //if((m_atod_variant == AD7798) && (u16temp != 0x0331u))  //ad7798, an2, ext ref, G=8
            //{
            //   ad779x_state = AD779X_RESET;
            //   break;
            //}
                
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
      return TENSION_ADC_COMM_ERROR;
}//init_tension_adc


/********************************************************************
* Function name : read_tension_adc
*
* returns : The tension ADC error state.
*
* Created by : Bill Twomey
* Date created : 7/13/2010
*
* Description : This function is used to read the tension ADC,
*               filter and calculate the tension percentage.
*
* Notes : None
********************************************************************/
ADC_ERROR_T read_tension_adc(void)				
{
   ADC_ERROR_T error = NO_ADC_ERROR;
   U16 u16temp;
   U32 U32temp;
   U32 ten_sample = 0;
   S32 transducer_cal_span = 0;
   U32 temp_filtered_tension = 0;
   S16 signed_tension_percent = 0;

   //check mode register to verify communication hasn't fallen out
   // of sync due to an electrical event (such as ESD)
   u16temp = read_ad779x(MODE_REG);
   //if(u16temp != 0x0202u)  //242Hz
   if(u16temp != 0x0002u)  //242Hz
   {
      error = init_tension_adc();
      if (error != NO_ADC_ERROR)
         return TENSION_ADC_COMM_ERROR;
      else
      {
         u16temp = read_ad779x(MODE_REG);
         //if(u16temp != 0x0202u)  //242Hz
         if(u16temp != 0x0002u)  //242Hz
            return TENSION_ADC_COMM_ERROR;  
      }
   }

   U32temp = read_ad779x(DATA_REG);
   //for 16 bit a/d, need to remove lower byte
   // and right shift result to bring down to 16 bits
   //if(m_atod_variant == AD7798)
   //{   
   //   U32temp = U32temp & 0xFFFF00;
   //   U32temp = U32temp >> 8;
   //}
   //else //24 bit a/d
			
   //filter the sampled tension
   ten_sample = U32temp;
   temp_filtered_tension = tension_filter(ten_sample, tension_filter_array[TENSION_FILTER_DEPTH]);

   //convert average tension to a 16 bit value, regardless of ADC used
   if(m_atod_variant == AD7799)
   {   
      temp_filtered_tension = (temp_filtered_tension >> 8);
   }
   m_filtered_tension = temp_filtered_tension;

   //calculate tension as percentage (-20.00% to 120.00% represented as -2000 to 12000)
   //only calculate transducer tension tension if calibration has been completed
   if (read_config_parameter(XDCR_CAL_COMPLETE) == YES)
   {
      transducer_cal_span = (((U32)read_config_parameter(XDCR_CAL_SPAN_COUNTS_HIGH)) << 16) & 0xFFFF0000;
      transducer_cal_span = transducer_cal_span | ((U32)read_config_parameter(XDCR_CAL_SPAN_COUNTS_LOW) & 0x0000FFFF);
      if (read_config_parameter(TENSION_SPAN_SIGN) == NEGATIVE)
         transducer_cal_span = -transducer_cal_span;
      signed_tension_percent = calculate_transducer_tension(m_filtered_tension,
                                                            read_config_parameter(XDCR_ZERO_TENSION_COUNTS),
                                                            transducer_cal_span);
   }
   else 
      signed_tension_percent = -250; //output -0.25V if tension has not been calibrated

   if (signed_tension_percent < 0)
   {
      m_tension_sign = NEGATIVE;
      signed_tension_percent = -(signed_tension_percent);
   }
   else
   {
      m_tension_sign = POSITIVE;
   }
   m_tension_percent = (U16)signed_tension_percent;
	
   return NO_ADC_ERROR;	
}//read_tension_adc


/********************************************************************
* Function name : read_ad779x
*
* reg : ad779x register to read
*
* returns : data from desired register
*
* Created by : Bill Twomey
* Date created : 3/11/2010
*
* Description : Transfers a reading to the micro.
*
* Notes : This function is based on code initially used on the Ti17c
*         /Ti18C project, which utilizes the same ADC.
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
* Created by : Bill Twomey
* Date created : 3/11/2010
*
* Description : Transfers a write from the micro to the ADC.
*
* Notes : This function is based on code initially used on the Ti17c
*         /Ti18C project, which utilizes the same ADC.
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
* Function name : tension_filter
*
* sample : most recent tension sample
* depth : sample depth of the filter
*
* returns : current filtered tension value
*
* Created by : Bill Twomey
* Date created : 3/12/2010
*
* Description : This function is the digital tension filter.  It is 
*               a simple moving average FIR, with integer adjustable 
*               depth. 
*
* Notes : This function is based on code initially used on the Ti17c
*         /Ti18C project
********************************************************************/
static U32 tension_filter(const U32 sample, const U16 depth)
{
   static U32 running_sum = 0U;
   static U32 filter_array[TENSION_FILTER_DEPTH];
   static U16 index = 0U;
   static U16 init = 0U;
   U16 filter_depth = depth;
   static U16 previous_depth = 0U;
   U32 return_val = sample;

   if (filter_depth == 0U) //protect against divide by zero
      filter_depth = 1U;
   if (filter_depth > TENSION_FILTER_DEPTH) //protect array bounds overflow
      filter_depth = TENSION_FILTER_DEPTH;

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
}//tension_filter


/********************************************************************
* Function name : calculate_transducer_tension
*
* tension_adc_count : The 16-bit transducer adc tension reading
* tension_zero_adc_count : The 16-bit transducer adc zero tension value
* tension_cal_span_count: The calibrated tension span
*
* returns : Tension as a percentage, with 10,000 representing 100%
*
* Created by : Bill Twomey
* Date created : 4/9/2010
*
* Description : This function simply calculates the tension as an
*               interger precentage, with 10,000 representing 100%.
*
* Notes : None
********************************************************************/
static S16 calculate_transducer_tension(const U16 tension_adc_count,
                                        const U16 tension_zero_adc_count,
                                        const S32 tension_cal_span_count)
{
   S32 tension_percentage = 0; //10000 = 100.00%

   tension_percentage = (10000*((S32)tension_adc_count - (S32)tension_zero_adc_count))/tension_cal_span_count;

   //peg the tension percentage to 20% over and under range
   if(tension_percentage > 12000)
      tension_percentage = 12000;
   else if(tension_percentage < -2000)
      tension_percentage = -2000;

   return ((S16)tension_percentage);
}//calculate_transducer_tension



