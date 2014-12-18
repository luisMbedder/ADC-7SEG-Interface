<snippet>
<content>
# ADC & LED Display Interface
This project interfaces a 24-bit ADC with a pic24 microcontroller using the SPI bus and displays the result on a 4-digit LED display using the I2C bus. The ADC samples the input at 242Hz, which gives a maximum potential response time of about 4msec. Some DSP is performed on the signal using a 16-term moving average filter. This essentially acts as a low pass or smoothing filter with about 80msec response.  If you need a faster response, the filter depth can be reduced but note that the smaller the filter depth, the more noise can work its way to the output. The software also contains error handling code for the ADC, if the software encounters an error during communication with the ADC an error signal will be reported by blinking all 0's on the display.
The breadboard and schematic are shown below. 
 
![Picture](https://cloud.githubusercontent.com/assets/7320156/5497430/6fcaf556-86df-11e4-9d70-9744a71e1622.JPG)

![Picture](https://cloud.githubusercontent.com/assets/7320156/5476600/a5bb01b8-85f1-11e4-88fc-7242de7033e1.JPG)
 
## Links
 
 [ADC Datasheet](http://www.analog.com/static/imported-files/data_sheets/AD7798_7799.pdf)
 
 [Display datasheet](http://www.adafruit.com/products/878)
 
 [PIC24 datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/39897c.pdf)
 





