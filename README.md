<snippet>
<content>
# ADC & LED Display Interface
This project interfaces a 24-bit ADC with a pic24 microcontroller using the SPI bus and displays the result on a 4-digit LED display using the I2C bus. The ADC samples the input at 242Hz, which gives a maximum potential response time of about 4msec.  The sampled signal is averaged in software using a 16-term moving average filter. This essentially acts as a low pass or smoothing filter with about 80msec response.  If you need a faster response, the filter depth can be reduced but note that the smaller the filter depth, the more noise can work its way to the output.
The breadboard and schematic are shown below. 
 
![Picture](https://cloud.githubusercontent.com/assets/7320156/5474628/d6537276-85e0-11e4-924c-47e41891e6b8.JPG)

![Picture](https://cloud.githubusercontent.com/assets/7320156/5476600/a5bb01b8-85f1-11e4-88fc-7242de7033e1.JPG)
 
## Links
 
 [ADC Datasheet](http://www.analog.com/static/imported-files/data_sheets/AD7798_7799.pdf)
 
 [Display datasheet](http://www.adafruit.com/products/878)
 
 [PIC24 datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/39897c.pdf)
 





