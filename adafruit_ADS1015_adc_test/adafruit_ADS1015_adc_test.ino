///////////////////////////////////////////////////////////////////////////////////////////
//
//  Title: Adafruit ads1015 ADC Test
//  Capstone Team: 37 
//  Authors: Tyler Dickson, Joseph Edmund, Isaac Sorensen
//  
//  Description: This is a body of code to test the Adafruit ads1015 ADC breakout boards.
//  
///////////////////////////////////////////////////////////////////////////////////////////

/****************************************Libraries****************************************/
#include <Adafruit_ADS1X15.h> //Include the library for the Adafruit ads1015 boards
#include <Wire.h>
/****************************************Macros****************************************/
//SDA is defined as pin 18 which is A4 on the Arduino nano
//SCL is defined as pin 19 which is A5 on the Arduino nano
//ADDR for the load side ADC is connected to GND for this setup. This gives the ADC an address of 0x48 for I2C.
//ADDR for the cable side ADC is connected to 5V for this setup. This gives the ADC an address of 0x49 for I2C.

#define LOAD_ADC_ADDRESS 0x48
#define CABLE_ADC_ADDRESS 0x49

//These values are defined in the Arduino core (or something like it) so there is no need to manually define them

//Code to verify this finding
// Serial.begin(115200);
//   Serial.println(F("\nI2C PINS"));
//   Serial.print(F("\tSDA = ")); Serial.println(SDA);
//   Serial.print(F("\tSCL = ")); Serial.println(SCL);

/****************************************Variables****************************************/
Adafruit_ADS1015 loadADC;
Adafruit_ADS1015 cableADC;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");
  
  loadADC.setGain(GAIN_TWO); 
  loadADC.begin(LOAD_ADC_ADDRESS);
  cableADC.setGain(GAIN_TWO); 
  cableADC.begin(CABLE_ADC_ADDRESS);
}

void loop(void)
{
  int16_t adc0v1, adc0v2;
  adc0v1 = loadADC.readADC_SingleEnded(0);
  adc0v2 = cableADC.readADC_SingleEnded(0);

  Serial.print(adc0v1);
  Serial.print(", ");
  Serial.println(adc0v2);

  // delay(1000);
}