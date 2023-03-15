///////////////////////////////////////////////////////////////////////////////////////////
//
//  Title: Adafruit ADS1015 ADC Test
//  Capstone Team: 37 
//  Authors: Tyler Dickson, Joseph Edmund, Isaac Sorensen
//  
//  Description: This is a body of code to test the Adafruit ADS1015 ADC breakout boards.
//  
///////////////////////////////////////////////////////////////////////////////////////////

/****************************************Libraries****************************************/
#include <Adafruit_ADS1X15.h> //Include the library for the Adafruit ADS1015 boards
#include <Wire.h>
/****************************************Macros****************************************/
//SDA is defined as pin 18 which is A4 on the Arduino nano
//SCL is defined as pin 19 which is A5 on the Arduino nano

//These values are defined in the Arduino core (or something like it) so there is no need to manually define them

//Code to verify this finding
// Serial.begin(115200);
//   Serial.println(F("\nI2C PINS"));
//   Serial.print(F("\tSDA = ")); Serial.println(SDA);
//   Serial.print(F("\tSCL = ")); Serial.println(SCL);

/****************************************Variables****************************************/
Adafruit_ADS1015 ads1015;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");
  
  ads1015.setGain(GAIN_ONE); 
  // Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  ads1015.begin();
}

void loop(void)
{
  int16_t adc0, results;
  // adc0 = ads1015.readADC_SingleEnded(0);
  results = ads1015.readADC_Differential_0_1();
  // Serial.print("Differential: "); 
  Serial.println(results);
  // Serial.println(adc0);

  // delay(1000);
}