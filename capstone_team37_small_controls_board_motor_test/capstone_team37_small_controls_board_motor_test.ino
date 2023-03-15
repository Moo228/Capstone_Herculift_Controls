///////////////////////////////////////////////////////////////////////////////////////////
//
//  Title: Control System 2D Test Board
//  Capstone Team: 37 
//  Authors: Tyler Dickson, Joseph Edmund, Isaac Sorensen
//  
//  Description: This is a body of code to test the controls system of the wearable lift 
//    assist device (Herculift Device) by using a 2D mockup of the system.
//  
///////////////////////////////////////////////////////////////////////////////////////////

/****************************************Libraries****************************************/

//Include HX711 Arduino Library (Bogdan Necula & Andreas Motl) for load cell.
#include "HX711.h"

/****************************************Macros****************************************/

//Per the Arduino Nano documentation, pins 3, 5, 6, 9, 10, and 11 support pulse width modulation (PWM).
//The PWM pins 5 and 6 are connected to a 16-bit timer and run at a default frequency of 976.56Hz.
//The PWM pins 3, and 11 are connected to Timer 0? in the ATmega168p microcontroller chip.
//The PWM pins 9, and 10 are connected to Timer 2? in the ATmega168p microcontroller chip.
//This will be important for modifying the default frequencies supported by the PWM pins (controlled by the timers).

//Pins connected to L298N motor controller.
#define MOTOR_PWM_INPUT_PIN 5
#define MOTOR_DIRECTION_PIN 12

//Pins for HX711 breakout board for the load.
#define LOAD_DT 2
#define LOAD_SCK 3

//Pins for HX711 breakout board for the cable.
#define CABLE_DT 7
#define CABLE_SCK 8

//Define values in which errorToPWM() will change functionality.
#define ERROR_ZONE_VAL 100
#define MOVE_ZONE_VAL 5000

#define MAX_MOTOR_PWM 30

//Define a fake number to test our single load cell system.
#define FAKE_LOAD_TENSION_VAL 3000.0

//Define the sample period. This is used for load_scale.get_units() in the HX711 library.
#define SAMPLE_PERIOD 1

//Define a number to use as the scaled value of the load tension.
#define WEIGHT_ASSIST_FACTOR 0.5

//A constant to calibrate the load cells to correct values.
#define LOAD_CALIBRATION_FACTOR -8.5e4
#define CABLE_CALIBRATION_FACTOR -8.5e4

/****************************************Variables****************************************/

//Define variables for load cell readings & calibration factors.
double load_scale_reading; // load cell value
double cable_scale_reading; // load cell value
double timeVar; // variable to track time

//Create a variable to hold the value of the weight of the load. It will be scaled by WEIGHT_ASSIST_FACTOR.
double scaledTensionHandle;

//Object for the load cells for the cable and handle.
HX711 load_scale;
HX711 cable_scale;

//Enum to simplify motor movement.
enum MotorMotion{UP, DOWN, NONE};

/****************************************Function Declarations****************************************/

//This function wraps the logic of motor control into a simple command.
//Parameters:
//  direction- uses the MotorMotion enum to specify UP, DOWN, or NONE.
//  dutyCycle- specify the duty cycle to run the motors at.
void moveMotor(MotorMotion direction, int dutyCycle);

//This function simply calculates the difference between the tension measured by the load and the tension measured by the cable. 
//In other words, the error is the difference between the desired tension (cable value) and the actual tension (scaled load value).
//The value read by the tension sensor in the handle is scaled down before being passed into this function.
//Parameters:
//  scaledTensionHandle- A double to hold the scaled value read in by the force sensor attatched to the load.
//  tensionCable- A double to hold the value read in by the force sensor attatched to the cable.
double calculateError(double scaledTensionHandle, double tensionCable);

//A function that takes in the error in load and cable tension and converts that into a PWM value for the L298N motor controller.
//Parameters:
//  errorVal- the value returned from calculateError().
void errorToPWM(double errorVal);

/****************************************Main****************************************/

void setup() {
  //Set the pins as output.
  //These pins will control the motors by interfacing with the L298N Motor controller.
  pinMode(MOTOR_PWM_INPUT_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {
   ; //Wait for serial port to connect. Needed for native USB port only.
  }

  //Initialize communication with the load and cable scales.
  load_scale.begin(LOAD_DT, LOAD_SCK);
  cable_scale.begin(CABLE_DT, CABLE_SCK);

  //Zero both scales when first starting
  load_scale.tare();
  cable_scale.tare();

  //Apply calibration factor to load and cable scales and print to serial monitor.
  load_scale.set_scale(LOAD_CALIBRATION_FACTOR);
  cable_scale.set_scale(CABLE_CALIBRATION_FACTOR);
  Serial.print("Load calibration factor: ");
  Serial.println(LOAD_CALIBRATION_FACTOR);
}

void loop() {
  //Get reading from the scales and print to serial monitor.
  load_scale_reading = -load_scale.get_units(SAMPLE_PERIOD);
  cable_scale_reading = -cable_scale.get_units(SAMPLE_PERIOD);

  //Scale the value of the tension from the handle.
  scaledTensionHandle = WEIGHT_ASSIST_FACTOR * load_scale_reading;
  
  // load_scale_reading = load_scale_reading * 1000;
  // cable_scale_reading = cable_scale_reading * 1000;
  Serial.print(load_scale_reading * 1000);
  Serial.print(",");
  Serial.print(cable_scale_reading * 1000);
  Serial.print(",");
  Serial.println(calculateError(scaledTensionHandle * 1000, cable_scale_reading * 1000));
  
  //Adjust the motor based on the read handle sensor data.
  errorToPWM(calculateError(scaledTensionHandle * 1000, cable_scale_reading*1000));
}

/****************************************Function Definitions****************************************/

void moveMotor(MotorMotion direction, int dutyCycle) {
    switch (direction) {
      case UP:
        //When going UP put the direction pin of the motor control driver LOW.
        analogWrite(MOTOR_PWM_INPUT_PIN, dutyCycle);
        digitalWrite(MOTOR_DIRECTION_PIN, LOW);
        break;
      case DOWN:
        //When going UP put the direction pin of the motor control driver HIGH.
        analogWrite(MOTOR_PWM_INPUT_PIN, dutyCycle);
        digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
        break;
      case NONE:
        //The driver naturally pulls the pin high so put the direction pin HIGH.
        analogWrite(MOTOR_PWM_INPUT_PIN, 0);
        digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
        break;
    }
}

double calculateError(double scaledTensionHandle, double tensionCable) {
  return scaledTensionHandle - tensionCable;
}

//                              ||
// ________________             ||             ________________<- MAX DUTY CYCLE
//                |\            ||            /|               
//                | \           ||           / |               
//                |  \          ||          /  |               
//                |   \         ||         /   |               
//                |    \        ||        /    |               
//                |     \       ||       /     |               
//                |      \______||______/<-----|----------------- MIN DUTY CYCLE (0)         
// ===============|======|======||======|======|===============
//                |      |      ||      |      |
//             -MOVE  -ERROR          ERROR   MOVE
//              ZONE   ZONE           ZONE    ZONE 
//              VAL    VAL            VAL     VAL  

//map(value, fromLow, fromHigh, toLow, toHigh)
//Parameters
//  value- the number to map.
//  fromLow- the lower bound of the value's current range.
//  fromHigh- the upper bound of the value's current range.
//  toLow- the lower bound of the value's target range.
//  toHigh- the upper bound of the value's target range.

void errorToPWM(double errorVal) {
  if(errorVal < -MOVE_ZONE_VAL) {
    //Move the motor down at the max value when the error value is too negetive (Safety feature to prevent excessive motor speed).
    moveMotor(DOWN, MAX_MOTOR_PWM);
    Serial.println("MAX DOWN regime");
  } else if(errorVal > -MOVE_ZONE_VAL && errorVal < -ERROR_ZONE_VAL) {
    //Move the motor down at a linear rate between the (x, y) points (-MOVE_ZONE_VAL, MAX_MOTOR_PWM) and (-ERROR_ZONE_VAL, 0).
    // moveMotor(DOWN, map(errorVal, -MOVE_ZONE_VAL, -ERROR_ZONE_VAL, MAX_MOTOR_PWM, 0));
    moveMotor(DOWN, 30);
    Serial.println("DOWN regime");
  } else if(errorVal > -ERROR_ZONE_VAL && errorVal < ERROR_ZONE_VAL) {
    //Stop the motor when errorVal is within a certain margin of error defined by 2 * ERROR_ZONE_VAL.
    moveMotor(NONE, 0);
    Serial.println("NONE regime");
  } else if(errorVal > ERROR_ZONE_VAL && errorVal < MOVE_ZONE_VAL) {
    //Move the motor down at a linear rate between the (x, y) points (ERROR_ZONE_VAL, 0) and (MOVE_ZONE_VAL, MAX_MOTOR_PWM).
    // moveMotor(UP, map(errorVal, ERROR_ZONE_VAL, MOVE_ZONE_VAL, 0, MAX_MOTOR_PWM));
    moveMotor(UP, 30);
    Serial.println("UP regime");
  } else {
    //Move the motor down at the max value when the error value is too positive (Safety feature to prevent excessive motor speed).
    moveMotor(UP, MAX_MOTOR_PWM);
    Serial.println("MAX UP regime");

  }
}
