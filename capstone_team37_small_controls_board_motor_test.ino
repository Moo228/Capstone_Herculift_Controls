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
#define MOTOR_FORWARD_PIN 5
#define MOTOR_BACKWARD_PIN 6

//Pins for HX711 breakout board.
#define LOAD_DT 2
#define LOAD_SCK 3

//Define values in which errorToPWM() will change functionality.
#define ERROR_ZONE_VAL 100
#define MOVE_ZONE_VAL 5000

#define MAX_MOTOR_PWM 255

//Define a fake number to test our single load cell system.
#define FAKE_LOAD_TENSION_VAL 1000.0

/****************************************Variables****************************************/

//Define variables for load cell reading & calibration factor.
double reading; // load cell value
double timeVar; // variable to track time
float calibration_factor = -8.5e4;

double scaleVal = 0.5;
double scaledTensionHandle;

//Object for scale.
HX711 scale;

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
  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {
   ; //Wait for serial port to connect. Needed for native USB port only
  }

  //Initialize communication with scale
  scale.begin(LOAD_DT, LOAD_SCK);

  //Zero scale when first starting
  scale.tare();

  //Apply calibration factor to scale and print to serial monitor
  scale.set_scale(calibration_factor);
  Serial.print("Calibration factor: ");
  Serial.println(calibration_factor);
}

void loop() {
  //Scale the value of the tension from the handle.
  scaledTensionHandle = scaleVal * FAKE_LOAD_TENSION_VAL;

  // Get reading from scale and print to serial monitor
  reading = -scale.get_units(10);
  Serial.println(reading*1000, 12);
  Serial.print("Error: ");
  Serial.println(scaledTensionHandle - reading*1000, 12);
  
  //Adjust the motor based on the read handle sensor data.
  errorToPWM(calculateError(scaledTensionHandle, reading*1000));
  //moveMotor(UP, 255);
}

/****************************************Function Definitions****************************************/

void moveMotor(MotorMotion direction, int dutyCycle) {
    switch (direction) {
      case UP:
        analogWrite(MOTOR_FORWARD_PIN, dutyCycle);
        analogWrite(MOTOR_BACKWARD_PIN, 0);
        break;
      case DOWN:
        analogWrite(MOTOR_FORWARD_PIN, 0);
        analogWrite(MOTOR_BACKWARD_PIN, dutyCycle);
        break;
      case NONE:
        analogWrite(MOTOR_FORWARD_PIN, 0);
        analogWrite(MOTOR_BACKWARD_PIN, 0);
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
  } else if(errorVal > -MOVE_ZONE_VAL && errorVal < -ERROR_ZONE_VAL) {
    //Move the motor down at a linear rate between the (x, y) points (-MOVE_ZONE_VAL, MAX_MOTOR_PWM) and (-ERROR_ZONE_VAL, 0).
    moveMotor(DOWN, map(errorVal, -MOVE_ZONE_VAL, -ERROR_ZONE_VAL, MAX_MOTOR_PWM, 0));
  } else if(errorVal > -ERROR_ZONE_VAL && errorVal < ERROR_ZONE_VAL) {
    //Stop the motor when errorVal is within a certain margin of error defined by 2 * ERROR_ZONE_VAL.
    moveMotor(NONE, 0);
  } else if(errorVal > ERROR_ZONE_VAL && errorVal < MOVE_ZONE_VAL) {
    //Move the motor down at a linear rate between the (x, y) points (ERROR_ZONE_VAL, 0) and (MOVE_ZONE_VAL, MAX_MOTOR_PWM).
    moveMotor(UP, map(errorVal, ERROR_ZONE_VAL, MOVE_ZONE_VAL, 0, MAX_MOTOR_PWM));
  } else {
    //Move the motor down at the max value when the error value is too positive (Safety feature to prevent excessive motor speed).
    moveMotor(UP, MAX_MOTOR_PWM);
  }
}
