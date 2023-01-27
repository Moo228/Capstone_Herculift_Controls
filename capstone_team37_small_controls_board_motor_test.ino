///////////////////////////////////////////////////////////////////////////////////////////
//
//  Capstone Team 37 Code
//
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////

/****************************************Include Statements****************************************/

//Include HX711 Arduino Library (Bogdan Necula & Andreas Motl) for load cell.
#include "HX711.h"

/****************************************Macros****************************************/

//Per the Arduino nano documentation pins 3, 5, 6, 9, 10, and 11 support pulse width modulation (PWM).
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

/****************************************Variables****************************************/

//Define variables for load cell reading & calibration factor.
double reading; // load cell value
double timeVar; // variable to track time
float calibration_factor = -8.5e4;

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

double calculateError(double tensionHandle, double tensionCable);
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
  // Set reference time
  timeVar = micros()/1e6;

  // Get reading from scale and print to serial monitor
  reading = -scale.get_units(10);
  // Serial.print("dt (s): ");
  // Serial.print(micros()/1e6 - timeVar, 6);
  // Serial.print(" Load (g): ");
  Serial.println(reading*1000, 12);
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

double calculateError(double tensionHandle, double tensionCable) {
  return tensionHandle - tensionCable;
}

void errorToPWM(double errorVal) {
  if(errorVal < -MOVE_ZONE_VAL){
    //Move the motor down at the max value when the error value is two negetive.
    moveMotor(DOWN, MAX_MOTOR_PWM);
  } else if(errorVal > -MOVE_ZONE_VAL && errorVal < -ERROR_ZONE_VAL){
    //Move the motor down at a linear rate
    moveMotor(DOWN, map(abs(errorVal), ERROR_ZONE_VAL, MOVE_ZONE_VAL, 0, MAX_MOTOR_PWM));
  } else if(errorVal > -ERROR_ZONE_VAL && errorVal < ERROR_ZONE_VAL){
    moveMotor(NONE, 0);
  } else if(errorVal > ERROR_ZONE_VAL && errorVal < MOVE_ZONE_VAL){
    moveMotor(UP, map(abs(errorVal), ERROR_ZONE_VAL, MOVE_ZONE_VAL, 0, MAX_MOTOR_PWM));
  } else {
    moveMotor(UP, MAX_MOTOR_PWM);
  }
}
