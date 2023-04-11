///////////////////////////////////////////////////////////////////////////////////////////
//
//  Title: Herculift Device Control System
//  Capstone Team: 37 
//  Authors: Tyler Dickson, Joseph Edmund, Isaac Sorensen
//  
//  Description: This is a body of code implementing the controls system of the wearable lift 
//    assist device (Herculift Device) by using a 2D mockup of the system.
//  
///////////////////////////////////////////////////////////////////////////////////////////

/****************************************Libraries****************************************/

#include <PID_v1.h>

/****************************************Macros****************************************/

//Per the Arduino Nano documentation, pins 3, 5, 6, 9, 10, and 11 support pulse width modulation (PWM).
//The PWM pins 5 and 6 are connected to a 16-bit timer and run at a default frequency of 976.56Hz.

//Pins connected to the motor controller.
#define MOTOR_DIRECTION_PIN 2
#define ENABLE_PIN 3
#define READY_PIN 4
#define MOTOR_PWM_INPUT_PIN 5

//Pin for strain gauge amplifier for the cable. Pin 16 corresponds to pin A2.
#define INPUT_PIN_CABLE 16

//Pin for strain gauge amplifier for the load. Pin 17 corresponds to pin A3.
#define INPUT_PIN_LOAD 17

//Define a number to use as the scaled value of the load tension.
#define WEIGHT_ASSIST_FACTOR 0.75

/****************************************Variables****************************************/

//Define variables for load cell readings.
double load_scale_reading; // load cell value
double cable_scale_reading; // load cell value
double tension_error; // our defined error value

//Enum to simplify motor movement.
enum MotorMotion{UP, DOWN, NONE};

//Define the variables we'll be connecting to
double setpoint, input, output_up, output_down;

//Specify the links and initial tuning parameters
double kp_up=.5, kd_up=0.0, ki_up=0.0;
double kp_down=.5, kd_down=0.0, ki_down=0.0;
PID myPID_UP(&input, &output_up, &setpoint, kp_up, ki_up, kd_up, DIRECT);
PID myPID_DOWN(&input, &output_down, &setpoint, kp_down, ki_down, kd_down, REVERSE);

/****************************************Function Declarations****************************************/

//This function simply calculates the difference between the tension measured by the load and the tension measured by the cable. 
//In other words, the error is the difference between the desired tension (cable value) and the actual tension (scaled load value).
//The value read by the tension sensor in the handle is scaled down before being passed into this function.
//Parameters:
//  scaledTensionHandle- A double to hold the scaled value read in by the force sensor attatched to the load.
//  tensionCable- A double to hold the value read in by the force sensor attatched to the cable.
double calculateError(double scaledTensionHandle, double tensionCable);

//This function wraps the logic of motor control into a simple command.
//Parameters:
//  direction- uses the MotorMotion enum to specify UP, DOWN, or NONE.
//  dutyCycle- specify the duty cycle to run the motors at.
void moveMotor(MotorMotion direction, int dutyCycle);

/****************************************Main****************************************/

void setup() {
  //Set the pins as output.
  //These pins will control the motors by interfacing with the motor controller.
  pinMode(MOTOR_PWM_INPUT_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(READY_PIN, INPUT);

  //Set the enable pin high.
  digitalWrite(ENABLE_PIN, HIGH);

  //Set the direction pin high.
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);

  Serial.begin(9600);
  while (!Serial) {
   ; //Wait for serial port to connect. Needed for native USB port only.
  }
  analogReference(INTERNAL); // Set the internal reference voltage to 1.1V to increase resolution on force sensors

  //Read and normalize the values from the strain gauges.
  load_scale_reading = analogRead(INPUT_PIN_LOAD);
  cable_scale_reading = analogRead(INPUT_PIN_CABLE);
  tension_error = calculateError(load_scale_reading, cable_scale_reading);

  //Initialize the variables we're linked to
  input = tension_error;
  setpoint = load_scale_reading * WEIGHT_ASSIST_FACTOR;

  //Turn the PID on
  myPID_UP.SetMode(AUTOMATIC);
  myPID_DOWN.SetMode(AUTOMATIC);
}

void loop() {
  //Read and normalize the values from the strain gauges. A decimal percentage of 
  load_scale_reading = analogRead(INPUT_PIN_LOAD);
  cable_scale_reading = analogRead(INPUT_PIN_CABLE);
  tension_error = calculateError(load_scale_reading, cable_scale_reading);

  input = tension_error;
  setpoint = load_scale_reading * WEIGHT_ASSIST_FACTOR;

  //Refactor?
  if (tension_error > 0) {
    myPID_UP.Compute();
    moveMotor(UP, output_up);
  }
  else {
    myPID_DOWN.Compute();
    moveMotor(DOWN, output_down);
  }
  
  //Serial Output
  Serial.print("Load:");
  Serial.print(load_scale_reading, 3);
  Serial.print(", ");
  Serial.print("Cable:");
  Serial.print(cable_scale_reading, 3);
  Serial.print(", ");
  Serial.print("Error:");
  Serial.print(tension_error);
  // Serial.print(", ");
  // Serial.print("Output:");
  // Serial.println(output);

  delayMicroseconds(100); // Wait and increase the sampling rate
}

/****************************************Function Definitions****************************************/

double calculateError(double scaledTensionHandle, double tensionCable) {
  return WEIGHT_ASSIST_FACTOR * scaledTensionHandle - tensionCable;
}

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