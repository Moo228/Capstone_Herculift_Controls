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

/****************************************Debug****************************************/

#define PRINT_DEBUG

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
double loadScaleReading; // load cell value
double cableScaleReading; // load cell value
double tensionError; // our defined error value

//Enum to simplify motor movement.
enum MotorMotion{UP, DOWN, NONE};

//Define the variables we'll be connecting to
double setpoint, input, outputUp, outputDown;

//Specify the links and initial tuning parameters
double kpUp=.5, kdUp=0.0, kiUp=0.0;
double kpDown=.5, kdDown=0.0, kiDown=0.0;
PID myPID_UP(&input, &outputUp, &setpoint, kpUp, kiUp, kdUp, DIRECT);
PID myPID_DOWN(&input, &outputDown, &setpoint, kpDown, kiDown, kdDown, REVERSE);

/****************************************Function Declarations****************************************/

//This function simply calculates the difference between the tension measured by the load and the tension measured by the cable. 
//In other words, the error is the difference between the desired tension (cable value) and the actual tension (scaled load value).
//Parameters:
//  scaledTensionHandle- A double to hold the scaled value read in by the force sensor attatched to the load.
//  tensionCable- A double to hold the value read in by the force sensor attatched to the cable.
double calculateError(double scaledTensionHandle, double tensionCable);

//This function wraps the logic of motor control into a simple command.
//Parameters:
//  direction- uses the MotorMotion enum to specify UP, DOWN, or NONE.
//  dutyCycle- specify the duty cycle to run the motors at.
void moveMotor(MotorMotion direction, int dutyCycle);

//A function which defines the direction of the motor based upon the value of the error in the strain gauges.
//Parameters:
//  tensionError- the error value when comparing the strain gauge values.
MotorMotion setMotorDirection(double tensionError);

//This function tells the respective PID controller to compute its value based upon the updated information
//the motor is then moved in the appropriate direction at the calculated speed.
//Parameters:
//  direction- Specifies whether we are using either the 'up' PID controller or 'down' as well as the motor's direction
void computeAndMoveMotor(MotorMotion direction);

//A function that encapsulates the values we'd like to keep track of on the serial monitor.
//Parameters:
//  loadRead- the load side strain gauge value
//  cableRead- the cable side strain gauge value
//  tensionError- the output of calculateError()
//  outputUp- the output of myPID_up()
//  outputDown- the output of myPID_down()
//  direction- the output of setMotorDirection() used to specify whether outputUp or outputDown should be printed
void serialPrintDebug(double loadRead,   double cableRead, 
                      double tensionError, double outputUp, 
                      double outputDown, MotorMotion direction);

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
  loadScaleReading = analogRead(INPUT_PIN_LOAD);
  cableScaleReading = analogRead(INPUT_PIN_CABLE);
  tensionError = calculateError(loadScaleReading, cableScaleReading);

  //Initialize the variables we're linked to
  input = tensionError;
  setpoint = loadScaleReading * WEIGHT_ASSIST_FACTOR;

  //Turn the PID on
  myPID_UP.SetMode(AUTOMATIC);
  myPID_DOWN.SetMode(AUTOMATIC);
}

void loop() {
  //Read and normalize the values from the strain gauges.
  loadScaleReading = analogRead(INPUT_PIN_LOAD);
  cableScaleReading = analogRead(INPUT_PIN_CABLE);
  tensionError = calculateError(loadScaleReading, cableScaleReading);
  
  //Reinitialize the values for PID controllers
  input = cableScaleReading;
  setpoint = loadScaleReading * WEIGHT_ASSIST_FACTOR;

  //define which way the motor should move
  MotorMotion currentMotorDirection = setMotorDirection(tensionError);

  //Perform the PID computation and move the motor accordingly
  computeAndMoveMotor(currentMotorDirection);
  
#ifdef PRINT_DEBUG
  //Serial Output
  serialPrintDebug(loadScaleReading, 
                   cableScaleReading, 
                   tensionError,      
                   outputUp, 
                   outputDown,        
                   currentMotorDirection);
#endif

  delayMicroseconds(100); // Insert a delay to set the sample rate
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
    default:
      break;
  }
}

MotorMotion setMotorDirection(double tensionError) {
  MotorMotion direction = (tensionError > 0) ? (UP) : (DOWN);
  return direction;
}

void computeAndMoveMotor(MotorMotion direction) {
  switch (direction) {
    case UP:
      myPID_UP.Compute();
      moveMotor(UP, outputUp);
      break;
    case DOWN:
      myPID_DOWN.Compute();
      moveMotor(DOWN, outputDown);
      break;
    default:
      break;
  }
}

void serialPrintDebug(double loadRead,   double cableRead, 
                      double tensionError, double outputUp, 
                      double outputDown, MotorMotion direction) {
  //Note that there is no space after ':' in any of these statements
  //This is for allowing the Arduino Serial Plotter to use these strings as a labels
  Serial.print("Load:"); 
  Serial.print(loadRead, 3);
  Serial.print(", "); //Comma delimeter
  Serial.print("Cable:");
  Serial.print(cableRead, 3);
  Serial.print(", ");
  Serial.print("Error:");
  Serial.print(tensionError);
  Serial.print(", ");
  Serial.print("Output:");

  //Print the output of the currently used PID controller
  switch (direction) {
  case UP:
    Serial.print(outputUp);
    break;
  case DOWN:
    Serial.print(outputDown);
  default:
    break;
  }

  Serial.println();
}