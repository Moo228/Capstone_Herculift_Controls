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

// #include PID.h

/****************************************Macros****************************************/

//Per the Arduino Nano documentation, pins 3, 5, 6, 9, 10, and 11 support pulse width modulation (PWM).
//The PWM pins 5 and 6 are connected to a 16-bit timer and run at a default frequency of 976.56Hz.
//The PWM pins 3, and 11 are connected to Timer 0? in the ATmega168p microcontroller chip.
//The PWM pins 9, and 10 are connected to Timer 2? in the ATmega168p microcontroller chip.
//This will be important for modifying the default frequencies supported by the PWM pins (controlled by the timers).

//Pins connected to L298N motor controller.
#define MOTOR_DIRECTION_PIN 2
#define ENABLE_PIN 3
#define READY_PIN 4
#define MOTOR_PWM_INPUT_PIN 5


//Pin for strain gauge amplifier for the load. Pin 16 corresponds to pin A3.
#define INPUT_PIN_LOAD 17

//Pin for strain gauge amplifier for the cable. Pin 17 corresponds to pin A2.
#define INPUT_PIN_CABLE 16

//Define values in which errorToPWM() will change functionality.
#define ERROR_ZONE_VAL 0.05
#define MOVE_ZONE_VAL 0.25

#define MAX_MOTOR_PWM 150
#define MIN_MOTOR_PWM 20

//Define the sample period. This is used for load_scale.get_units() in the HX711 library.
// #define SAMPLE_PERIOD 1

//Define a number to use as the scaled value of the load tension.
#define WEIGHT_ASSIST_FACTOR 0.75

/****************************************Variables****************************************/

//Define variables for load cell readings & calibration factors.
double load_scale_reading; // load cell value
double cable_scale_reading; // load cell value
double timeVar; // variable to track time

//Create a variable to hold the value of the weight of the load. It will be scaled by WEIGHT_ASSIST_FACTOR.
double scaledTensionHandle;

//Enum to simplify motor movement.
enum MotorMotion{UP, DOWN, NONE};

//Enum to categorize the motor movement regime.
enum MovementRegime{MAX_UP_REGIME, UP_REGIME, NONE_REGIME, DOWN_REGIME, MAX_DOWN_REGIME};

/****************************************Function Declarations****************************************/

// This is the built-in Arduno map function but with the double data type instead of the long data type.
//Parameters:
//  x- the number to map.
//  in_min- the lower bound of the value's current range.
//  in_max- the upper bound of the value's current range.
//  out_in- the lower bound of the value's target range.
//  out_max- the upper bound of the value's target range.
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

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
//It returns an enumerated type that allows for diagnosis of which part of the function is being called.
//Parameters:
//  errorVal- the value returned from calculateError().
enum MovementRegime errorToPWM(double errorVal);

/****************************************Main****************************************/

void setup() {
  //Set the pins as output.
  //These pins will control the motors by interfacing with the L298N Motor controller.
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
  analogReference(INTERNAL); // Set the internal reference voltage to 1.1V 
}

void loop() {
  // unsigned long startTime = millis(); // Record the start time
  
  //Read and normalize the values from the strain gauges. A decimal percentage of 
  load_scale_reading = analogRead(INPUT_PIN_LOAD)/1023.0;
  cable_scale_reading = analogRead(INPUT_PIN_CABLE)/1023.0;

  //Scale the value of the tension from the handle.
  // scaledTensionHandle = WEIGHT_ASSIST_FACTOR * load_scale_reading;

  Serial.print("Load:");
  Serial.print(load_scale_reading, 3);
  Serial.print(", ");
  Serial.print("Cable:");
  Serial.print(cable_scale_reading, 3);
  Serial.print(", ");
  Serial.print("Error:");
  Serial.print(calculateError(load_scale_reading, cable_scale_reading));
  
  //Adjust the motor based on the read handle sensor data.
  enum MovementRegime movementRegime = errorToPWM(calculateError(load_scale_reading, cable_scale_reading));
  Serial.print(", ");
  Serial.print("Regime:");
  Serial.println((double) movementRegime/10.0);

  delayMicroseconds(100); // Wait and increase the sampling rate
}

/****************************************Function Definitions****************************************/

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

double calculateError(double scaledTensionHandle, double tensionCable) {
  return WEIGHT_ASSIST_FACTOR * scaledTensionHandle - tensionCable;
}
//                              PWM
//                              ||
// ________________             ||             ________________<- MAX DUTY CYCLE
//                |\            ||            /|               
//                | \           ||           / |               
//                |  \          ||          /  |               
//                |   \         ||         /   |               
//                |    \        ||        /    |               
//                |     \       ||       /     |               
//                |      \______||______/<-----|----------------- MIN DUTY CYCLE (0)         
// ===============|======|======||======|======|=============== Error
//                |      |      ||      |      |
//             -MOVE  -ERROR          ERROR   MOVE
//              ZONE   ZONE           ZONE    ZONE 
//              VAL    VAL            VAL     VAL  

 enum MovementRegime errorToPWM(double errorVal) {
  if(errorVal < -MOVE_ZONE_VAL) {
    //Move the motor down at the max value when the error value is too negetive (Safety feature to prevent excessive motor speed).
    moveMotor(DOWN, MAX_MOTOR_PWM);
    // Serial.println("MAX DOWN regime");
    return MAX_UP_REGIME;
  } else if(errorVal > -MOVE_ZONE_VAL && errorVal < -ERROR_ZONE_VAL) {
    //Move the motor down at a linear rate between the (x, y) points (-MOVE_ZONE_VAL, MAX_MOTOR_PWM) and (-ERROR_ZONE_VAL, 0).
    double mappedDouble = mapDouble(errorVal, -MOVE_ZONE_VAL, -ERROR_ZONE_VAL, MAX_MOTOR_PWM, MIN_MOTOR_PWM);
                // double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
                // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    // Serial.
    // Serial.print(mappedDouble);
    moveMotor(DOWN,mappedDouble);
    // moveMotor(DOWN, 50);
    // Serial.println("DOWN regime");
    return UP_REGIME;
  } else if(errorVal > -ERROR_ZONE_VAL && errorVal < ERROR_ZONE_VAL) {
    //Stop the motor when errorVal is within a certain margin of error defined by 2 * ERROR_ZONE_VAL.
    moveMotor(NONE, 0);
    // Serial.println("NONE regime");
    return NONE_REGIME;
  } else if(errorVal > ERROR_ZONE_VAL && errorVal < MOVE_ZONE_VAL) {
    //Move the motor down at a linear rate between the (x, y) points (ERROR_ZONE_VAL, 0) and (MOVE_ZONE_VAL, MAX_MOTOR_PWM).
    double mappedDouble = mapDouble(errorVal, ERROR_ZONE_VAL, MOVE_ZONE_VAL, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
    // Serial.print(mappedDouble);
    moveMotor(UP, mappedDouble);
    // moveMotor(UP, 50);
    // Serial.println("UP regime");
    return DOWN_REGIME;
  } else {
    //Move the motor down at the max value when the error value is too positive (Safety feature to prevent excessive motor speed).
    moveMotor(UP, MAX_MOTOR_PWM);
    // Serial.println("MAX UP regime");
    return MAX_DOWN_REGIME;
  }
}
