//Per the Arduino nano documentation pins 3, 5, 6, 9, 10, and 11 support pulse width modulation (PWM).
//The PWM pins 5 and 6 are connected to a 16-bit timer and run at a default frequency of 976.56Hz.
//The PWM pins 3, and 11 are connected to Timer 0? in the ATmega168p microcontroller chip.
//The PWM pins 9, and 10 are connected to Timer 2? in the ATmega168p microcontroller chip.
//This will be important for modifying the default frequencies supported by the PWM pins (controlled by the timers).

#define MOTOR_FORWARD_PIN 5
#define MOTOR_BACKWARD_PIN 6
#define LOAD_DT 2
#define LOAD_SCK 3

// Include HX711 Arduino Library (Bogdan Necula & Andreas Motl) for load cell
#include "HX711.h"

//Create an enum to simplify the names for handle motion.
enum HandleMotion { UP,
                   DOWN,
                   NONE };

//Function declarations

//This function wraps the logic of motor control into a simple command.
//Parameters:
//  direction- uses the HandleMotion enum to specify UP, DOWN, or NONE.
//  dutyCycle- specify the duty cycle to run the motors at.
void moveHandle(HandleMotion direction, int dutyCycle);

// Define variables for load cell reading & calibration factor
double reading; // load cell value
double timeVar; // variable to track time
float calibration_factor = -8.5e4;

// Object for scale
HX711 scale;

void setup() {
  //Set the pins as output.
  //These pins will control the motors by interfacing with the L298N Motor controller.
  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {
   ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialize communication with scale
  scale.begin(LOAD_DT, LOAD_SCK);

  // Zero scale when first starting
  scale.tare();

  // Apply calibration factor to scale and print to serial monitor
  scale.set_scale(calibration_factor);
  Serial.print("Calibration factor: ");
  Serial.println(calibration_factor);
}

void loop() {
  //Create a simple sketch to control the motor.
  // moveHandle(UP, 255);
  // delay(2000);
  // moveHandle(DOWN, 255);
  // delay(2000);
  // moveHandle(NONE, 0);
  // delay(2000);

  // Set reference time
  timeVar = micros()/1e6;

  // Get reading from scale and print to serial monitor
  reading = -scale.get_units(10);
  // Serial.print("dt (s): ");
  // Serial.print(micros()/1e6 - timeVar, 6);
  // Serial.print(" Load (g): ");
  Serial.println(reading*1000, 12);
}

//Function definitions

void moveHandle(HandleMotion direction, int dutyCycle) {
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