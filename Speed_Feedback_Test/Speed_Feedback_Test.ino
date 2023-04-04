
// define the input pin
int inputPin = 9;
#define MOTOR_PWM_INPUT_PIN 5
#define ENABLE_PIN 3


void setup() {
  // set the input pin as an input
  pinMode(inputPin, INPUT);
  pinMode(MOTOR_PWM_INPUT_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_INPUT_PIN, 80);
   //Set the enable pin high.
  digitalWrite(ENABLE_PIN, HIGH);
  
  // start serial communication
  Serial.begin(9600);
}

void loop() {
  // read the duration of the pulse on the input pin
  unsigned long pulseDuration = pulseIn(inputPin, HIGH);

  // calculate the frequency of the pulse
  float frequency = 1000000.0 / pulseDuration;

  // print the current speed to the serial monitor
  Serial.print("Speed: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  // wait for a short time before taking the next measurement
  delay(1);
}
