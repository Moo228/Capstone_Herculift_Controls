// Set the pin that will be used to read analog input
const int inputPinLoad = A2;
const int inputPinTension = A3;

void setup() {
  Serial.begin(9600); // Initialize the serial port
  analogReference(INTERNAL); // Set the internal reference voltage to 1.1V
}

void loop() {
  unsigned long startTime = millis(); // Record the start time

  // Loop for the specified sample duration
  float inputValueLoad = analogRead(inputPinLoad)/1023.0;
  float inputValueTension = analogRead(inputPinTension)/1023.0;
  // Serial.print("Input voltage: ");
  Serial.print("Load: ");
  Serial.print(inputValueLoad, 3); // Print the inputValue with 3 decimal places
  Serial.print(", ");
  Serial.print("Tension: ");
  Serial.println(inputValueTension, 3); // Print the inputValue with 3 decimal places
  delayMicroseconds(100); // Wait for 100us to increase the sampling rate

}
