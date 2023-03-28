#define PWM_PIN 5
#define ENABLE_PIN 3

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  // analogWrite(PWM_PIN, 0);     // Test 1, 8, 
  // analogWrite(PWM_PIN, 26);    // Test 2
  // analogWrite(PWM_PIN, 30);    // Test ##
  // analogWrite(PWM_PIN, 51);    // Test 3
  // analogWrite(PWM_PIN, 77);    // Test 4
  // analogWrite(PWM_PIN, 128);   // Test 5
  // analogWrite(PWM_PIN, 191);   // Test 6
  analogWrite(PWM_PIN, 255);   // Test 7
}
