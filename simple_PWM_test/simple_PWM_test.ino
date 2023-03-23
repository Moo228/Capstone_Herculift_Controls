#define PWM_PIN 5

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PWM_PIN, 128);
}
