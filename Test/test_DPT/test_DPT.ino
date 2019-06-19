const uint8_t  pinPWM  = 3;
const uint8_t  pinEN  = 4;
const uint8_t  pinInA  = 5;
const uint8_t  pinInB  = 6;

void setup() {
  pinMode(pinPWM, OUTPUT);
  digitalWrite(pinEN, HIGH);
  analogWrite(pinPWM, 64);
}

void loop() {
    digitalWrite(pinInA, HIGH);
    digitalWrite(pinInB, LOW);
    delay(2000);
    digitalWrite(pinInA, LOW);
    digitalWrite(pinInB, HIGH);
    delay(2000);
}
