const uint8_t  pinEncoderL  = 21;
const uint8_t  pinEncoderR  = 20;

void setup() {
  Serial.begin(9600);  // debug
  attachInterrupt(2, interruptEncoderL, RISING);  // pin 21 по фронту
  attachInterrupt(3, interruptEncoderR, RISING);  // pin 20 по фронту
}

void loop() {
  Serial.print("L = ");
  Serial.println(analogRead(pinEncoderL));
  Serial.print("R = ");
  Serial.println(analogRead(pinEncoderR));
  delay(50);
  
}

void interruptEncoderR() {
  Serial.print("interruptL = ");
  Serial.println(analogRead(pinEncoderL));
}
void interruptEncoderL() {
  Serial.print("interruptR = ");
  Serial.println(analogRead(pinEncoderR));
}
