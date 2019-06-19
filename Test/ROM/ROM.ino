void setup () {
  Serial.begin ( 9600 );
  Serial2.begin ( 9600 );
}

void loop () {
}

void serialEvent() {
  while (Serial.available()) {
    Serial2.write(Serial.read());
  }
}

void serialEvent2() {
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
