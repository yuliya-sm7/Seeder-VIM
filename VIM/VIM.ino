#include <EEPROMex.h>
#include <math.h>

const int param_count = 5;
char* param[param_count] = {"CELLS", "GAP", "PLOT", "DIAMETR", "MICROSTEP"};  // диаметр колеса, промежуток, делянка, количество ячеек в касете
int value[param_count];
const int addr[param_count] = {0, 2, 4, 6, 8};

const uint8_t  pinDebug  = 13;
const uint8_t  pinTable  = 0;
const uint8_t  pinDispenser  = 0;
const uint8_t  pinEncoder  = 21;

const int BUFF  = 128;

float  ENCODER_STEP  =  0;  // шаг одного импульса энкодера, см
float  MOTOR_STEP  = (M_PI * 122) / 200;
int step_count = 0;
int cells_count = 0;  // счетчик оставшихся ячеек
long long L = 0;  // суммарное пройденное расстояние, см
float distance = 0;  // расстояние от предыдущего высева, см
float distanceL = 0;
float distanceR = 0;
bool fl_run = false;

void setup() {
  attachInterrupt(2, interruptEncoderL, RISING);  // pin 21 по фронту
  attachInterrupt(3, interruptEncoderR, RISING);  // pin 20 по фронту
  Serial2.begin(9600);  // display
  Serial3.begin(9600, SERIAL_8E1);  // RS-485
  Serial.begin(9600);  // debug
  pinMode(pinDebug, OUTPUT);
  pinMode(pinDispenser, OUTPUT);

  for (int i = 0; i < param_count; ++i) {
    value[i] = EEPROM.readInt(addr[i]);
  }
  ENCODER_STEP  =  (M_PI * value[3]) / 100;
  Serial.println("Init");
  Serial3.print(":01EN\r\n"); 
}

void loop() {
  if (fl_run) {
    if (distance >= value[1] + value[2]) {
      digitalWrite(pinDispenser, HIGH);
      delay(2000);
      digitalWrite(pinDispenser, LOW);
      L += distance;
      distanceL -= distance;
      distanceR -= distance;
      distance = 0;
      cells_count--;
      if (cells_count <= 0) {
        Serial2.print((String)"page 3" + char(255) + char(255) + char(255));
        fl_run = false;
      }
      step_count = 0;
      Serial2.print((String)"n0.val=" + cells_count + char(255) + char(255) + char(255));
    }
  }
}

int receive(HardwareSerial &port, char *message) {
  int n = 0;
  do {
    message[n] = port.read();
    delay(5);
    n++;
  } while (port.available() && n < (BUFF - 1) && message[n - 1] != '\0');
  message[n] = '\0';
  //Serial.println(message);
  return n;
}

int readNum(char* message, int p) {
  char value[4];
  value[0] = message[p];
  value[1] = message[p + 1];
  value[2] = message[p + 2];
  value[3] = message[p + 3];
  uint32_t result = *(uint32_t*) value;
  //Serial.println((int)result);
  return (int)result;
}

void serialEvent2() {  // display
  char message[BUFF];
  receive(Serial2, message);
  if (!fl_run) {
    if (strstr(message, "ON")) {
      fl_run = true;
    } else if (!setConfig(message)) {
      DriverManual(message);
    }
  } else if (strstr(message, "OFF")) {
    fl_run = false;
  } else {
    Serial.println("Invalid command");
  }
}

void serialEvent3() {  // rs485
  char message[BUFF];
  receive(Serial3, message);
  Serial.println(message);
  if (strstr(message, "E16")) {
    Serial.println("Stepper Motor Error");
  }
}

bool setConfig(char * message) {
  for (int i = 0; i < param_count; ++i) {
    if (strstr(message, param[i])) {
      value[i] = readNum(strstr(message, param[i]), strlen(param[i]));
      EEPROM.updateInt(addr[i], value[i]);
      Serial.print("write "); Serial.print(param[i]); Serial.println(value[i]);
      return true;
      if (param[i] == "CELLS") {
        cells_count = value[i];
        delay(100);
        Serial2.print((String)"n0.val=" + cells_count + char(255) + char(255) + char(255));
      } else if (param[i] == "DIAMETR") {
        ENCODER_STEP = (M_PI * value[i]) / 100;
      }
    }
  }
  return false;
}

void DriverManual(char * message) {
  if (strstr(message, "STOP")) {
    Serial3.print(":01SP\r\n"); Serial.println("stop");
    return;
  }
  int sp = 0;
  if (strstr(message, "RIGHT")) {
    sp = readNum(strstr(message, "RIGHT"), strlen("RIGHT"));
    Serial3.print(":01DR\r\n"); Serial.println("moveR");
  } else if (strstr(message, "LEFT")) {
    sp = readNum(strstr(message, "LEFT"), strlen("LEFT"));
    Serial3.print(":01DL\r\n"); Serial.println("moveL");
  } else return;
  delay(200);
  Serial.println(sp);
  Serial3.print(":01SD"); Serial3.print(sp); Serial3.print("\r\n");
  delay(200);
  Serial3.print(":01MV1000\r\n");
}

void interruptEncoderR() {
  if (fl_run) {
    increment(distanceR);
  }
}
void interruptEncoderL() {
  if (fl_run) {
    increment(distanceL);
  }
}

void increment(float &dist) {
  dist += ENCODER_STEP;
  distance = min(distanceL, distanceR);
  Serial.println(distance);
  while (distance > step_count * MOTOR_STEP) {
    Serial3.print(":01MV1\r\n");
    step_count++;
  }
  int p = L + distance;
  Serial2.print((String)"n1.val=" + p + char(255) + char(255) + char(255));
}
