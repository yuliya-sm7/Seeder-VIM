#include <EEPROMex.h>

const int param_count = 4;
char* param[param_count] = {"DIAMETR", "GAP", "PLOT", "CELLS"};
int value[param_count];
const int addr[param_count] = {0, 2, 4, 6};

const uint8_t  pinDebug  = 13;
const uint8_t  pinTable  = 0;
const uint8_t  pinDispenser  = 0;
const uint8_t  pinEncoder  = 21;

const int BUFF  = 128;
const int MICROSTEP  = 16;

int  ENCODER_STEP  = 20;  // см
int cell_count = 0;
long long L = 0;  // пройденное расстояние, см
long gap = 0;
bool fl_run = false;

void setup() {
  attachInterrupt(2, interruptHall, RISING);  // pin 21 по фронту
  Serial2.begin(9600);  // display
  Serial3.begin(9600);  // RS-485
  Serial.begin(9600);  // debug
  pinMode(pinDebug, OUTPUT);
  pinMode(pinDispenser, OUTPUT);
  
  value[0] = EEPROM.readInt(addr[0]);  // диаметр колеса
  value[1] = EEPROM.readInt(addr[1]);  // промежуток
  value[2] = EEPROM.readInt(addr[2]);  // делянка
  value[3] = EEPROM.readInt(addr[3]);  // количество ячеек в касете

  Serial.println("Init");
}

void loop() {
  if (gap >= value[1]) {
    digitalWrite(pinDispenser, HIGH);
    delay(2000);
    digitalWrite(pinDispenser, LOW);
    gap = 0;
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
    if (strstr(message, "RUN")) {
      fl_run = true;
    }else if (!setConfig(message)) {
      DriverManual(message);
    }
  }
}

void serialEvent3() {  // rs485
  char message[BUFF];
  receive(Serial3, message);
  if (strstr(message, "E16")) {
    Serial.println("Error");
  }
}

bool setConfig(char * message) {
  for (int i = 0; i < 4; ++i) {
    if (strstr(message, param[i])) {
      value[i] = readNum(strstr(message, param[i]), strlen(param[i]));
      EEPROM.updateInt(addr[i], value[i]);
      Serial.print("write ");
      Serial.println(param[i]);
      Serial.println(value[i]);
      return true;
    }
  }
  return true;
}

void DriverManual(char * message) {
  if (strstr(message, "FORVARD")) {
    int sp = readNum(strstr(message, "FORVARD"), strlen("FORVARD"));
    Serial3.write(":01SD");
    Serial3.write(sp);
    Serial3.write("\r\n");
    Serial3.write(":01MV\r\n");
  } else if (strstr(message, "STOP")) {
    Serial3.write(":01 \r\n");
  }
}

void interruptHall() {
  if (fl_run) {
    L += ENCODER_STEP;
    gap += ENCODER_STEP;
  }
}
/*void DriverInit() {
  Serial3.write(":01AL200\r\n"); // ускорение
  Serial3.write(":01SD300\r\n"); // скорость
  Serial3.write(":01MV100\r\n"); // 100 шагов
}*/
