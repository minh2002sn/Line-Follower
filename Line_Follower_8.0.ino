#include <QTRSensors.h>
#include <EEPROM.h>
#include "Motor.h"

/*
 * EEPROM memory accesses from 0 to 15 store minimum value of each sensor, 2 byte per sensor
 * EEPROM memory accesses from 16 to 31 store maximum value of each sensor, 2 byte per sensor
 * EEPROM memory access 32 stores 0 or 1 meaning the line is black or white color
 * EEPROM memory access from 33 to 35 store KP, KI, KD value.
 * EEPROM memory access 36 stores normal speed value.
 */

#define           BUTTON_PIN      12
#define           IN1             2
#define           IN2             4
#define           ENA             3
#define           IN3             5
#define           IN4             7
#define           ENB             6
#define           NORMAL_POSITION 3500
#define           COMPARE_VALUE   500
#define           MAX_SPEED       255
#define           MAX_ADJUSTMENT  100

int               NORMAL_SPEED=   100;
int               KP          =   35;           //  0.9,  1.0       |   1.0
int               KD          =   25;           //  15.0, 30.0      |   10.0
float             KI          =   0;
float             Ki          =   0;

bool              lineColor   =   0;
bool              noLine      =   0;
bool              lastPos     =   0;

int               adjustment  =   0;

//uint16_t minimumOn[8] = {40, 40, 40, 40, 40, 40, 40, 40};
//uint16_t maximumOn[8] = {800, 800, 800, 800, 800, 800, 800, 800};

QTRSensors qtr;

Motor M1(IN1, IN2, ENA, 255);
Motor M2(IN3, IN4, ENB, 255);

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int Combo[4][4] = {
  {50, 50, 180, 80},    // Combo[0][1]
  {65, 50, 200, 100},
  {90, 80, 250, 120},
  {95, 90, 255, 150}
};

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(8);

//  EEPROM.write(32, 1);
//  delay(5);
//  for (uint8_t i=0; i < SensorCount; i++){
//    EEPROM.write(i * 2, (minimumOn[i]));
//    delay(5);
//    EEPROM.write(i * 2 + 1, minimumOn[i] >> 8);
//    delay(5);
//    EEPROM.write(i * 2 + 16 , (maximumOn[i]));
//    delay(5);
//    EEPROM.write(i * 2 + 17 , maximumOn[i] >> 8);
//    delay(5);
//  }

  if(!digitalRead(BUTTON_PIN)){
    Serial.println("Calibrating...");
    QTR_calibration();
    for (uint8_t i=0; i < SensorCount; i++){
      EEPROM.write(i * 2, qtr.calibrationOn.minimum[i]);
      delay(5);
      EEPROM.write(i * 2 + 1, (qtr.calibrationOn.minimum[i] >> 8));
      delay(5);
      EEPROM.write(i * 2 + 16 , qtr.calibrationOn.maximum[i]);
      delay(5);
      EEPROM.write(i * 2 + 17 , qtr.calibrationOn.maximum[i] >> 8);
      delay(5);
    }
    Serial.println("Calibrated");
  } else{
    Serial.println("No calibrate");
    qtr.calibrationOn.initialized = true;
    qtr.calibrationOn.minimum = new uint16_t[SensorCount];
    qtr.calibrationOn.maximum = new uint16_t[SensorCount];
    for(int i = 0; i < SensorCount; i++){
      uint16_t low = EEPROM.read(i * 2);
      delay(5);
      uint16_t high = EEPROM.read(i * 2 + 1);
      delay(5);
      qtr.calibrationOn.minimum[i] = (high << 8) | low;
      low = EEPROM.read(i * 2 + 16);
      delay(5);
      high = EEPROM.read(i * 2 + 17);
      delay(5);
      qtr.calibrationOn.maximum[i] = (high << 8) | low;
    }
  }
  lineColor = EEPROM.read(32);
  delay(5);
  KP = EEPROM.read(33);
  delay(5);
  KI = EEPROM.read(34) / 100.0;
  delay(5);
  KD = EEPROM.read(35);
  delay(5);
  NORMAL_SPEED = EEPROM.read(36);
  delay(5);

  print_calib();
  
  uint8_t state = 1, lastState = 1, count = 0;
  while(count != 2){
    state = digitalRead(BUTTON_PIN);
    if(state != lastState){
      count++;
      lastState = state;
      delay(50);
    }
    if(Serial.available() > 0){
      char data = Serial.read();
      if(data == 'B'){
        EEPROM.write(32, 0);
        lineColor = 0;
      } else if(data == 'W'){
        EEPROM.write(32, 1);
        lineColor = 1;
      } else if(data == 'P' || data == 'I' || data == 'D' || data == 'S' || data == 'C'){
        int value = 0;
        delay(100);
        while(Serial.available() > 0){
          char temp = Serial.read();
          if(temp >= '0' && temp <= '9'){
            value = value * 10 + (temp - '0');
          }
        }
        Serial.println(value);
        if(data == 'P'){
          KP = value;
          EEPROM.write(33, value);
          delay(5);
        } else if(data == 'I'){
          KI = value / 100.0;
          EEPROM.write(34, value);
          delay(5);
        } else if(data == 'D'){
          KD = value;
          EEPROM.write(35, value);
          delay(5);
        } else if(data == 'S'){
          NORMAL_SPEED = value;
          EEPROM.write(36, value);
          delay(5);
        } else if(data == 'C'){
          KP = Combo[value][0];
          KI = Combo[value][1] / 100.0;
          KD = Combo[value][2];
          NORMAL_SPEED = Combo[value][3];
          EEPROM.write(33, Combo[value][0]);
          delay(5);
          EEPROM.write(34, Combo[value][1]);
          delay(5);
          EEPROM.write(35, Combo[value][2]);
          delay(5);
          EEPROM.write(36, Combo[value][3]);
          delay(5);
        }
      }
      print_calib();
    }
  }
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);

}

void loop() {
  adjustment = PID_calculation(get_sensor_value());
  M1.move(NORMAL_SPEED - adjustment);
  M2.move(NORMAL_SPEED + adjustment);
//  Serial.println();
}



void QTR_calibration(){
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 100; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}



uint16_t get_sensor_value(){
  bool lineMap[SensorCount] = {};
  noLine = 1;

  qtr.readLineBlack(sensorValues);

  for(int i = 0; i < SensorCount; i++){
    if(lineColor){
      sensorValues[i] = 1000 - sensorValues[i];
    }
    if(sensorValues[i] > COMPARE_VALUE){
      lineMap[i] = 1;
      noLine = 0;
    }
  }

  if(noLine){
    if(lastPos){
      lineMap[SensorCount - 1] = 1;
    } else{
      lineMap[0] = 1;
    }
  }

  if((lineMap[0] || lineMap[SensorCount - 1]) && lineMap[1] == 0  && lineMap[2] == 0
        && lineMap[3] == 0  && lineMap[4] == 0 && lineMap[5] == 0 && lineMap[6] == 0){
    Ki = KI;
  } else{
    Ki = 0;
  }

  uint16_t sumLine = 0;
  uint16_t sumPos = 0;
  for(int i = 0; i < SensorCount; i++){
    sumLine += lineMap[i];
    sumPos += i*1000*lineMap[i];
  }
  uint16_t position = sumPos/sumLine;

  lastPos = (position > 3500) ? 1 : 0;

//  for (uint8_t i = 0; i < SensorCount; i++){
//    Serial.print(lineMap[i]);
//    Serial.print(' ');
//  }
//  Serial.println();

  return position;
}



int PID_calculation(uint16_t position){
  static float I = 0;
  static float lastError = 0;
  
  float error = (3500 - (int)position) / 1000.0;

  float P = error;
  if(Ki == 0){
    I = 0;
  } else{
    I += error;
  }
  float D = error - lastError;
  lastError = error;

  int adjustment = (KP*P) + (Ki*I) + (KD*D);

//  Serial.print(position);
//  Serial.print(' ');
//  Serial.print(error);
//  Serial.print(' ');
//  Serial.print(adjustment);
//  Serial.print(' ');
//  Serial.print(P);
//  Serial.print(' ');
//  Serial.print(I);
//  Serial.print(' ');
//  Serial.println();

  return adjustment;
}


void print_calib(){
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Line color is ");
  Serial.println((lineColor) ? "white (1)." : "black (0).");

  Serial.print("KP = ");
  Serial.print(KP);
  Serial.print(" KI = ");
  Serial.print(KI);
  Serial.print(" KD = ");
  Serial.println(KD);

  Serial.print("NORMAL_SPEED = ");
  Serial.print(NORMAL_SPEED);

  Serial.println();

//  for (uint8_t i = 0; i < SensorCount; i++){
//    Serial.print(qtr.calibrationOff.minimum[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
//
//  for (uint8_t i = 0; i < SensorCount; i++){
//    Serial.print(qtr.calibrationOff.maximum[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
}
