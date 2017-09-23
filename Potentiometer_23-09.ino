/*

send id, value
map to degrees

possibly code in bytes - only 0-255 range

 */
#include <ros.h>

const int sensorNumber = 5;
int sensorPins[sensorNumber] = {A0, A1, A2, A3, A4};

int sensor1Value = 0;
int sensor2Value = 0;
int sensor3Value = 0;
int sensor4Value = 0;
int sensor5Value = 0;
int servo1Position = sensor1Value;
int servo2Position = sensor2Value;
int servo3Position = sensor3Value;
int servo4Position = sensor4Value;
int servo5Position = sensor5Value;
int DataSet[10] = {1,0,2,0,3,0,4,0,5,0};

void setup() {
  Serial.begin(9600);
  }

void loop() {

sensor1Value = analogRead(sensorPins[0]);
sensor2Value = analogRead(sensorPins[1]);
sensor3Value = analogRead(sensorPins[2]);
sensor4Value = analogRead(sensorPins[3]);
sensor5Value = analogRead(sensorPins[4]);

servo1Position = map(sensor1Value, 0, 1024, 0, 360);
servo2Position = map(sensor2Value, 0, 1024, 0, 360);
servo3Position = map(sensor3Value, 0, 1024, 0, 360);
servo4Position = map(sensor4Value, 0, 1024, 0, 360);
servo5Position = map(sensor5Value, 0, 1024, 0, 360);

DataSet[1] = servo1Position;
DataSet[3] = servo2Position;
DataSet[5] = servo3Position;
DataSet[7] = servo4Position;
DataSet[9] = servo5Position;


for(int i = 0; i <= 9; i++)
{
  Serial.println(DataSet[i]);
}

Serial.println();
Serial.println();
Serial.println();
// delay(2000);


/*
  if
  {
  Serial.println(sensorPins[0]);
  sensorValue = analogRead(sensorPini);
  Serial.println(sensorValue);
  }
  */
}
