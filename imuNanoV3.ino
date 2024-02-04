/*
  Arduino LSM6DS3 - Simple Accelerometer

  This example reads the acceleration values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM6DS3.h>

float accx, accy, accz;
float gyrox, gyroy, gyroz;

float NormX = 0.03;
float NormY = 0.01;
float NormZ = 1.03;

float threshold = 0.01;

float deltaX,deltaY,deltaZ;

float movement;
float time;
float time_1;
float time_2;
float time_3;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
}

void loop() {


  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accx,accy, accz);
    IMU.readGyroscope(gyrox, gyroy, gyroz);


    // movement = sqrt(accx*accx + accy*accy + accz*accz);
    // movement = (accx + accy + accz)/3;
    // Serial.println(movement);
    printData();
  

  }
}

void movementDetection() {

    deltaX = abs(NormX - accx);
    deltaY = abs(NormY - accy);
    deltaZ = abs(NormZ - accz);

    if (deltaX > threshold && deltaY > threshold){

      Serial.println("moving");

    } else if (deltaX > threshold && deltaZ > threshold) {

      Serial.println("moving");

    } else if (deltaZ > threshold && deltaY > threshold) {

      Serial.println("moving");

    } else {

      Serial.println("Static");

    }
}

void printData(){

    Serial.print(millis());
    Serial.print(",");
    Serial.print(accx);
    Serial.print(",");
    Serial.print(accy);
    Serial.print(",");
    Serial.print(accz);
    Serial.print(",");
    Serial.print(gyrox);
    Serial.print(",");
    Serial.print(gyroy);
    Serial.print(",");
    Serial.print(gyroz);
    Serial.println();


}