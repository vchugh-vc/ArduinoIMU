/*
  Arduino LSM6DS3 - Accelerometer Application

  This example reads the acceleration values as relative direction and degrees,
  from the LSM6DS3 sensor and prints them to the Serial Monitor or Serial Plotter.

  The circuit:
  - Arduino Nano 33 IoT

  Created by Riccardo Rizzo

  Modified by Jose García
  27 Nov 2020

  This example code is in the public domain.
*/

#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>

float accx, accy, accz;
float gyrox, gyroy, gyroz;

float movement;

int degreesX = 0;
int degreesY = 0;

uint32_t LoopTimer;

BLEService customService("180C");



void setup() {
  Serial.begin(57600);
  while (!Serial);
  Serial.println("Started");

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  BLE.setLocalName("Nano IOT 33")

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  Serial.println("Time,Pitch,Roll,AccY,AccX,AccZ,GyroX,GyroY,GyroZ");

}

void loop() {

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accx, accy, accz);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(gyrox, gyroy, gyroz);
  }



  // movement = sqrt(accx*accx + accy*accy + accz*accz);
  // Serial.print(movement);
  // Serial.println();



}

void printData(){

  Serial.print(millis());
  Serial.print(",");
  Serial.print(0); // Roll Angle
  Serial.print(",");
  Serial.print(0); // Pitch Angle
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
