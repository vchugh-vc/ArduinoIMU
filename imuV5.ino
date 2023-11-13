#include <Wire.h>
#include <math.h>


float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // Calibrating Gyroscope
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float KalmanOutput[]={0,0}; // {Prediction, Uncertainty of Prediction}


void KalmanFilter(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  
  // Input: Rotation Rate (Gyro). Measurement: Accelerometer Angle (Trig). State: Angle Calculated using the Filter
  
  KalmanState = KalmanState + 0.008 * KalmanInput; // Iteration Length of 0.004
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 16; // Estimate Variance of 16
  float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + 9); // Kalman Gain Equation with Measurement Variance of 9
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState); // State Update Equation
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty; // Covariance Update Equation

  KalmanOutput[0] = KalmanState; 
  KalmanOutput[1] = KalmanUncertainty;

}

void imuSignals(void) {

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;

  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;

  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(800000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  gyroCalibration();


}

void loop() {

  imuSignals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch; // Move to Calibration Function?
  RateYaw -= RateCalibrationYaw;

  KalmanFilter(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=KalmanOutput[0]; 
  KalmanUncertaintyAngleRoll=KalmanOutput[1];

  KalmanFilter(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=KalmanOutput[0]; 
  KalmanUncertaintyAnglePitch=KalmanOutput[1];
  

  printData();

  // Serial.print(" ");
  // Serial.print(LoopTimer);
  // Serial.println();



  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}

void printData(){

  // Serial.print("Roll Angle  ");
  // Serial.print(" ");
  Serial.print(KalmanAngleRoll);
  // Serial.print(" Pitch Angle ");
  // Serial.print(" ");
  Serial.print(",");
  Serial.print(KalmanAnglePitch);
  // Serial.print(" ");
  // Serial.print(20);
  // Serial.print(" ");
  // Serial.print(-20);
  Serial.println();

}

void gyroCalibration(){


  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {

    imuSignals();

    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();

}


