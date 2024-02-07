#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <math.h>

char ssid[] = "VM8679854";             // your network SSID (name)
char pass[] = "j2qDcMnzmxyf";          // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;           // the Wi-Fi radio's status                    
unsigned long previousMillisInfo = 0;  //will store last time Wi-Fi information was updated
const int intervalInfo = 5000;         // interval at which to update the board information

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "test.mosquitto.org";
int port = 1883;
const char topic[] = "VarunChugh1";

float accx, accy, accz;
float gyrox, gyroy, gyroz;
float AngleRoll, AnglePitch;

float NormX = 0.03;
float NormY = 0.01;
float NormZ = 1.03;

float threshold = 0.02;

float deltaX,deltaY,deltaZ;

float movement;
float time;
float time_1;
float time_2;
float time_3;


void setup() {
  // Serial.begin(9600);
  // while (!Serial);

  if (!IMU.begin()) {
    // Serial.println("Failed to initialize IMU!");

    while (1);
  }

  while (status != WL_CONNECTED) {
    // Serial.print("Attempting to connect to network: ");
    // Serial.println(ssid);
    // // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  // Serial.println("You're connected to the network");
  // Serial.println("---------------------------------------");

  while (!mqttClient.connect(broker, port)) {

    // Serial.println("MQTT Connection Failed. Error Code = ");
    // Serial.println(mqttClient.connectError());
  }
  mqttClient.print("Connected");
  // Serial.println("Connected to MQTT Broker!");
  // Serial.println("---------------------------------------");

  // Serial.print("Accelerometer sample rate = ");
  // Serial.print(IMU.accelerationSampleRate());
  // Serial.println(" Hz");
  // Serial.println();
  // Serial.println("Acceleration in g's");
  // Serial.println("X\tY\tZ");
}

void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accx,accy, accz);
    IMU.readGyroscope(gyrox, gyroy, gyroz);


      AngleRoll=atan(accy/sqrt(accx*accx+accz*accz))*1/(3.142/180);
      AnglePitch=-atan(accx/sqrt(accy*accy+accz*accz))*1/(3.142/180);

      // mqttClient.beginMessage(topic);
      // mqttClient.print(accx);
      // mqttClient.print(",");
      // mqttClient.print(accy);
      // mqttClient.print(",");
      // mqttClient.print(accz);
      // mqttClient.print(",");
      // mqttClient.print(gyrox);
      // mqttClient.print(",");
      // mqttClient.print(gyroy);
      // mqttClient.print(",");
      // mqttClient.print(gyroz);
      // mqttClient.endMessage();

    // movement = sqrt(accx*accx + accy*accy + accz*accz);
    // movement = (accx + accy + accz)/3;
    // Serial.println(movement);
    // printData();
    movementDetection();
  

  }
}

void movementDetection() {

    deltaX = abs(NormX - accx);
    deltaY = abs(NormY - accy);
    deltaZ = abs(NormZ - accz);

    if (deltaX > threshold && deltaY > threshold){

      //  Serial.println("moving");
      mqttClient.beginMessage(topic);
      mqttClient.print(accx);
      mqttClient.print(",");
      mqttClient.print(accy);
      mqttClient.print(",");
      mqttClient.print(accz);
      mqttClient.print(",");
      mqttClient.print(gyrox);
      mqttClient.print(",");
      mqttClient.print(gyroy);
      mqttClient.print(",");
      mqttClient.print(gyroz);
      mqttClient.print(",");
      mqttClient.print(AngleRoll);
      mqttClient.print(",");
      mqttClient.print(AnglePitch);
      mqttClient.endMessage();

    } else if (deltaX > threshold && deltaZ > threshold) {

      // Serial.println("moving");
      mqttClient.beginMessage(topic);
      mqttClient.print(accx);
      mqttClient.print(",");
      mqttClient.print(accy);
      mqttClient.print(",");
      mqttClient.print(accz);
      mqttClient.print(",");
      mqttClient.print(gyrox);
      mqttClient.print(",");
      mqttClient.print(gyroy);
      mqttClient.print(",");
      mqttClient.print(gyroz);
      mqttClient.print(",");
      mqttClient.print(AngleRoll);
      mqttClient.print(",");
      mqttClient.print(AnglePitch);
      mqttClient.endMessage();

    } else if (deltaZ > threshold && deltaY > threshold) {

      // Serial.println("moving");
      mqttClient.beginMessage(topic);
      mqttClient.print(accx);
      mqttClient.print(",");
      mqttClient.print(accy);
      mqttClient.print(",");
      mqttClient.print(accz);
      mqttClient.print(",");
      mqttClient.print(gyrox);
      mqttClient.print(",");
      mqttClient.print(gyroy);
      mqttClient.print(",");
      mqttClient.print(gyroz);
      mqttClient.print(",");
      mqttClient.print(AngleRoll);
      mqttClient.print(",");
      mqttClient.print(AnglePitch);
      mqttClient.endMessage();

    } else {

      //Serial.println("Static");

    }
}

// void printData(){

//     Serial.print(millis());
//     Serial.print(",");
//     Serial.print(accx);
//     Serial.print(",");
//     Serial.print(accy);
//     Serial.print(",");
//     Serial.print(accz);
//     Serial.print(",");
//     Serial.print(gyrox);
//     Serial.print(",");
//     Serial.print(gyroy);
//     Serial.print(",");
//     Serial.print(gyroz);
//     Serial.println();


// }