#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

//please enter your sensitive data in the Secret tab
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

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
    ;

  // attempt to connect to Wi-Fi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  Serial.println("---------------------------------------");

  while (!mqttClient.connect(broker, port)) {

    Serial.println("MQTT Connection Failed. Error Code = ");
    Serial.println(mqttClient.connectError());
  }

  Serial.println("Connected to MQTT Broker!");
  Serial.println("---------------------------------------");
}

void loop() {

  mqttClient.beginMessage(topic);
  mqttClient.print(micros());
  Serial.println(micros());
  mqttClient.endMessage();
  delay(1);


}

void checkWifi(){

  unsigned long currentMillisInfo = millis();

  // check if the time after the last update is bigger the interval
  if (currentMillisInfo - previousMillisInfo >= intervalInfo) {
    previousMillisInfo = currentMillisInfo;

    Serial.println("Board Information:");
    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print your network's SSID:
    Serial.println();
    Serial.println("Network Information:");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);
    Serial.println("---------------------------------------");
  }

}