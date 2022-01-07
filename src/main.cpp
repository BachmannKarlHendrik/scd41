#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <credentials.h>


//Wifi info and debugging led
int LED_BUILTIN = 2;

String clientId = "KarliESP-SCD41-2";
String command = "";
String TOPIC = "s/us";
unsigned long lastMsg = 0;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

const int16_t SCD_ADDRESS = 0x62;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFINAME, WIFIPASS);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  command = "100,"+clientId+",c8y_MQTTdevice";
  mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    default:
      Serial.println("Unhandled wifi event just happened!");
    }
}

void onMqttConnect(bool sessionPresent) {
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  //Start serial and LED
  Serial.begin(115200);
  while(!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.println();

  //Sensor setup
  Wire.begin();

  // start scd measurement in periodic mode, will update every 5 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();
  lastMsg = millis(); // Make sure that there is no measuring done in 5 seconds.

  //Create MQTT timers
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  //Create WIFI state listener
  WiFi.onEvent(WiFiEvent);

  //Create MQTT state methods
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(URL, 1883);
  mqttClient.setCredentials(TENANT,IOTPASS);
  mqttClient.setClientId(clientId.c_str());

  //Start connection
  connectToWifi();
}

void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 5000) { // Wait 5 seconds between every request
    lastMsg = now;
  
    float co2, temperature, humidity;
    uint8_t data[12], counter;

    // send read data command
    Wire.beginTransmission(SCD_ADDRESS);
    Wire.write(0xec);
    Wire.write(0x05);
    Wire.endTransmission();
    
    // read measurement data: 2 bytes co2, 1 byte CRC,
    // 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC,
    // 2 bytes sensor status, 1 byte CRC
    // stop reading after 12 bytes (not used)
    // other data like  ASC not included
    Wire.requestFrom(SCD_ADDRESS, 12);

    counter = 0;
    while (Wire.available()) {
      data[counter++] = Wire.read();
    }
    
    // floating point conversion according to datasheet
    co2 = (float)((uint16_t)data[0] << 8 | data[1]);
    // convert T in degC
    temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
    // convert RH in %
    humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

    Serial.print(co2);
    Serial.print("\t");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print(humidity);
    Serial.println();

    command = "200,co2Measurement,particles per million,"+String(co2)+",ppm";
    mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
    command = "211,"+String(temperature);
    mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
    command = "200,humidityMeasurement,percent,"+String(humidity)+",%";
    mqttClient.publish(TOPIC.c_str(), 0, false, command.c_str());
  }
}