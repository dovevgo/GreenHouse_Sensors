/*
Mitspe Golan Sensors Module

Pins Layout

SHT20:
RED - VIN
BLUE - GND
GREEN - D21
YELLO - D22

Soil:
AO - D35
*/
#include <WiFiManager.h> 
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include "DFRobot_SHT20.h"
#include <Wire.h>
#include <FastLED.h>
#include "secrets.h"


// Define secrets
#define SENSOR_ID "1001"

// Define devices connectors
#define ONBOARD_LED 2
#define FASTLED_PIN 4
#define SOIL 35
#define NUM_LEDS 1

// Define Variables
int last_millis;
int soil_value = 0;
float temp_value = 0;
float humid_value = 0;
String message_string;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
StaticJsonDocument<100> doc;

DFRobot_SHT20 sht20;

// Defmine MQTT Variables
int port = 1883;
const char topic[]  = "/sensors";
int timeout = 90; // seconds to run for
CRGB leds[NUM_LEDS];
IPAddress ip;

void lightLed(int color){
  leds[0] = color;
  FastLED.setBrightness(30);
  FastLED.show();
}

void flashLed(int color, int flashDelay){
  lightLed(color);
  delay(flashDelay);
  lightLed(CRGB::Black);
  delay(flashDelay);    
}

void blink_led(int blink_delay){
  digitalWrite(ONBOARD_LED, HIGH);
  delay(blink_delay);
  digitalWrite(ONBOARD_LED, LOW);
  delay(blink_delay);
}

void reconnect_mqtt(){
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(MQTT_BROKER);
  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS);
  if (!mqttClient.connect(MQTT_BROKER, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    delay(1000);
  } else {
    Serial.println("You're connected to the MQTT broker!");
  }  
}
void send_mqtt_message(String target_topic, String msg){
  mqttClient.beginMessage(target_topic);
  mqttClient.print(msg);
  mqttClient.endMessage();
}

void setup() {
  pinMode(ONBOARD_LED, OUTPUT); // Define on-board led pin
  pinMode(SOIL, INPUT);
  Serial.begin(9600);
  FastLED.addLeds<WS2852, FASTLED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  flashLed(CRGB::Orange, 750);
  
  sht20.initSHT20();                         // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20();   

  last_millis = millis();
  //Initialize Wifi Manager
  WiFiManager wm;
  WiFi.mode(WIFI_STA); // exp
  lightLed(CRGB::LightBlue);
  wm.setConfigPortalTimeout(timeout);
  wm.setWiFiAutoReconnect(true);
  if (!wm.startConfigPortal("Sensors ESP32", "PassWord")) {
    Serial.println("No Client connected to Portal");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
  }
  Serial.print("Conntecting to ");
  Serial.println(wm.getWiFiSSID());
  lightLed(CRGB::OrangeRed);
  reconnect_mqtt();

}

void loop() {
 if (WiFi.status() == WL_DISCONNECTED) {
    lightLed(CRGB::LightBlue);
    WiFi.begin();
    delay(2000);
    Serial.println("Connected to wifi");
    ip = WiFi.localIP();
    Serial.println(ip);
    lightLed(CRGB::OrangeRed);
    reconnect_mqtt();
  } else if (WiFi.status() == WL_CONNECTED) {
    // Read Sensors
    flashLed(CRGB::LightBlue, 150);
    soil_value = map(analogRead(SOIL), 0, 4095, 100, 0); // Read soil moist and map values between 0 (dry) and 100 (wet)
    humid_value = sht20.readHumidity();         // Read Humidity
    temp_value = sht20.readTemperature();       // Read Temperatur
    // Send MQTT message
    lightLed(CRGB::Green);
    doc.clear();
    doc["time"] = millis();
    doc["sensor"] = SENSOR_ID;
    JsonArray data = doc.createNestedArray("data");
    data.add(soil_value);
    Serial.println(soil_value);
    data.add(round(temp_value));
    data.add(round(humid_value));
    message_string = "";
    serializeJson(doc, message_string);
    Serial.println(message_string);
    char target_topic[24];
    strcpy(target_topic, topic);
    strcat(target_topic, "/");
    strcat(target_topic, SENSOR_ID);
    send_mqtt_message(target_topic, message_string);
    lightLed(CRGB::Black);
  }
}

