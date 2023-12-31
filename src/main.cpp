#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define LED 2

#include "Adafruit_BME280.h"

#define BME280_ADDRESS         0x76

#define batVoltagePin A0

#define SLEEP_FOR_MINUTES 10

Adafruit_BME280 bme;

float humidity, temperature, pressure, bat_voltage = 0;
int avg_cycles = 0;

// Change this to point to your Wifi Credentials
const char *ssid = "<your-wifi>";
const char *password = "<your-wifi-password>";
// Your MQTT broker ID
const char *mqttBroker = "192.168.xx.xx"; // IP address of your MQTT broker
const char *mqttUser = "";                // user name and password for MQTT
const char *mqttPassword = "";

const int mqttPort = 1883;

// MQTT topics
String publishTopic = "weather" + String(ESP.getChipId());

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

String mqttName = "Weather sensor";
String mqttModel = "SolarSense";
String stateTopic = publishTopic + String("/state");


void readAvgSensorValues();

// Connect to Wifi
bool setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("Signal strength: ");
  Serial.println(WiFi.RSSI());

  int connect_timeout = 10; // if it can't connect in 10 seconds give up
  while ((WiFi.status() != WL_CONNECTED) && (connect_timeout>0))
  {
    // read sensor while waiting for wifi
    for (int i=0; i<10; i++) {
      delay(100);
      readAvgSensorValues();
      Serial.printf("RSSI: %i \t Temp: %.1f \t Hum: %.1f \t Pressure: %.1f \t Bat: %.2f\n", WiFi.RSSI(), temperature/avg_cycles, humidity/avg_cycles, pressure/avg_cycles, bat_voltage/avg_cycles);
    }

    Serial.printf("Status %i time-out %i", WiFi.status(), connect_timeout);
    connect_timeout--;
  }

  if (connect_timeout==0) {
    return false; // WiFi connection failed
  }
  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}


void reconnect()
{
  // Loop until we're reconnected
  int retries = 3;
  while (!client.connected() and retries >0)
  {
    Serial.print("Attempting MQTT connection...");

    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) 
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second ");
      Serial.print(retries);
      // Wait 5 seconds before retrying
      delay(1000);
    }
    retries--;
  }
}



void sendMQTTTemperatureDiscoveryMsg() {
  String discoveryTopic = String("homeassistant/sensor/")+publishTopic+String("/temperature/config");

  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Temperature";
  doc["stat_t"]   = stateTopic;
  doc["state_class"]= "measurement";
  doc["unit_of_meas"] = "°C";
  doc["device_class"] = "temperature";
  doc["frc_upd"] = true;
  doc["value_template"] = "{{ value_json.temperature|default(0) }}";
  doc["uniq_id"] = publishTopic+String("_temperature");

  doc["device"]["identifiers"][0] = publishTopic;
  doc["device"]["name"] = mqttName;
  doc["device"]["model"] = mqttModel;
  doc["device"]["manufacturer"] = "FX";
  
  size_t n = serializeJson(doc, buffer);
  client.publish(discoveryTopic.c_str(), buffer, n);

}

void sendMQTTHumidityDiscoveryMsg() {
  String discoveryTopic = String("homeassistant/sensor/")+publishTopic+String("/humidity/config");

  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Humidity";
  doc["stat_t"]   = stateTopic;
  doc["state_class"]= "measurement";

  doc["unit_of_meas"] = "%";
  doc["device_class"] = "humidity";
  doc["frc_upd"] = true;
  doc["value_template"] = "{{ value_json.humidity|default(0) }}";
  doc["uniq_id"] = publishTopic+String("_humidity");

  doc["device"]["identifiers"][0] = publishTopic;
  doc["device"]["name"] = mqttName;
  doc["device"]["model"] = mqttModel;
  doc["device"]["manufacturer"] = "FX";

  size_t n = serializeJson(doc, buffer);
  client.publish(discoveryTopic.c_str(), buffer, n);
}

void sendMQTTVoltageDiscoveryMsg() {
  String discoveryTopic = String("homeassistant/sensor/")+publishTopic+String("/voltage/config");

  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Voltage";
  doc["stat_t"]   = stateTopic;
  doc["state_class"]= "measurement";

  doc["unit_of_meas"] = "V";
  doc["device_class"] = "voltage";
  doc["frc_upd"] = true;
  doc["value_template"] = "{{ value_json.voltage|default(0) }}";
  doc["uniq_id"] = publishTopic+String("_voltage");

  doc["device"]["identifiers"][0] = publishTopic;
  doc["device"]["name"] = mqttName;
  doc["device"]["model"] = mqttModel;
  doc["device"]["manufacturer"] = "FX";

  size_t n = serializeJson(doc, buffer);
  client.publish(discoveryTopic.c_str(), buffer, n);
}

void sendMQTTPressureDiscoveryMsg() {
  String discoveryTopic = String("homeassistant/sensor/")+publishTopic+String("/pressure/config");

  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Pressure";
  doc["stat_t"]   = stateTopic;
  doc["state_class"]= "measurement";
  doc["device_class"] = "pressure";
  doc["unit_of_meas"] = "hPa";
  doc["frc_upd"] = true;
  doc["value_template"] = "{{ value_json.pressure|default(0) }}";
  doc["uniq_id"] = publishTopic+String("_pressure");

  doc["device"]["identifiers"][0] = publishTopic;
  doc["device"]["name"] = mqttName;
  doc["device"]["model"] = mqttModel;
  doc["device"]["manufacturer"] = "FX";

  size_t n = serializeJson(doc, buffer);
  client.publish(discoveryTopic.c_str(), buffer, n);
}


void send_autodiscovery() {
  sendMQTTHumidityDiscoveryMsg();
  sendMQTTPressureDiscoveryMsg();
  sendMQTTTemperatureDiscoveryMsg();
  sendMQTTVoltageDiscoveryMsg();
}

void readAvgSensorValues() {
  bat_voltage += 0.00457* analogRead(batVoltagePin);
  humidity += bme.readHumidity();
  temperature += bme.readTemperature();
  pressure += bme.readPressure()/100.0f;

  avg_cycles++;
}

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  uint32_t Freq = 0;

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  Wire.begin();
  Wire.setClock(100000);
  uint16_t stat = bme.begin(BME280_ADDRESS);
  Serial.print(stat, HEX);
  Serial.println();

  String resetReason = ESP.getResetReason();
  Serial.println(resetReason);

  bool wifi_success = setup_wifi();

  if (!wifi_success) {
    digitalWrite(LED, LOW);
    Serial.println("WiFi connection failed - going to sleep...");  
    Serial.flush(); 
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    ESP.deepSleep(SLEEP_FOR_MINUTES*60*1e6);

  }
// setup the mqtt server and callback
  client.setServer(mqttBroker, mqttPort);
  client.setBufferSize(512);
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //if (resetReason != "Deep-Sleep Wake") 
  {
    Serial.println("First boot: sending autodiscovery...");
    send_autodiscovery();
  }
  


}


void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  digitalWrite(LED, LOW);

  char msgString[20];


  DynamicJsonDocument doc(1024);
  char buffer[256];
  readAvgSensorValues();
  humidity /= avg_cycles;
  temperature /= avg_cycles;
  pressure /= avg_cycles;
  bat_voltage /= avg_cycles;

  doc["humidity"]      = String(humidity, 2);
  doc["temperature"]   = String(temperature, 2);
  doc["pressure"]      = String(pressure, 2);
  doc["voltage"]      =  String(bat_voltage, 3);

  Serial.printf("Temp: %.1f \t Hum: %.1f \t Pressure: %.1f \t Bat: %.2f\n", temperature, humidity, pressure, bat_voltage);

  size_t n = serializeJson(doc, buffer);

  Serial.println("publishing topic "+publishTopic);
  bool published = client.publish(stateTopic.c_str(), buffer, n);


//  esp_sleep_enable_timer_wakeup(600*1000000); 
  Serial.println("Going to sleep now");
  delay(50);
  Serial.flush(); 
  ESP.deepSleep(SLEEP_FOR_MINUTES*60*1e6);
//  esp_deep_sleep_start();
}