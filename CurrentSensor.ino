#include "arduino_secrets.h"

#include <WiFi.h>
#include <PubSubClient.h>

// WIFI CONFIG 
const char* ssid     = "Gooners";
const char* password = "Pabg830!";

// MQTT CONFIG 
const char* mqtt_server = "192.168.0.90";
const int   mqtt_port   = 1883;

// MQTT TOPICS 
const char* topicCurrent = "factory/nodeB/current";
const char* topicCmd     = "factory/nodeB/cmd";
const char* topicAlarm   = "factory/nodeB/alarm";

// PIN DEFINITIONS 
const int ACS_PIN    = 34;
const int BUZZER_PIN = 23;

// ADC PARAMETERS
const float ADC_REF = 3.3;
const int   ADC_MAX = 4095;

// ACS712 PARAMETERS 
float zeroOffset = 0.0;
const float CURRENT_THRESHOLD = 3.0;
const float SENSITIVITY = 0.066;

//  PASSIVE BUZZER 
const uint32_t BUZZER_FREQ = 2000;
const uint8_t  BUZZER_RES  = 8;

// OBJECTS 
WiFiClient espClient;
PubSubClient client(espClient);

// WIFI 
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

// MQTT 
void connectMQTT() {
  while (!client.connected()) {
    String clientId = "NodeB_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Connecting to MQTT as ");
    Serial.println(clientId);

    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
      client.subscribe(topicCmd);
      Serial.print("Subscribed to ");
      Serial.println(topicCmd);
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(client.state());
      delay(3000);
    }
  }
}

// MQTT CALLBACK 
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("\nMQTT message received");

  Serial.print("Topic: ");
  Serial.println(topic);

  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Payload: ");
  Serial.println(msg);

  if (String(topic) == topicCmd) {
    if (msg == "ON") {
      ledcWrite(BUZZER_PIN, 128);
      Serial.println("Buzzer ON (remote)");
    }
    else if (msg == "OFF") {
      ledcWrite(BUZZER_PIN, 0);
      Serial.println("Buzzer OFF (remote)");
    }
  }
}

// ACS712 CALIBRATION
void calibrateACS712() {
  Serial.println("Calibrating ACS712 (NO LOAD)...");
  delay(2000);

  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(ACS_PIN);
    delay(10);
  }

  zeroOffset = (sum / 100.0) * ADC_REF / ADC_MAX;
  Serial.print("Zero offset voltage: ");
  Serial.println(zeroOffset, 3);
}

void setup() {
  Serial.begin(115200);

  analogSetAttenuation(ADC_11db);

  // Passive buzzer PWM (ESP32 core v3)
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES);
  ledcWrite(BUZZER_PIN, 0);

  connectWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  connectMQTT();   // ð´ force connection print at startup
  calibrateACS712();
}

void loop() {
  if (!client.connected()) {
    Serial.println("MQTT disconnected, reconnecting...");
    connectMQTT();
  }
  client.loop();

  static unsigned long lastPublish = 0;
  static bool alarmState = false;

  if (millis() - lastPublish >= 2000) {
    lastPublish = millis();

    int adc = analogRead(ACS_PIN);
    float voltage = (adc * ADC_REF) / ADC_MAX;
    float current = abs((voltage - zeroOffset) / SENSITIVITY);

    char msg[16];
    snprintf(msg, sizeof(msg), "%.3f", current);
    client.publish(topicCurrent, msg);

    if (current > CURRENT_THRESHOLD && !alarmState) {
      ledcWrite(BUZZER_PIN, 128);
      client.publish(topicAlarm, "ON", true);
      Serial.println("ALARM TRIGGERED");
      alarmState = true;
    }
    else if (current <= CURRENT_THRESHOLD && alarmState) {
      ledcWrite(BUZZER_PIN, 0);
      client.publish(topicAlarm, "OFF", true);
      Serial.println("ALARM CLEARED");
      alarmState = false;
    }

    Serial.print("Current: ");
    Serial.print(current, 3);
    Serial.println(" A");
  }
}
