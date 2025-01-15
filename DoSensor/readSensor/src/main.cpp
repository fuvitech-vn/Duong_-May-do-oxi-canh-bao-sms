#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

#define simSerial               Serial1
#define MCU_SIM_BAUDRATE        115200
#define MCU_SIM_TX_PIN              10
#define MCU_SIM_RX_PIN              9
#define MCU_SIM_EN_PIN              15

#define PHONE_NUMBER                ""
#define RXD2 16
#define TXD2 17

const char* ssid = "";                
const char* password = "";        

const char* mqttServer = "";  
const int mqttPort = ;                  
const char* mqttUser = "";                  
const char* mqttPassword = "";              
const char* mqttTopic = "";
const char* mqttTopic2 = "";    

const char* mqttSensorLimitTopic = "";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

byte ByteArray[250];
const char msg_reguest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x2A, 0xC4, 0x15};
float oxi_ = 0.0;
float sensorLimit = 0.0;
unsigned long lastSmsTime = 0;
unsigned long smsInterval = 60000;  
unsigned long lastMqttTime = 0; 
unsigned long mqttInterval = 60000;  
void sim_at_wait()
{
    delay(100);
    while (simSerial.available()) {
        Serial.write(simSerial.read());
    }
}
bool sim_at_cmd(String cmd){
    simSerial.println(cmd);
    sim_at_wait();
    return true;
}
bool sim_at_send(char c){
    simSerial.write(c);
    return true;
}
void send_sms(String message) {
    sim_at_cmd("AT+CMGF=1");
    String temp = "AT+CMGS=\"";
    temp += (String)PHONE_NUMBER;
    temp += "\"";
    sim_at_cmd(temp);
    sim_at_cmd(message);
    sim_at_send(0x1A);
    Serial.println("SMS sent: " + message);
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);
  if (String(topic) == mqttSensorLimitTopic) {
    sensorLimit = message.toFloat();
    Serial.print("Updated sensorLimit: ");
    Serial.println(sensorLimit);
  }
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); 
  simSerial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("MQTT connected");
      client.subscribe(mqttSensorLimitTopic);
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
  sim_at_cmd("AT");
  sim_at_cmd("ATI");
  sim_at_cmd("AT+CPIN?");
  sim_at_cmd("AT+CSQ");
  sim_at_cmd("AT+CIMI");
}

void loop() {
  int len = 8;
  for (int i = 0; i < len; i++) {
    Serial2.write(msg_reguest[i]);
    Serial.print("[" + String(i) + "]="); 
    Serial.print("Ox");
    Serial.print(String(msg_reguest[i], HEX));      
    Serial.print(" ");
  }
  Serial.println();

  int a = 0;
  while (Serial2.available()) {
    ByteArray[a] = Serial2.read();
    a++;
  }

  if (a > 0) {
    word oxi = ((ByteArray[43] << 8) & 0xFF00) | ByteArray[44];
    oxi_ = oxi * 0.01;
    

    Serial.print("Byte 43: ");
    Serial.println(ByteArray[43], HEX); 
    Serial.print("Byte 44: ");
    Serial.println(ByteArray[44], HEX);
    Serial.print("Muc Oxy: ");
    Serial.println(oxi_);

    StaticJsonDocument<200> doc;
    doc["oxi"] = oxi_;  
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(mqttTopic, jsonBuffer);
    if(millis() - lastMqttTime >= mqttInterval){
      client.publish(mqttTopic2, jsonBuffer);
      lastMqttTime = millis();
    }
  }

  if (!client.connected()) {
    while (!client.connected()) {
      Serial.println("Reconnecting to MQTT...");
      if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
        Serial.println("MQTT reconnected");
        client.subscribe(mqttSensorLimitTopic);
      } else {
        Serial.print("Failed to connect, rc=");
        Serial.print(client.state());
        delay(2000);
      }
    }
  }
   if (oxi_ < sensorLimit && millis() - lastSmsTime >= smsInterval) {
    String alertMessage = "Canh bao! Muc Oxy = ";
    alertMessage += String(oxi_);
    alertMessage += " thap hon nguong " + String(sensorLimit);
    send_sms(alertMessage);
    lastSmsTime = millis();
  }

  client.loop();  
  delay(1000);   
}
