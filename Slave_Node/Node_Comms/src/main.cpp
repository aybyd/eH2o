#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <Arduino.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial nodemcu(D1, D2);
SimpleTimer secondTimer;
WiFiManager wifiManager;

unsigned long previousMillis = 0;
unsigned long interval = 10000;

uint8_t hardware_id[6];
char id_char[18];
char c;
String dataIn;

const char *mqttServer = "192.168.0.154";
const int mqttPort = 1883;

void callback(char *topic, byte *payload, unsigned int length)
{

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  Serial.println();

  String message = "";
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  Serial.println("-----------------------");
  if (String(topic) == "LED")
  {
    if (message == "LED ON")
    {
      // digitalWrite(LED, HIGH);
      Serial.println("LED IS ON");
    }
    else
    {
      // digitalWrite(LED, LOW);
    }
  }
}

WiFiClient espClient;
PubSubClient client(mqttServer, mqttPort, callback, espClient);

void setup()
{
  Serial.begin(115200);
  nodemcu.begin(9600);
  secondTimer.setInterval(10000);
  // wifiManager.resetSettings();
  WiFi.macAddress(hardware_id);
  for (unsigned int i = 0; i < sizeof(hardware_id); ++i)
  {
    sprintf(id_char, "%s%02X", id_char, hardware_id[i]);
  }
  Serial.println(id_char);
  wifiManager.autoConnect("EH2O_NODE");
  Serial.println("Connected to WiFi!");
  client.setServer(mqttServer, mqttPort);
  // client.setCallback(callback);
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop()
{
  client.loop();
  while (nodemcu.available() > 0)
  {
    c = nodemcu.read();
    if (c == '\n')
    {
      break;
    }
    else
    {
      dataIn += c;
    }
  }
  if (c == '\n')
  {
    Serial.println(dataIn);
    StaticJsonDocument<256> doc;
    doc = dataIn;
    client.publish("nodes_data", dataIn.c_str());
    c = 0;
    dataIn = "";
  }
}