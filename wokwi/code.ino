#include <WiFi.h>
// #include <Wire.h>
#include "LiquidCrystal.h"
#include "DHT.h"
#include "ESP32Servo.h"
#include "PubSubClient.h"
#include "ThingSpeak.h"
// #include "RTClib.h"
#include "HX711.h"

#define LCD_I2C_ADDR 0x27
#define DHT_PIN 4
#define PIR_PIN 2
#define BUZZER_PIN 16
#define LED2_PIN 25
#define LED3_PIN 26
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 27
#define DOUT_PIN 15  
#define SCK_PIN 17  

LiquidCrystal lcd(23, 22, 21, 19, 18, 5);
DHT dht(DHT_PIN, DHT22);
Servo servo1, servo2, servo3, servo4;
HX711 scale;


#define NOTE_C4 261.63
#define NOTE_D4 293.66
#define NOTE_E4 329.63
#define NOTE_F4 349.23
#define NOTE_G4 392.00
#define NOTE_A4 440.00
#define NOTE_B4 493.88

const char* ssid = "Wokwi-GUEST";
const char* pwd = "";
const char* mqttServer = "test.mosquitto.org";
const char* host = "";
const char* req = "";
int port = 1883;
unsigned long myChannelNumber = 2373888;
const char* myWriteAPIKey = "5U7X98T6N86XGGA2";
const char* myReadAPIKey = "PPXQIRCSVIGGUN8B";
WiFiClient espClient;
PubSubClient Client(espClient);
unsigned long previous_time = 0;
unsigned long total_sleep = 0;
unsigned long start_time = -1;
int cnt = 0;
int tmp = 0;
void wifiConnect(){
  WiFi.begin(ssid, pwd);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }

  Serial.print("Connected!");
}

void mqttReconnect(){
  while(!Client.connected()) {
    Serial.println("Attemping MQTT connection...");
    if(Client.connect("21127211")){
      Serial.println("Connected");
      Client.subscribe("topicName/led");
    }
    else {
      Serial.println("Try again in 2 secs");
      delay(2000);
    }
  }
}

void sendHttpRequest() {
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  WiFiClient client;
  while (!client.connect(host, port)){
    Serial.println("connection fail");
    delay(500);
  }

  client.print(String("GET /") + String(req) + "HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection : close\r\n\r\n");
  delay(500);

  while (client.available()) {
    String line = client.readStringUntil('\R');
    Serial.print(line);
  }
  Serial.println();
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print(topic);
  String stMessage;
  for (int i = 0; i<length; i++) {
    stMessage += (char)message[i];
  }
  Serial.println(": ");
  Serial.println(stMessage);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Connecting to Wifi...");
  
  wifiConnect();

  Client.setServer(mqttServer, port);
  Client.setCallback(callback);
  
  lcd.begin(16, 2);
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  Serial.println("Initializing the scale");
  scale.begin(DOUT_PIN, SCK_PIN);

  ThingSpeak.begin(espClient);
}