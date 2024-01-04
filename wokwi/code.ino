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
// RTC_DS1307 rtc;
HX711 scale;
// DateTime startTime;
// TimeSpan elapsedTime;
// TimeSpan totalSleepTime;

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
unsigned long start_time = 0;

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

void loop() {
  if (getScale()){
    if (start_time == 0) {
      start_time = millis();
    }
    Serial.println(start_time);
    if(!espClient.connected()) {
      mqttReconnect();
    }
    Client.loop();

  
    tempAndHumid();
    if (digitalRead(PIR_PIN) == HIGH){
      // countTime();
      unsigned long current_time = millis();
      long sleep_time = current_time - start_time - previous_time;
      detectMotion();
      previous_time = current_time + 10500;
      total_sleep += sleep_time;
    }
    Serial.print("Total sleep: ");
    Serial.print(total_sleep);
    Serial.println(" milliseconds");
    int returncode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    int sleep_thingspeak = 0;
    if (total_sleep > 0){
      sleep_thingspeak = 1;
    } else{
      sleep_thingspeak = 0;
    }
    ThingSpeak.setField(3, sleep_thingspeak);

    if (returncode == 200) {
      Serial.println("Channel update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code");
    }
    // Wait a bit before scanning again
    delay(500);
  }
  else {
    start_time = 0;
    if(total_sleep > 0) {
      Serial.println(total_sleep);
      int returncode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      int sleep_thingspeak = 0;
      if (total_sleep > 0){
        sleep_thingspeak = 1;
      } else{
        sleep_thingspeak = 0;
      }
      ThingSpeak.setField(3, sleep_thingspeak);
      total_sleep = 0;
    }
    previous_time = 0;
  }

}


double getWeight() {
  if (scale.is_ready()) return scale.read() * 1000.0 / 420.0;
  else return -1;
}

bool getScale() {
  double weight = getWeight();
  if (weight > 1000) return true;
  else return false;
}

void tempAndHumid(){
  int temperature = dht.readTemperature();
  int humidity = dht.readHumidity();
  if(temperature > 37 || humidity > 60) {
    soundAlarm();
  }
  char buffer1[50];
  sprintf(buffer1, "{\"temperature\": %d}", temperature);
  Client.publish("Home/Temperature", buffer1);
  char buffer2[50];
  sprintf(buffer2, "{\"humidity\": %d}", humidity);
  Client.publish("Home/Humidity", buffer2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: "); 
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");
  delay(500);

  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
}



void detectMotion() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Motion detected!");
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);
  controlServos();
  playMelody();
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
}

void soundAlarm() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(5000); // Sound alarm for 5 seconds
  digitalWrite(BUZZER_PIN, LOW);  
}

void playMelody() {

  // Define the melody notes and durations
  // int melody[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4};
  int melody[] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_B4};

  // Iterate through the melody
  for (int i = 0; i < 4; i++) {
    int noteFrequency = melody[i];

    // Calculate the number of cycles for the PWM signal
    int cycles = 500;

    // Play the note manually with PWM
    for (int j = 0; j < cycles; j++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(noteFrequency);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(noteFrequency);
    }

    delay(50);  // add a small delay between notes
  }

  digitalWrite(BUZZER_PIN, LOW);  // turn off the buzzer
}


void controlServos() {
  // Motion detected, run the servos
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  servo4.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);  
  delay(1000);
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  servo4.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  delay(1000);
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  servo4.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  delay(1000);
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  servo4.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  delay(1000);
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  servo4.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  delay(1000);
}

