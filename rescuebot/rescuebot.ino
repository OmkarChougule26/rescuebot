#include <WiFi.h>
#include <FirebaseESP32.h>
#include "DHT.h"
#include <TinyGPSPlus.h>
#include "driver/i2s.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= CREDENTIALS =================
#define WIFI_SSID "OmkarPhone"
#define WIFI_PASSWORD "98765432"
#define FIREBASE_HOST "rescuebot-9f045-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "YOUR_DATABASE_SECRET_KEY"

// ================= PINS =================
#define RXD2 16
#define TXD2 17
#define BT_RX 18
#define BT_TX 19
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26
#define FLAME_PIN 34
#define MQ2_PIN 36
#define MQ135_PIN 39
#define DHTPIN 4
#define TRIG_PIN 27
#define ECHO_PIN 13

// AUDIO PINS
#define I2S_WS 21
#define I2S_SCK 22
#define I2S_SD 23
#define BUTTON_PIN 5

#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;
WiFiServer audioServer(88);

unsigned long lastFirebaseTime = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable Brownout
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // GPS
  Serial1.begin(9600, SERIAL_8N1, BT_RX, BT_TX);   // Bluetooth

  pinMode(FLAME_PIN, INPUT); pinMode(MQ2_PIN, INPUT); pinMode(MQ135_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  stopBot();
  dht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println("WiFi Connected");
  Serial.print("AUDIO IP: "); Serial.println(WiFi.localIP());

  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // --- OPTIMIZED AUDIO SETUP (LOW RAM USAGE) ---
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 8000, // REDUCED to 8kHz (Saves RAM)
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4, // REDUCED from 8 (Saves RAM)
    .dma_buf_len = 64,  // Keep small
    .use_apll = false
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  
  audioServer.begin();
}

void loop() {
  // 1. Bluetooth
  if (Serial1.available()) {
    char cmd = Serial1.read();
    if (cmd == 'F') moveForward(); else if (cmd == 'B') moveBackward();
    else if (cmd == 'L') turnLeft(); else if (cmd == 'R') turnRight();
    else if (cmd == 'S') stopBot();
  }

  // 2. Audio Stream
  WiFiClient client = audioServer.available();
  if (client) {
    client.print("HTTP/1.1 200 OK\r\nContent-Type: audio/wav\r\n\r\n");
    char i2s_buff[512]; // Smaller buffer
    size_t bytes_read;

    while (client.connected()) {
      if (digitalRead(BUTTON_PIN) == LOW) {
        i2s_read(I2S_NUM_0, (void*)i2s_buff, sizeof(i2s_buff), &bytes_read, portMAX_DELAY);
        client.write((const uint8_t*)i2s_buff, bytes_read);
      } else {
        // Send tiny keep-alive data to prevent disconnection
        client.write((uint8_t)0); // <--- FIXED LINE HERE
        delay(10);
      }
      // Keep checking motors/sensors while talking
      if (Serial1.available()) { char cmd = Serial1.read(); if(cmd=='S') stopBot(); }
      
      if (millis() - lastFirebaseTime > 3000) { // Update sensors slower (3s) to save CPU
        lastFirebaseTime = millis();
        uploadData();
      }
    }
    client.stop();
  }

  // 3. Sensors
  if (millis() - lastFirebaseTime > 2000) {
    lastFirebaseTime = millis();
    uploadData();
  }
  while (Serial2.available() > 0) gps.encode(Serial2.read());
}

void uploadData() {
  // Read Sensors
  int mq2 = analogRead(MQ2_PIN);
  int mq135 = analogRead(MQ135_PIN);
  int flame = digitalRead(FLAME_PIN) == LOW ? 1 : 0;
  float t = dht.readTemperature();
  
  // Ultrasonic
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float dist = duration * 0.034 / 2;

  if (isnan(t)) t=0; 

  FirebaseJson json;
  json.set("mq2", mq2); 
  json.set("mq135", mq135);
  json.set("flame", flame); 
  json.set("temp", t);
  json.set("distance", dist);
  // Skipped Humidity/GPS log to save memory for Audio
  
  if (gps.location.isValid()) {
    json.set("lat", gps.location.lat());
    json.set("lng", gps.location.lng());
    json.set("gps_status", "LOCKED");
  } else {
    json.set("lat", 0); json.set("lng", 0);
    json.set("gps_status", "SEARCHING");
  }
  
  Firebase.setJSON(firebaseData, "/sensors", json);
}

void moveForward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void moveBackward() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnLeft() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void turnRight() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void stopBot() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }