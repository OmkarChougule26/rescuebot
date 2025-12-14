#include <WiFi.h>
#include "driver/i2s.h"

// ================= CREDENTIALS =================
#define WIFI_SSID "OmkarPhone"
#define WIFI_PASSWORD "98765432"

// ================= PINS =================
#define I2S_SCK 14      // Shared Clock
#define I2S_WS 15       // Shared Word Select
#define I2S_SD_MIC 13   // Mic Data IN
#define I2S_SD_SPK 27   // Speaker Data OUT
#define SWITCH_PIN 4    // Control Switch

#define SAMPLE_RATE 16000
#define BUFFER_LEN 512

WiFiServer server(88);
WiFiClient client;
bool connected = false;

// Task Handles
TaskHandle_t TaskMic;
TaskHandle_t TaskSpk;

void setup() {
  Serial.begin(115200);
  
  // SETUP SWITCH (High when Open, Low when Closed/GND)
  pinMode(SWITCH_PIN, INPUT_PULLUP); 

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.print("AUDIO IP: "); Serial.println(WiFi.localIP());

  // I2S Setup (Full Duplex capable)
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_SD_SPK,
    .data_in_num = I2S_SD_MIC
  };
  
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  
  server.begin();

  // Start Tasks
  xTaskCreatePinnedToCore(micTask, "MicTask", 10000, NULL, 1, &TaskMic, 0);
  xTaskCreatePinnedToCore(spkTask, "SpkTask", 10000, NULL, 1, &TaskSpk, 1);
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      client.setNoDelay(true);
      connected = true;
    } else {
      connected = false;
    }
  }
  delay(100);
}

// === MIC TASK (Controlled by Switch) ===
void micTask(void * parameter) {
  uint8_t buff[BUFFER_LEN];
  size_t bytes_read;
  
  while(true) {
    if (connected && client.connected()) {
      
      // CHECK SWITCH LOGIC HERE
      // LOW means connected to GND (Switch ON)
      if (digitalRead(SWITCH_PIN) == LOW) {
         // Switch is ON -> Read Mic & Send
         i2s_read(I2S_NUM_0, (void*)buff, sizeof(buff), &bytes_read, portMAX_DELAY);
         client.write(buff, bytes_read);
      } else {
         // Switch is OFF -> Do NOT send audio
         // We send a tiny "heartbeat" 0 to keep connection alive, or just wait
         delay(10); 
      }
      
    } else {
      delay(10);
    }
  }
}

// === SPEAKER TASK (Always Active) ===
void spkTask(void * parameter) {
  uint8_t buff[BUFFER_LEN];
  size_t bytes_written;
  
  while(true) {
    if (connected && client.connected() && client.available()) {
      int len = client.read(buff, sizeof(buff));
      if (len > 0) {
         // Play whatever comes from Laptop (YouTube/Voice)
         // Shift for volume boost
         for (int i=0; i<len; i+=2) {
            int16_t* sample = (int16_t*)&buff[i];
            *sample = *sample << 1; 
         }
         i2s_write(I2S_NUM_0, (const char*)buff, len, &bytes_written, portMAX_DELAY);
      }
    } else {
      delay(5);
    }
  }
}