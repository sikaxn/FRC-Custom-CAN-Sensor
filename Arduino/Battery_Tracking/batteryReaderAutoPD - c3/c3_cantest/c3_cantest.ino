#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN  GPIO_NUM_8
#define CAN_RX_PIN  GPIO_NUM_9

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting TWAI (CAN) receiver at 1 Mbps...");

  // General TWAI config
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

  // 1 Mbps timing config
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

  // Accept all incoming messages
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and start TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    while (true);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    while (true);
  }

  Serial.println("TWAI driver started. Listening for CAN frames...");
}

void loop() {
  twai_message_t message;

  // Blocking wait for message (timeout 1000ms)
  if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    // Message ID
    if (message.extd) {
      Serial.printf("Extended ID: 0x%08X\n", message.identifier);
    } else {
      Serial.printf("Standard ID: 0x%03X\n", message.identifier);
    }

    // Message type
    if (message.rtr) {
      Serial.println("Remote Transmission Request (RTR) frame");
    } else {
      // Data length and data bytes
      Serial.printf("DLC: %d | Data: ", message.data_length_code);
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.printf("%02X ", message.data[i]);
      }
      Serial.println();
    }
  }
}
