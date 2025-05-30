#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "driver/twai.h"

// === MCP2515 Pins ===
#define MCP_CS_PIN 25
#define MCP_SCK    18
#define MCP_MISO   19
#define MCP_MOSI   23
#define LED_BUILTIN 2

MCP_CAN mcpCAN(MCP_CS_PIN);

// === TWAI Pins ===
#define TWAI_TX GPIO_NUM_5
#define TWAI_RX GPIO_NUM_4

// === CAN IDs ===
#define UNIVERSAL_HEARTBEAT_CAN_ID 0x01011840
#define JAT_RELAY_HEARTBEAT_ID     0x298
#define JAT_ENABLE_STATUS_ID       0x299

// === RIO CAN ID Constants ===
#define DEVICE_TYPE_ID  0x0A
#define MANUFACTURER_ID 0x08
#define DEVICE_NUMBER   11

// === Control Map ===
struct ControlMap {
  uint16_t rm_can_id;
  uint16_t rio_api_id;
};

ControlMap rio_to_rm_map[] = {
  {0x200, 0x111},
  {0x1FF, 0x112},
  {0x2FF, 0x113}
};

ControlMap rm_to_rio_map[] = {
  {0x201, 0x121}, {0x202, 0x122}, {0x203, 0x123},
  {0x204, 0x124}, {0x205, 0x125}, {0x206, 0x126},
  {0x207, 0x127}, {0x208, 0x128}, {0x209, 0x129},
  {0x20A, 0x12A}, {0x20B, 0x12B}
};

uint8_t rio_data[3][8] = {0};
uint32_t rio_last_received[3] = {0};
bool heartbeat_enabled = false;
uint32_t last_heartbeat = 0;
const uint32_t DATA_TIMEOUT_MS = 1000;

// === CAN ID Encoder ===
uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

// === Tasks ===

// MCP → TWAI (RIO → RM)
void Task_MCP_Reader(void *pv) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    long unsigned id;
    uint8_t len, buf[8];
    if (mcpCAN.readMsgBuf(&id, &len, buf) == CAN_OK) {
      id &= 0x1FFFFFFF;

      // Heartbeat handling
      if (id == UNIVERSAL_HEARTBEAT_CAN_ID && len == 8) {
        last_heartbeat = millis();
        heartbeat_enabled = (buf[4] >> 4) & 0x01;
        digitalWrite(LED_BUILTIN, heartbeat_enabled);

        // Relay heartbeat
        twai_message_t relay_msg = {};
        relay_msg.identifier = JAT_RELAY_HEARTBEAT_ID;
        relay_msg.data_length_code = 8;
        memcpy(relay_msg.data, buf, 8);
        twai_transmit(&relay_msg, pdMS_TO_TICKS(10));

        // Enabled status
        twai_message_t en_msg = {};
        en_msg.identifier = JAT_ENABLE_STATUS_ID;
        en_msg.data_length_code = 1;
        en_msg.data[0] = heartbeat_enabled ? 1 : 0;
        twai_transmit(&en_msg, pdMS_TO_TICKS(10));
      }

      // RIO → RM
// RIO → RM
for (int i = 0; i < 3; i++) {
  uint32_t expected_id = makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, rio_to_rm_map[i].rio_api_id, DEVICE_NUMBER);
  if (id == expected_id && len <= 8) {
    memcpy(rio_data[i], buf, len);
    rio_last_received[i] = millis();

    twai_message_t rm_msg = {};
    rm_msg.identifier = rio_to_rm_map[i].rm_can_id;
    rm_msg.data_length_code = 8;

    if (heartbeat_enabled) {
      memcpy(rm_msg.data, buf, len);
      if (len < 8) memset(rm_msg.data + len, 0, 8 - len);
    } else {
      memset(rm_msg.data, 0, 8);
    }

    twai_transmit(&rm_msg, pdMS_TO_TICKS(10));
  }
}

    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// TWAI → MCP (RM → RIO)
void Task_TWAI_Reader(void *pv) {
  while (1) {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      for (int i = 0; i < 11; i++) {
        if (rm_to_rio_map[i].rm_can_id == rx_msg.identifier) {
          uint32_t rio_id = makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, rm_to_rio_map[i].rio_api_id, DEVICE_NUMBER);
          mcpCAN.sendMsgBuf(rio_id, 1, rx_msg.data_length_code, rx_msg.data);
          break;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Timeout Handler (sends zeros if stale)
void Task_TimeoutHandler(void *pv) {
  while (1) {
    uint32_t now = millis();
    for (int i = 0; i < 3; i++) {
      if (!heartbeat_enabled || (now - rio_last_received[i] > DATA_TIMEOUT_MS)) {
        twai_message_t timeout_msg = {};
        timeout_msg.identifier = rio_to_rm_map[i].rm_can_id;
        timeout_msg.data_length_code = 8;
        memset(timeout_msg.data, 0, 8);
        twai_transmit(&timeout_msg, pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// === Hardware Setup ===
void setupTWAI() {
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NO_ACK);
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
  twai_start();
}

void setupMCP2515() {
  while (mcpCAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init failed. Retrying...");
    delay(200);
  }
  mcpCAN.setMode(MCP_NORMAL);
  Serial.println("MCP2515 ready.");
}


// === Setup ===
void setup() {
  Serial.begin(115200);
  SPI.begin(MCP_SCK, MCP_MISO, MCP_MOSI, MCP_CS_PIN);

  setupTWAI();
  setupMCP2515();

  Serial.println("=== Calculated CAN IDs ===");
for (int i = 0; i < 3; i++) {
  Serial.print("RIO->RM: API 0x");
  Serial.print(rio_to_rm_map[i].rio_api_id, HEX);
  Serial.print(" → CAN ID 0x");
  Serial.println(makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, rio_to_rm_map[i].rio_api_id, DEVICE_NUMBER), HEX);
}

for (int i = 0; i < 11; i++) {
  Serial.print("RM->RIO: RM ID 0x");
  Serial.print(rm_to_rio_map[i].rm_can_id, HEX);
  Serial.print(" → CAN ID 0x");
  Serial.println(makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, rm_to_rio_map[i].rio_api_id, DEVICE_NUMBER), HEX);
}


  xTaskCreatePinnedToCore(Task_MCP_Reader, "MCP_Reader", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_TWAI_Reader, "TWAI_Reader", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_TimeoutHandler, "TimeoutHandler", 2048, NULL, 1, NULL, 1);
}

// === Loop ===
void loop() {
  vTaskDelay(portMAX_DELAY);
}
