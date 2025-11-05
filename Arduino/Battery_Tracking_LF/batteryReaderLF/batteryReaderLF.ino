#include <Arduino.h>
#include <HardwareSerial.h>
#include "driver/twai.h"   // ESP32 CAN driver

// =============================
// --- Pin configuration ---
// =============================
#define RFID_RX_PIN 26
#define RFID_TX_PIN 34  // unused for read-only

#define LED_G 13
#define LED_B 14
#define LED_R 15

#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

// =============================
// --- Globals ---
// =============================
HardwareSerial rfidSerial(1);
SemaphoreHandle_t lfMutex;

String lfTagSerial = "";
bool lfReaderActive = false;
uint8_t lfTagCounter = 0; // 0–255 increments when new tag detected

bool CANGood = false;            // Global: true = CAN activity OK
unsigned long lastCANRxTime = 0; // Timestamp of last received frame

// =============================
// --- Function declarations ---
// =============================
void TaskHandleLFReader(void* pvParameters);
void TaskLEDIndicator(void* pvParameters);
void TaskCANTx(void* pvParameters);
void TaskCANRx(void* pvParameters);
void CAN_SendFrame(uint16_t apiId, uint8_t* data, uint8_t len);


// === CAN Constants ===
#define DEVICE_ID        0x0A  // DO NOT CHANGE
#define MANUFACTURER_ID  0x08  // DO NOT CHANGE
#define DEVICE_NUMBER    55    // 0-63 unique per device

#define STATUS_API_ID        0x180
#define COLOR_SENSOR_API_ID  0x184
#define CONTROL_API_ID       0x190
#define HEARTBEAT_ID         0x01011840

// Custom API IDs for LF Reader (use your previous 0x10–0x12)
#define API_REQUEST_REBOOT   0x110
#define API_SN_FIRST8        0x111
#define API_SN_LAST6_STATUS  0x112

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}



// =============================
// --- Setup ---
// =============================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== ESP32 LF Reader + CAN Interface ==="));

  // --- Setup CAN (TWAI) ---
  twai_general_config_t g_config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = CAN_TX_PIN,
    .rx_io = CAN_RX_PIN,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 16,
    .alerts_enabled = TWAI_ALERT_RX_DATA,
    .clkout_divider = 0
  };
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK)
    Serial.println(F("[CAN] TWAI driver started (1Mbps)"));
  else
    Serial.println(F("[CAN] ERROR starting driver"));

  lfMutex = xSemaphoreCreateMutex();

  // --- Create tasks ---
  xTaskCreatePinnedToCore(TaskHandleLFReader, "LFReader", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskLEDIndicator,   "LEDIndicator", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANTx,          "CANTx", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx,          "CANRx", 4096, NULL, 1, NULL, 0);
}

// =============================
// --- LOOP (unused) ---
// =============================
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

// =============================
// --- Task: Handle LF Reader ---
// =============================
void TaskHandleLFReader(void* pvParameters) {
  rfidSerial.begin(9600, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  Serial.println(F("[LFReader] Started. Waiting for tag..."));

  static String buffer = "";
  static String lastTag = "";
  unsigned long lastReadTime = 0;
  bool tagPresent = false;

  for (;;) {
    while (rfidSerial.available()) {
      char c = rfidSerial.read();
      if ((c >= 32 && c <= 126) || c == 0x02 || c == 0x03) {
        buffer += c;

        int startIdx = buffer.indexOf(0x02);
        int endIdx   = buffer.indexOf(0x03, startIdx + 1);

        if (startIdx >= 0 && endIdx > startIdx) {
          String frame = buffer.substring(startIdx + 1, endIdx);
          buffer = "";

          // Normalize to 14-char tag serial
          if (frame.length() > 14) frame = frame.substring(frame.length() - 14);
          while (frame.length() < 14) frame = "0" + frame;

          if (frame != lastTag) {
            lastTag = frame;
            tagPresent = true;

            lfTagCounter = (lfTagCounter + 1) & 0xFF; // increment with wrap
            Serial.printf("[LFReader] New tag: %s (count=%d)\n", frame.c_str(), lfTagCounter);
          }
          lastReadTime = millis();
        }

        if (buffer.length() > 32)
          buffer.remove(0, buffer.length() - 16);
      }
    }

    if (tagPresent && millis() - lastReadTime > 1000) {
      Serial.printf("[LFReader] Tag removed: %s\n", lastTag.c_str());
      tagPresent = false;
      lastTag = "";
    }

    // Update globals
    if (lfMutex && xSemaphoreTake(lfMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lfReaderActive = tagPresent;
      lfTagSerial = tagPresent ? lastTag : "";
      xSemaphoreGive(lfMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =============================
// --- Task: LED Indicator ---
// =============================
void TaskLEDIndicator(void* pvParameters) {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  for (;;) {
    bool active = false;
    if (lfMutex && xSemaphoreTake(lfMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      active = lfReaderActive;
      xSemaphoreGive(lfMutex);
    }

    // In LED loop:
    if (!CANGood) {
      // Blink blue fast if CAN timeout
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_B, (millis() / 200) % 2);
    }
    else if (active) {
      // Green = tag present
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_B, LOW);
    } else {
      // Yellow = no tag (R+G)
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_B, LOW);
    }


    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =============================
// --- Task: CAN Transmit (FRC Spec, Binary Null SN) ---
// =============================
void TaskCANTx(void* pvParameters) {
  String lastSentSerial = "";   // Remember last tag for counter logic
  uint8_t tagCounter = 0;
  bool lastStatus = false;

  for (;;) {
    String serial;
    bool active;

    // --- Copy shared data safely ---
    if (lfMutex && xSemaphoreTake(lfMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      serial = lfTagSerial;
      active = lfReaderActive;
      xSemaphoreGive(lfMutex);
    }

    // --- Detect tag change ---
    if (!serial.isEmpty() && serial != lastSentSerial) {
      tagCounter = (tagCounter + 1) & 0xFF;  // increment on new tag
      Serial.printf("[CANTx] New tag: %s (count=%d)\n", serial.c_str(), tagCounter);
      lastSentSerial = serial;
    }

    // --- If tag removed, persist last serial but mark inactive ---
    if (!active && serial.isEmpty()) {
      serial = lastSentSerial;  // keep last known SN
    }

    // --- Prepare frame data ---
    uint8_t data11[8] = {0};
    uint8_t data12[8] = {0};

    if (!serial.isEmpty()) {
      // Convert serial to ASCII bytes (normal mode)
      for (int i = 0; i < 8; i++) {
        data11[i] = (i < serial.length()) ? serial.charAt(i) : 0x00;
      }
      for (int i = 0; i < 6; i++) {
        data12[i] = (i + 8 < serial.length()) ? serial.charAt(i + 8) : 0x00;
      }
    } else {
      // No tag ever detected — all zeros
      memset(data11, 0x00, sizeof(data11));
      memset(data12, 0x00, sizeof(data12));
    }

    // --- Append status + counter ---
    data12[6] = active ? 1 : 0;  // 1 = tag present, 0 = removed
    data12[7] = tagCounter;

    // --- Transmit frames ---
    CAN_SendFrame(makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_SN_FIRST8, DEVICE_NUMBER), data11, 8);
    CAN_SendFrame(makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_SN_LAST6_STATUS, DEVICE_NUMBER), data12, 8);

    // --- Debug state transitions ---
    if (active != lastStatus) {
      Serial.printf("[CANTx] Tag %s, Status=%s, Count=%d\n",
                    serial.c_str(),
                    active ? "PRESENT" : "REMOVED",
                    tagCounter);
      lastStatus = active;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}



// =============================
// --- Task: CAN Receive ---
// =============================
void TaskCANRx(void* pvParameters) {
  twai_message_t msg;
  lastCANRxTime = millis();

  for (;;) {
    bool gotMsg = false;

    if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK && msg.extd) {
      gotMsg = true;
      lastCANRxTime = millis();

      uint32_t expectedRebootID = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_REQUEST_REBOOT, DEVICE_NUMBER);
      if (msg.identifier == expectedRebootID && msg.data_length_code >= 1) {
        Serial.println(F("[CAN] FRC Reboot Request received → Restarting ESP32..."));
        delay(100);
        ESP.restart();
      }
    }

    // --- Update CAN health ---
    bool good = (millis() - lastCANRxTime <= 500);
    if (lfMutex && xSemaphoreTake(lfMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      CANGood = good;
      xSemaphoreGive(lfMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =============================
// --- Helper: CAN Send Frame ---
// =============================
void CAN_SendFrame(uint32_t id, uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 1;
  msg.data_length_code = len;
  memcpy(msg.data, data, len);

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (result != ESP_OK) {
    Serial.printf("[CAN] TX failed for 0x%08lX (err=%d)\n", id, result);
  }
}
