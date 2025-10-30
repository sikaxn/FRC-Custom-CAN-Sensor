#include <Arduino.h>  // Needed for FreeRTOS in Arduino context

#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <ArduinoJson.h>
#include <NdefMessage.h>
#include <NdefRecord.h>

#include "driver/twai.h"

// === Pin Definitions ===
#define RST_PIN 22
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

// === CAN Constants ===
#define BATTERY_STATUS_API_ID_1  0x135
#define BATTERY_STATUS_API_ID_2  0x136
#define RFID_META_API_ID_1       0x131
#define RFID_META_API_ID_2       0x132
#define RFID_META_API_ID_3       0x133
#define HEARTBEAT_ID             0x01011840

#define DEVICE_ID        0x0A  // DO NOT CHANGE
#define MANUFACTURER_ID  0x08  // DO NOT CHANGE
#define DEVICE_NUMBER    33    // Device Number 0–63

// === CTRE PDP Constants ===
#define CTRE_PDP_TYPE_ID 0x08
#define CTRE_PDP_MANUF_ID 0x04
#define CTRE_PDP_API_VOLTAGE 0x052
#define CTRE_PDP_API_CURRENT 0x05D

// === REV PDH Constants ===
#define REV_PDH_TYPE_ID 0x08
#define REV_PDH_MANUF_ID 0x05
#define REV_PDH_API_ID 0x064

// LED Define
#define LED_R 13
#define LED_B 14
#define LED_G 15



#define DISABLE_WRITE_DELAY_MS 1000

// === CAN Message ID Constructor ===
uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

// === RFID Setup ===
MFRC522DriverPinSimple ss_pin1(32);  // Reader 1 SS
MFRC522DriverPinSimple ss_pin2(33);  // Reader 2 SS

MFRC522DriverSPI driver1{ss_pin1};
MFRC522DriverSPI driver2{ss_pin2};

MFRC522 mfrc1{driver1};
MFRC522 mfrc2{driver2};

MFRC522::MIFARE_Key keyA;

// === Task Declarations ===
void TaskCANRxJava(void* pvParameters);
void TaskCANRxrioHeartbeat(void* pvParameters);
void TaskCANRxPD(void* pvParameters);
void TaskCANTx(void* pvParameters);
void TaskAutoBatteryManager(void* pvParameters);
void TaskLEDIndicator(void* pvParameters);
void TaskEnergyCalc(void* pvParameters);

// === CAN & System State ===
volatile bool canAvailable = false;
volatile bool lastEnabled = false;

volatile int year = 0, month = 0, day = 0;

volatile uint8_t hour = 0, minute = 0, second = 0;
volatile float voltage = 0.0, lowestVoltage = 0.0;
volatile float PDvoltage = 0, PDcurrent = 0;
volatile int energy = 0;

// === Battery Metadata (from tag) ===
char batterySN[17] = "";            // Up to 16 characters + null
uint16_t batteryFirstUse = 0;       // Encoded as MMDD (e.g., 603 for June 3)
uint16_t batteryCycleCount = 0;
uint8_t batteryNote = 0;
bool batteryMetaValid = false;
int currentSessionId = 0;

volatile unsigned long lastHeartbeatMs = 0;
volatile bool currentlyEnabled = false;
volatile bool wasEnabled = false;
volatile bool heartbeatOk = false; 


  enum PDType {
    NO_PD,
    CTRE_PDP,
    REV_PDH
  };

    enum State {
    STATE_WAIT_FOR_TAG,
    STATE_PARSE_AND_WRITE_INITIAL,
    STATE_WAIT_FOR_DATA,
    STATE_WRITE_FINAL
  };

extern volatile State currentState;
extern volatile PDType pdType;
bool readerDetected = false;
bool heartbeatAvailable = false;
bool heartbeatEnabled = false;

volatile PDType pdType = NO_PD;
volatile State currentState = STATE_WAIT_FOR_TAG;


// === Forward Declarations ===
String handleReader(MFRC522& reader, int readerNum);
void printParsedBatteryJson(const String& json);
String extractJsonFromNdefText(const String& raw);
int extractInt(const String& src, const char* key);
String extractString(const String& src, const char* key);
void writeNewUsageLog(int eventId, const String& timeStr, int energy, float voltage, int readerId);
char batteryFirstUseFull[11] = "";  // Format: "yyMMddHHmm"



void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);

  Serial.begin(115200);
  while (!Serial);

  // === Reset pin for MFRC522 (shared between both readers) ===
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);  // Keep reset HIGH to enable readers

  // === Set MIFARE default key A ===
  byte keyVal[6] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};
  memcpy(keyA.keyByte, keyVal, 6);

  // === SPI Setup (SCK, MISO, MOSI) ===
  SPI.begin(18, 19, 23);  // SCK=18, MISO=19, MOSI=23

  // === Initialize RFID Readers ===
  mfrc1.PCD_Init();
  mfrc2.PCD_Init();

  // -------------------------------------------------------
  // Detect if MFRC522 readers are responding via version register
  // -------------------------------------------------------
  bool reader1OK = false;
  bool reader2OK = false;

  Serial.println("Reader 1:");
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc1, Serial);
  delay(100);

  Serial.println("Reader 2:");
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc2, Serial);
  delay(100);

  byte ver1 = mfrc1.PCD_GetVersion();
  byte ver2 = mfrc2.PCD_GetVersion();

  // Known valid MFRC522 and FM17522 family codes
  if (ver1 == 0x91 || ver1 == 0x92 || ver1 == 0x88 || ver1 == 0xB2) reader1OK = true;
  if (ver2 == 0x91 || ver2 == 0x92 || ver2 == 0x88 || ver2 == 0xB2) reader2OK = true;

  if (reader1OK || reader2OK) {
    readerDetected = true;
    Serial.printf("[INFO] Reader(s) detected: %s%s\n",
                  reader1OK ? "R1 " : "",
                  reader2OK ? "R2" : "");
  } else {
    readerDetected = false;
    Serial.println("[WARN] No reader detected (Version = unknown).");
  }

  // === CAN (TWAI) Configuration ===
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

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    canAvailable = true;
    Serial.println(F("[CAN] TWAI bus started OK."));
  } else {
    Serial.println(F("[CAN] TWAI bus failed to start."));
  }

  Serial.println(F("System ready. Present tag to one reader or send a command."));


xTaskCreatePinnedToCore(TaskAutoBatteryManager, "Battery Manager", 4096, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(TaskCANTx,              "CAN TX",          4096, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(TaskCANRxPD,            "CAN RX PD",       4096, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(TaskCANRxJava,          "CAN RX Java",     4096, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(TaskCANRxrioHeartbeat,  "CAN RX Heartbeat",4096, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(TaskEnergyCalc,         "Energy Calc",     4096, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(TaskLEDIndicator,       "LEDIndicator", 2048, NULL, 1, NULL, 1);

}




void loop() {}


void TaskAutoBatteryManager(void* pvParameters) {
  static int lockedReader = 0;
  bool wasEnabledLocal = false;  // Local copy to detect transition

  // Ensure initial state is set
  currentState = STATE_WAIT_FOR_TAG;

  for (;;) {
    switch (currentState) {
      case STATE_WAIT_FOR_TAG: {
        if (mfrc1.PICC_IsNewCardPresent() && mfrc1.PICC_ReadCardSerial()) {
          String json = handleReader(mfrc1, 1);
          if (json != "") {
            lockedReader = 1;
            batteryMetaValid = true;

            StaticJsonDocument<2048> doc;
            if (!deserializeJson(doc, json)) {
              JsonArray logs = doc["u"].as<JsonArray>();
              int maxId = 0;
              for (JsonObject log : logs) {
                int id = log["i"] | 0;
                if (id > maxId) maxId = id;
              }
              currentSessionId = maxId + 1;
              Serial.printf("[AUTO] Reader 1 locked. Session ID = %d\n", currentSessionId);
              printParsedBatteryJson(json);
              currentState = STATE_PARSE_AND_WRITE_INITIAL;
            }
          }
        } else if (mfrc2.PICC_IsNewCardPresent() && mfrc2.PICC_ReadCardSerial()) {
          String json = handleReader(mfrc2, 2);
          if (json != "") {
            lockedReader = 2;
            batteryMetaValid = true;

            StaticJsonDocument<2048> doc;
            if (!deserializeJson(doc, json)) {
              JsonArray logs = doc["u"].as<JsonArray>();
              int maxId = 0;
              for (JsonObject log : logs) {
                int id = log["i"] | 0;
                if (id > maxId) maxId = id;
              }
              currentSessionId = maxId + 1;
              Serial.printf("[AUTO] Reader 2 locked. Session ID = %d\n", currentSessionId);
              printParsedBatteryJson(json);
              currentState = STATE_PARSE_AND_WRITE_INITIAL;
            }
          }
        }
        break;
      }

      case STATE_PARSE_AND_WRITE_INITIAL: {
        writeNewUsageLog(currentSessionId, "0000000000", 0, 0.0f, lockedReader);
        Serial.println("[AUTO] Initial placeholder written.");
        wasEnabledLocal = currentlyEnabled;
        currentState = STATE_WAIT_FOR_DATA;
        break;
      }

      case STATE_WAIT_FOR_DATA: {
        if (wasEnabledLocal && !currentlyEnabled) {
          currentState = STATE_WRITE_FINAL;
        }
        wasEnabledLocal = currentlyEnabled;
        break;
      }

      case STATE_WRITE_FINAL: {
        char timeStr[11] = {0};  
        if (year > 20) {
          snprintf(timeStr, sizeof(timeStr), "%02d%02d%02d%02d%02d",
         year % 100, month, day, hour, minute);
          writeNewUsageLog(currentSessionId, String(timeStr), energy, lowestVoltage, lockedReader);
          Serial.println("[AUTO] Final usage log updated.");
        } else {
          Serial.println(timeStr);
          Serial.println("[AUTO] Final write skipped (invalid time).");
        }

        currentState = STATE_WAIT_FOR_DATA;
        //lowestVoltage = 0.0f; Do Not Reset lowest voltage across disable
        break;
      }
    }

    vTaskDelay(20);
  }
}

void TaskCANRxPD(void* pvParameters) {


  pdType = NO_PD;
  uint8_t dev_type = 0, manuf = 0, dev_num = 0;
  
  twai_message_t msg;

  Serial.println("[CANRxPD] Scanning for Power Distribution device...");

  for (;;) {
    if (twai_receive(&msg, pdMS_TO_TICKS(1000)) != ESP_OK || !msg.extd) continue;

    uint32_t id = msg.identifier;
    uint8_t this_type = (id >> 24) & 0x1F;
    uint8_t this_manuf = (id >> 16) & 0xFF;
    uint16_t api_id = (id >> 6) & 0x3FF;
    uint8_t this_dev_num = id & 0x3F;

    switch (pdType) {
      case NO_PD:
        if (this_type == CTRE_PDP_TYPE_ID && this_manuf == CTRE_PDP_MANUF_ID && api_id == CTRE_PDP_API_VOLTAGE) {
          pdType = CTRE_PDP;
          dev_type = this_type;
          manuf = this_manuf;
          dev_num = this_dev_num;
          Serial.printf("[CANRxPD] CTRE PDP detected (Device Number: %d)\n", dev_num);
        } else if (this_type == REV_PDH_TYPE_ID && this_manuf == REV_PDH_MANUF_ID && api_id == REV_PDH_API_ID) {
          pdType = REV_PDH;
          dev_type = this_type;
          manuf = this_manuf;
          dev_num = this_dev_num;
          Serial.printf("[CANRxPD] REV PDH detected (Device Number: %d)\n", dev_num);
        }
        break;

      case CTRE_PDP:
        if (this_type == dev_type && this_manuf == manuf && this_dev_num == dev_num) {
          switch (api_id) {
            case CTRE_PDP_API_VOLTAGE:
              PDvoltage = msg.data[6] * 0.05f + 4.0f;
              break;
            case CTRE_PDP_API_CURRENT:
              PDcurrent = ((msg.data[1] << 4) | (msg.data[2] >> 4)) * 0.125f;
              break;
          }

          if (voltage > 0) {
            //Serial.printf("[CANRxPD] Voltage: %.2f V | Current: %.2f A\n", PDvoltage, PDcurrent);
            //PDvoltage = PDcurrent = 0;
          }
          
        }
        break;

      case REV_PDH:
        if (this_type == dev_type && this_manuf == manuf && this_dev_num == dev_num && api_id == REV_PDH_API_ID) {
          uint16_t vbus_raw = ((msg.data[1] & 0x0F) << 8) | msg.data[0];
          PDvoltage = vbus_raw * 0.0078125f;
          PDcurrent = msg.data[4] * 2.0f;

          if (PDvoltage > 0) {
            //Serial.printf("[CANRxPD] Voltage: %.2f V | Current: %.2f A\n", PDvoltage, PDcurrent);
            //PDvoltage = PDcurrent = 0;
          }
        }
        break;
    }
  }
}

void TaskCANRxrioHeartbeat(void* pvParameters) {
  const unsigned long HEARTBEAT_TIMEOUT_MS = 5000;   // nominal frame spacing ~500 ms
  const uint8_t MAX_MISSED = 3;                      // require 3 consecutive misses
  uint8_t missedCount = 0;
  bool firstRead = true;
  bool heartbeatHealthy = false;                     // smoothed “OK” flag

  for (;;) {
    if (!canAvailable) {
      vTaskDelay(10);
      continue;
    }

    twai_message_t msg;
    // Drain all pending frames to avoid queue buildup
    while (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (msg.extd && msg.identifier == HEARTBEAT_ID && msg.data_length_code == 8) {
        uint64_t bits = 0;
        for (int i = 0; i < 8; ++i) bits = (bits << 8) | msg.data[i];

        auto get_bits = [](uint64_t v, int s, int l) -> int {
          return (v >> (64 - s - l)) & ((1ULL << l) - 1);
        };

        currentlyEnabled = get_bits(bits, 38, 1);
        lastHeartbeatMs  = millis();
        heartbeatOk      = true;

        // Update LED indicators
        heartbeatAvailable = true;
        heartbeatEnabled   = currentlyEnabled;

        year   = get_bits(bits, 26, 6) + 2000 - 36;
        month  = get_bits(bits, 22, 4) + 1;
        day    = get_bits(bits, 17, 5);
        hour   = get_bits(bits, 0, 5);
        minute = get_bits(bits, 5, 6);
        second = std::min(get_bits(bits, 11, 6), 59);

        if (firstRead || currentlyEnabled != wasEnabled) {
          firstRead = false;
          wasEnabled = currentlyEnabled;
          Serial.printf("[Heartbeat] %s | %.1f V  %d J | %04d-%02d-%02d %02d:%02d:%02d\n",
                        currentlyEnabled ? "ENABLED" : "DISABLED",
                        voltage, energy,
                        year, month, day, hour, minute, second);
        }
      }
      taskYIELD();   // prevent watchdog
    }

    // ---- Debounced timeout check ----
    bool okNow = (millis() - lastHeartbeatMs) <= HEARTBEAT_TIMEOUT_MS;

    if (!okNow) {
      if (missedCount < MAX_MISSED) missedCount++;
      else if (heartbeatHealthy) {
        heartbeatHealthy   = false;
        heartbeatOk        = false;
        heartbeatAvailable = false;
        heartbeatEnabled   = false;
        currentlyEnabled   = false;
        Serial.printf("[WARN] Heartbeat lost! No frame for %lu ms.\n",
                      millis() - lastHeartbeatMs);
      }
    } else {
      if (!heartbeatHealthy && missedCount >= MAX_MISSED)
        Serial.println("[INFO] Heartbeat restored.");
      missedCount = 0;
      heartbeatHealthy = true;
    }

    vTaskDelay(10);
  }
}



void TaskCANRxJava(void* pvParameters) {
  float rioVoltage = 0.0f;

  for (;;) {
    if (!canAvailable) {
      vTaskDelay(10);
      continue;
    }

    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK && msg.extd) {
      uint16_t apiId = (msg.identifier >> 6) & 0x3FF;

      if (apiId == BATTERY_STATUS_API_ID_1 && msg.data_length_code >= 7) {
        rioVoltage = msg.data[6] / 10.0f;
      } else if (apiId == BATTERY_STATUS_API_ID_2 && msg.data_length_code >= 2) {
        //energy = (msg.data[0] << 8) | msg.data[1];
        //No longer read energy from roborio
      }
    }

    // Always prefer PD voltage if available
    if (pdType != NO_PD && PDvoltage > 5.0f) {
      voltage = PDvoltage;
    } else if (rioVoltage > 5.0f) {
      voltage = rioVoltage;
    }

    if (voltage >= 5.0f && (lowestVoltage == 0.0f || voltage < lowestVoltage)) {
      lowestVoltage = voltage;
    }

    vTaskDelay(10);
  }
}

void TaskCANTx(void* pvParameters) {
  const uint32_t rfidMeta1 = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, RFID_META_API_ID_1, DEVICE_NUMBER);
  const uint32_t rfidMeta2 = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, RFID_META_API_ID_2, DEVICE_NUMBER);
  const uint32_t rfidMeta3 = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, RFID_META_API_ID_3, DEVICE_NUMBER);

  for (;;) {
    if (canAvailable && batteryMetaValid) {
      // Meta 1: SN part 1
      twai_message_t meta1 = {};
      meta1.identifier = rfidMeta1;
      meta1.extd = 1;
      meta1.data_length_code = 8;
      memset(meta1.data, 0, 8);
      strncpy((char*)meta1.data, batterySN, 8);
      twai_transmit(&meta1, pdMS_TO_TICKS(10));

      // Meta 2: SN part 2 + date (parsed from batteryFirstUseFull)
      twai_message_t meta2 = {};
      meta2.identifier = rfidMeta2;
      meta2.extd = 1;
      meta2.data_length_code = 8;
      memset(meta2.data, 0, 8);

      // Copy SN part 2 (up to 5 characters)
      if (strlen(batterySN) > 8) {
        strncpy((char*)meta2.data, batterySN + 8, 5);
      }

      // Parse year, month, day from batteryFirstUseFull (format: "yyMMddHHmm")
      int yy = atoi(String(batteryFirstUseFull).substring(0, 2).c_str());
      int mm = atoi(String(batteryFirstUseFull).substring(2, 4).c_str());
      int dd = atoi(String(batteryFirstUseFull).substring(4, 6).c_str());
      int fullYear = 2000 + yy;

      // Pack into CAN message
      meta2.data[4] = mm;
      meta2.data[5] = (fullYear >> 8) & 0xFF;
      meta2.data[6] = fullYear & 0xFF;
      meta2.data[7] = dd;

      twai_transmit(&meta2, pdMS_TO_TICKS(10));



      // Meta 3: cycles + note
      twai_message_t meta3 = {};
      meta3.identifier = rfidMeta3;
      meta3.extd = 1;
      meta3.data_length_code = 3;
      meta3.data[0] = (batteryCycleCount >> 8) & 0xFF;
      meta3.data[1] = batteryCycleCount & 0xFF;
      meta3.data[2] = batteryNote;
      twai_transmit(&meta3, pdMS_TO_TICKS(10));
    }
    vTaskDelay(100);
  }
}

void TaskEnergyCalc(void* pvParameters) {
  const TickType_t sampleInterval = pdMS_TO_TICKS(100);  // 100 ms
  const float dt = 0.1f;  // Time step in seconds
  static float localEnergyJ = 0.0f;  // Persistent local energy in J
  unsigned long lastPrint = millis();
  int lastKJ = 0;

  for (;;) {
    if (energy < INT32_MAX) {
      // Always accumulate, even if V or I = 0 (idle is part of energy tracking)
      localEnergyJ += voltage * PDcurrent * dt;
    }

    if (millis() - lastPrint >= 10000) { //print interval
      lastPrint = millis();

      int currentKJ = (int)((localEnergyJ / 1000.0f) + 0.5f);  // Round to nearest kJ
      int deltaKJ = currentKJ - lastKJ;

      if (deltaKJ > 0 && energy < INT32_MAX - deltaKJ) {
        energy += deltaKJ;
      } else if (energy >= INT32_MAX - deltaKJ) {
        energy = INT32_MAX;
      }

      Serial.printf("[EnergyCalc] +%d kJ | Total: %.1f J | (%d kJ) | V: %.2f V | I: %.2f A\n",
                    deltaKJ, localEnergyJ, energy, voltage, PDcurrent);

      lastKJ = currentKJ;
    }

    vTaskDelay(sampleInterval);
  }
}

// ==========================================================
// TaskLEDIndicator
//  R = reader / state
//  B = PD type
//  G = heartbeat
// ==========================================================
void TaskLEDIndicator(void* pvParameters) {


  for (;;) {
    // ---------------- RED LED ----------------
    if (!readerDetected) {
      digitalWrite(LED_R, LOW);
    } else {
      switch (currentState) {
        case STATE_WAIT_FOR_TAG:
        case STATE_PARSE_AND_WRITE_INITIAL:
          digitalWrite(LED_R, (millis() / 500) % 2);   // slow flash
          break;
        case STATE_WAIT_FOR_DATA:
          digitalWrite(LED_R, HIGH);                  // solid
          break;
        case STATE_WRITE_FINAL:
          digitalWrite(LED_R, (millis() / 100) % 2);  // quick flash
          break;
        default:
          digitalWrite(LED_R, LOW);
          break;
      }
    }

    // ---------------- BLUE LED ----------------
    switch (pdType) {
      case NO_PD:
        digitalWrite(LED_B, LOW);                     // off when no PD
        break;

      case CTRE_PDP:
      case REV_PDH:
        digitalWrite(LED_B, HIGH);                    // solid when CTRE or REV
        break;

      default:
        digitalWrite(LED_B, LOW);                     // off for unknown types
        break;
    }


    // ---------------- GREEN LED ----------------
    if (!heartbeatAvailable) {
      digitalWrite(LED_G, LOW);                      // off
    } else if (heartbeatEnabled) {
      digitalWrite(LED_G, (millis() / 300) % 2);     // blink
    } else {
      digitalWrite(LED_G, HIGH);                     // solid
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // update every 50 ms
  }
}


//=======READER Handing function DO NOT CHANGE========


String handleReader(MFRC522& reader, int readerNum) {
  const int MAX_BLOCKS = 64;
  String raw = "";
  bool blockValid[MAX_BLOCKS] = {false};
  byte blockData[MAX_BLOCKS][16];  // Final merged data

  reader.PCD_Reset();
  reader.PCD_Init();

  if (!reader.PICC_IsNewCardPresent() || !reader.PICC_ReadCardSerial()) {
    //Serial.printf("[Reader %d] Tag lost before reading\n", readerNum);
    return "";
  }

  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3) continue;  // Skip trailer blocks

    byte reads[3][16];
    bool success[3] = {false, false, false};

    byte sector = block / 4;
    byte trailer = sector * 4 + 3;

    for (int r = 0; r < 3; r++) {
      auto auth = reader.PCD_Authenticate(
        MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A,
        trailer,
        &keyA,
        &(reader.uid)
      );
      if (auth != MFRC522Constants::STATUS_OK) continue;

      byte buffer[18];
      byte size = sizeof(buffer);
      auto status = reader.MIFARE_Read(block, buffer, &size);

      if (status == MFRC522Constants::STATUS_OK && size >= 16) {
        memcpy(reads[r], buffer, 16);
        success[r] = true;
      }
    }

    // Compare 3 reads, accept if at least 2 match
    bool accepted = false;
    if (success[0] && success[1] && memcmp(reads[0], reads[1], 16) == 0) {
      memcpy(blockData[block], reads[0], 16);
      accepted = true;
    } else if (success[0] && success[2] && memcmp(reads[0], reads[2], 16) == 0) {
      memcpy(blockData[block], reads[0], 16);
      accepted = true;
    } else if (success[1] && success[2] && memcmp(reads[1], reads[2], 16) == 0) {
      memcpy(blockData[block], reads[1], 16);
      accepted = true;
    }

    if (accepted) {
      blockValid[block] = true;
      //Serial.printf("[Reader %d] Block %d HEX: ", readerNum, block);
      //for (byte i = 0; i < 16; i++) {
      //  Serial.printf("%02X ", blockData[block][i]);
      //}
      //Serial.println();
    } else {
      //Serial.printf("[Reader %d] Block %d failed validation.\n", readerNum, block);
    }
  }

  // Reconstruct raw
  raw = "";
  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3 || !blockValid[block]) continue;
    for (byte i = 0; i < 16; i++) raw += (char)blockData[block][i];
  }

  //Serial.printf("\n[Reader %d] Final RAW NDEF (len: %d bytes):\n", readerNum, raw.length());
  //Serial.println(raw);

  // Extract and verify JSON
  String cleanJson = extractJsonFromNdefText(raw);
  //Serial.println("[DEBUG] Extracted JSON:");
  //Serial.println(cleanJson);

  bool validStart = cleanJson.startsWith("{");
  bool validEnd = cleanJson.endsWith("}");
  bool hasSn = cleanJson.indexOf("\"sn\"") > 0;
  bool hasCc = cleanJson.indexOf("\"cc\"") > 0;
  bool hasU = cleanJson.indexOf("\"u\"") > 0;


  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
  delay(500);

  if (validStart && validEnd && hasSn && hasCc && hasU) {
    return cleanJson;
  } else {
    return "";  // Invalid or corrupted
  }
}

void printParsedBatteryJson(const String& json) {
  Serial.println(F("======= Battery Info ======="));

  int snIndex = json.indexOf("\"sn\":\"");
  if (snIndex != -1) {
    String sn = json.substring(snIndex + 6, json.indexOf("\"", snIndex + 6));
    sn.toCharArray(batterySN, sizeof(batterySN));  // ⬅️ Store to global
    Serial.print(F("Serial Number: ")); Serial.println(sn);
  }

  int fuIndex = json.indexOf("\"fu\":\"");
  if (fuIndex != -1) {
    String fu = json.substring(fuIndex + 6, json.indexOf("\"", fuIndex + 6));
  if (fu.length() >= 10) {
    strncpy(batteryFirstUseFull, fu.c_str(), sizeof(batteryFirstUseFull));
    batteryFirstUseFull[10] = '\0';  // Ensure null termination
  }

    Serial.print(F("First Use: ")); Serial.println(fu);
  }

  int ccIndex = json.indexOf("\"cc\":");
  if (ccIndex != -1) {
    int start = ccIndex + 5;
    int end = json.indexOf(",", start);
    if (end == -1) end = json.indexOf("}", start);
    if (end != -1) {
      batteryCycleCount = json.substring(start, end).toInt();  // ⬅️ Store to global
      Serial.print(F("Cycle Count: ")); Serial.println(batteryCycleCount);
    }
  }

  int nIndex = json.indexOf("\"n\":");
  if (nIndex != -1) {
    batteryNote = json.substring(nIndex + 4, json.indexOf(",", nIndex)).toInt();  //  Store to global
    Serial.print(F("Note Type: "));
    switch (batteryNote) {
      case 0: Serial.println(F("Normal")); break;
      case 1: Serial.println(F("Practice Only")); break;
      case 2: Serial.println(F("Scrap")); break;
      case 3: Serial.println(F("Other")); break;
      default: Serial.println(F("Unknown")); break;
    }
  }

  
  Serial.println(F("============================\n"));
}

int extractInt(const String& src, const char* key) {
  int i = src.indexOf(key);
  if (i == -1) return -1;
  return src.substring(i + strlen(key), src.indexOf(",", i + strlen(key))).toInt();
}

String extractString(const String& src, const char* key) {
  int i = src.indexOf(key);
  if (i == -1) return "";
  int start = i + strlen(key);
  int end = src.indexOf("\"", start);
  return src.substring(start, end);
}

String extractJsonFromNdefText(const String& raw) {
  String cleaned = "";

  // Keep only printable ASCII characters
  for (int i = 0; i < raw.length(); i++) {
    char c = raw[i];
    if (c >= 32 && c <= 126) {
      cleaned += c;
    }
  }

  // Find the first valid JSON object using brace matching
  int start = cleaned.indexOf('{');
  if (start == -1) return "";

  int depth = 0;
  for (int i = start; i < cleaned.length(); i++) {
    if (cleaned[i] == '{') depth++;
    else if (cleaned[i] == '}') depth--;

    if (depth == 0) {
      // Return the first full JSON block
      return cleaned.substring(start, i + 1);
    }
  }

  //Serial.println("[NDEF] JSON braces not balanced");
  return "";
}

void writeNewUsageLog(int eventId, const String& timeStr, int energy, float voltage, int readerId) {
  const int MAX_BLOCKS = 64;
  const int MAX_LOGS = 14;
  MFRC522& reader = (readerId == 2) ? mfrc2 : mfrc1;

  reader.PCD_Reset();
  reader.PCD_Init();

  if (!reader.PICC_IsNewCardPresent() || !reader.PICC_ReadCardSerial()) return;

  byte blockData[MAX_BLOCKS][16];
  bool blockValid[MAX_BLOCKS] = {false};
  String raw = "";

  // Step 1: Read and reconstruct JSON
  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3) continue;
    byte trailer = (block / 4) * 4 + 3;
    byte reads[3][16];
    bool success[3] = {false};

    for (int r = 0; r < 3; r++) {
      if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK)
        continue;
      byte buffer[18]; byte size = sizeof(buffer);
      if (reader.MIFARE_Read(block, buffer, &size) == MFRC522Constants::STATUS_OK && size >= 16) {
        memcpy(reads[r], buffer, 16);
        success[r] = true;
      }
    }

    if ((success[0] && success[1] && memcmp(reads[0], reads[1], 16) == 0) ||
        (success[0] && success[2] && memcmp(reads[0], reads[2], 16) == 0) ||
        (success[1] && success[2] && memcmp(reads[1], reads[2], 16) == 0)) {
      memcpy(blockData[block], reads[success[0] ? 0 : 1], 16);
      blockValid[block] = true;
    }
  }

  for (byte block = 4; block < MAX_BLOCKS; block++) {
    if (block % 4 == 3 || !blockValid[block]) continue;
    for (byte i = 0; i < 16; i++) raw += (char)blockData[block][i];
  }

  String cleanJson = extractJsonFromNdefText(raw);
  if (cleanJson == "") return;

  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, cleanJson)) return;

  JsonArray logs = doc["u"].as<JsonArray>();
  if (!logs) return;

  // Step 2: Remove all duplicates of eventId
  size_t index = 0;
  int foundCount = 0;
  while (index < logs.size()) {
    int id = logs[index]["i"] | 0;
    if (id == eventId) {
      if (foundCount == 0) {
        ++index;  // Keep the first match
        ++foundCount;
      } else {
        logs.remove(index);  // Remove duplicates
      }
    } else {
      ++index;
    }
  }

  // Step 3: Either overwrite the first matching or append if not found
  bool updated = false;
  for (JsonObject obj : logs) {
    if ((obj["i"] | 0) == eventId) {
      obj["t"] = timeStr;
      obj["d"] = 1;
      obj["e"] = energy;
      obj["v"] = voltage;
      updated = true;
      break;
    }
  }

  if (!updated) {
    // Remove oldest if over limit
    while (logs.size() >= MAX_LOGS) {
      size_t minIndex = 0;
      int minId = INT_MAX;
      for (size_t i = 0; i < logs.size(); i++) {
        int id = logs[i]["i"] | 0;
        if (id < minId) {
          minId = id;
          minIndex = i;
        }
      }
      logs.remove(minIndex);
    }

    JsonObject newEntry = logs.createNestedObject();
    newEntry["i"] = eventId;
    newEntry["t"] = timeStr;
    newEntry["d"] = 1;
    newEntry["e"] = energy;
    newEntry["v"] = voltage;
  }

  // Step 4: Encode and write as NDEF
  String newJson;
  serializeJson(doc, newJson);

  NdefMessage ndef;
  NdefRecord record;
  record.setTnf(TNF_WELL_KNOWN);
  const char* typeStr = "T";
  record.setType((const byte*)typeStr, 1);
  String lang = "en";
  String fullPayload = (char)lang.length() + lang + newJson;
  record.setPayload((const byte*)fullPayload.c_str(), fullPayload.length());
  ndef.addRecord(record);

  int msgLen = ndef.getEncodedSize();
  uint8_t buffer[msgLen + 6];
  int totalLen = 0;

  if (msgLen < 0xFF) {
    buffer[0] = 0x03;
    buffer[1] = msgLen;
    ndef.encode(&buffer[2]);
    totalLen = msgLen + 2;
  } else {
    buffer[0] = 0x03;
    buffer[1] = 0xFF;
    buffer[2] = (msgLen >> 8) & 0xFF;
    buffer[3] = msgLen & 0xFF;
    ndef.encode(&buffer[4]);
    totalLen = msgLen + 4;
  }

  if (totalLen < 752) buffer[totalLen++] = 0xFE;
  while (totalLen % 16 != 0) buffer[totalLen++] = 0x00;

  int block = 4;
  for (int i = 0; i < totalLen; i += 16) {
    if (block % 4 == 3) block++;
    byte trailer = (block / 4) * 4 + 3;
    if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Auth fail block %d\n", block);
      block++;
      continue;
    }
    if (reader.MIFARE_Write(block, &buffer[i], 16) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Failed block %d\n", block);
    }
    block++;
  }

  // Pad remaining blocks
  for (; block < 64; block++) {
    if (block % 4 == 3) continue;
    byte trailer = (block / 4) * 4 + 3;
    if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK) {

      continue;
    }
    byte blank[16] = {0};
    reader.MIFARE_Write(block, blank, 16);
  }

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
  Serial.println("[WRITE] Write complete.");
}
