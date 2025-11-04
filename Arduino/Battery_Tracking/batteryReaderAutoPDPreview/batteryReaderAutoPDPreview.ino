#include <Arduino.h>  // Needed for FreeRTOS in Arduino context

// === Libraries ===
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <ArduinoJson.h>
#include <NdefMessage.h>
#include <NdefRecord.h>
#include "driver/twai.h"

// ==================================================
// ===============  HARDWARE DEFINITIONS  ============
// ==================================================
#define RST_PIN        22
#define CAN_TX_PIN     GPIO_NUM_4
#define CAN_RX_PIN     GPIO_NUM_5

// RGB LED Pins
#define LED_R 15
#define LED_G 13
#define LED_B 14

#define DISABLE_WRITE_DELAY_MS 1000

// ==================================================
// ===================  CAN CONSTANTS  ===============
// ==================================================

// FRC Device Identifiers
#define DEVICE_ID        0x0A   // Do not change
#define MANUFACTURER_ID  0x08   // Do not change
#define DEVICE_NUMBER    33     // Custom device number (0–63)

// CAN API IDs (FRC-style)
#define HEARTBEAT_ID             0x01011840
#define RFID_META_API_ID_1       0x131
#define RFID_META_API_ID_2       0x132
#define RFID_META_API_ID_3       0x133
#define BATTERY_STATUS_API_ID_1  0x135
#define BATTERY_STATUS_API_ID_2  0x136

// CTRE PDP
#define CTRE_PDP_TYPE_ID     0x08
#define CTRE_PDP_MANUF_ID    0x04
#define CTRE_PDP_API_VOLTAGE 0x052
#define CTRE_PDP_API_CURRENT 0x05D

// REV PDH
#define REV_PDH_TYPE_ID      0x08
#define REV_PDH_MANUF_ID     0x05
#define REV_PDH_API_ID       0x064

// ==================================================
// ===============  CAN MESSAGE BUILDER  =============
// ==================================================
uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

// ==================================================
// ==================  RFID DRIVERS  =================
// ==================================================
MFRC522DriverPinSimple ss_pin1(32);  // Reader 1 SS
MFRC522DriverPinSimple ss_pin2(33);  // Reader 2 SS

MFRC522DriverSPI driver1{ss_pin1};
MFRC522DriverSPI driver2{ss_pin2};

MFRC522 mfrc1{driver1};
MFRC522 mfrc2{driver2};
MFRC522::MIFARE_Key keyA;

// ==================================================
// ==================  TASK HEADERS  =================
// ==================================================
void TaskCANRxJava(void* pvParameters);
void TaskCANRxrioHeartbeat(void* pvParameters);
void TaskCANRxPD(void* pvParameters);
void TaskCANTx(void* pvParameters);
void TaskAutoBatteryManager(void* pvParameters);
void TaskLEDIndicator(void* pvParameters);
void TaskCANGlobalHandler(void* pvParameters);
void TaskLEDIndicator(void* pvParameters);

// ==================================================
// ===============  SYSTEM STATE GLOBALS  ============
// ==================================================
volatile bool canAvailable = false;
volatile bool lastEnabled = false;
volatile bool currentlyEnabled = false;
volatile bool wasEnabled = false;
volatile bool heartbeatOk = false;

// Time / Date
volatile int year = 0, month = 0, day = 0;
volatile uint8_t hour = 0, minute = 0, second = 0;

// Power
//volatile float voltage = 0.0f;
volatile float lowestVoltage = 0.0f;
volatile float PDvoltage = 0.0f;
volatile float PDcurrent = 0.0f;
volatile int energy = 0;
volatile int roboRIOenergy = 0;
volatile bool useRoboRIOEnergy = false;  // true = RIO energy, false = ESP
volatile float rioVoltage = 0.0f;  // Voltage value reported from roboRIO

// Timestamps for activity detection
unsigned long lastCANMsgTime = 0;
unsigned long lastPDMsgTime = 0;
unsigned long lastJavaMsgTime = 0;
unsigned long lastHeartbeatTime = 0;

// Global online status flags
bool canOnline = false;
bool pdOnline = false;
bool javaOnline = false;
bool heartbeatOnline = false;

// Global values
float globalVoltage = 0.0f;
float energyTotal = 0.0f;



volatile unsigned long lastHeartbeatMs = 0;

// ==================================================
// =============  BATTERY METADATA GLOBALS  ==========
// ==================================================
char batterySN[17] = "";            // 16 chars + null
uint16_t batteryFirstUse = 0;       // Encoded as MMDD
uint16_t batteryCycleCount = 0;
uint8_t batteryNote = 0;
bool batteryMetaValid = false;
int currentSessionId = 0;

volatile uint16_t authFailCount = 0;
volatile uint16_t totalWriteCount = 0;
char batteryFirstUseFull[11] = "";  // Format: "yyMMddHHmm"

// ==================================================
// ==================  ENUMERATIONS  =================
// ==================================================
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

// ==================================================
// ================  GLOBAL FLAGS  ==================
// ==================================================
extern volatile State currentState;
extern volatile PDType pdType;

bool readerDetected = false;
bool heartbeatAvailable = false;
bool heartbeatEnabled = false;



volatile PDType pdType = NO_PD;
volatile State currentState = STATE_WAIT_FOR_TAG;

// ==================================================
// ===============  FORWARD DECLARATIONS  ============
// ==================================================
String handleReader(MFRC522& reader, int readerNum);
void printParsedBatteryJson(const String& json);
String extractJsonFromNdefText(const String& raw);
int extractInt(const String& src, const char* key);
String extractString(const String& src, const char* key);
void writeNewUsageLog(int eventId, const String& timeStr, int energy, float voltage, int readerId);

// CAN handlers
void onCANMessage(const twai_message_t* msg);
void handleHeartbeat(const twai_message_t& msg);
void handleJavaCAN(const twai_message_t& msg);
void handlePD(const twai_message_t& msg);

// Queue
QueueHandle_t canRxQueue;

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

  // Known valid MFRC522 or FM17522 variants → anything except 0xFF is good
  if (ver1 != 0xFF) reader1OK = true;
  if (ver2 != 0xFF) reader2OK = true;


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
    .alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED,
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

  // === Create CAN RX Queue ===
  canRxQueue = xQueueCreate(20, sizeof(twai_message_t));
  if (canRxQueue == nullptr) {
    Serial.println(F("[CAN] Failed to allocate CAN RX queue!"));
  } else {
    // Start a task to poll TWAI alerts for RX messages
    xTaskCreatePinnedToCore(TaskCANAlertHandler, "CAN Alert", 4096, NULL, 2, NULL, 1);
    Serial.println(F("[CAN] Alert handler task started."));
  }


  Serial.println(F("System ready. Present tag to one reader or send a command."));

  // === Create Tasks ===
  xTaskCreatePinnedToCore(TaskAutoBatteryManager, "Battery Manager", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANTx,              "CAN TX",          4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx,              "CAN RX Unified",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANGlobalHandler,   "Global Data",     4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskLEDIndicator,       "LEDIndicator",    2048, NULL, 1, NULL, 1);
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


// ---------------------------------------
// Task: Handle TWAI alerts asynchronously
// ---------------------------------------
void TaskCANAlertHandler(void* pvParameters) {
  uint32_t alerts;
  twai_message_t msg;

  for (;;) {
    // Wait for alert flags (RX_DATA triggers when a new frame is received)
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(100)) == ESP_OK) {
      if (alerts & TWAI_ALERT_RX_DATA) {
        while (twai_receive(&msg, 0) == ESP_OK) {
          xQueueSend(canRxQueue, &msg, 0);
        }
      }

      if (alerts & TWAI_ALERT_BUS_OFF) {
        Serial.println("[CAN] Bus Off detected, recovering...");
        twai_initiate_recovery();
      }

      if (alerts & TWAI_ALERT_TX_FAILED) {
        Serial.println("[CAN] TX failed!");
      }
    }
    vTaskDelay(1);
  }
}


static void IRAM_ATTR twai_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        xQueueSendFromISR(canRxQueue, &msg, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// TaskCANRx
void TaskCANRx(void* pvParameters) {
  twai_message_t msg;
  for (;;) {
    if (xQueueReceive(canRxQueue, &msg, portMAX_DELAY) == pdPASS) {
      onCANMessage(&msg);   // <— pass by value/reference, not pointer
    }
  }
}


void TaskCANTx(void* pvParameters) {
  const uint32_t canID_SN     = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, 0x131, DEVICE_NUMBER);
  const uint32_t canID_Meta   = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, 0x132, DEVICE_NUMBER);
  const uint32_t canID_Status = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, 0x133, DEVICE_NUMBER);

  for (;;) {
    if (canAvailable) {

      // ==========================================================
      // 0x131 → Battery Serial (8 bytes)
      // ==========================================================
      twai_message_t msgSN = {};
      msgSN.identifier = canID_SN;
      msgSN.extd = 1;
      msgSN.data_length_code = 8;
      memset(msgSN.data, 0, 8);

      if (batteryMetaValid && strlen(batterySN) > 0) {
        strncpy((char*)msgSN.data, batterySN, 8);
      }

      twai_transmit(&msgSN, pdMS_TO_TICKS(10));

      // ==========================================================
      // 0x132 → Battery Metadata
      // yy mm dd hh mm cycle state
      // ==========================================================
      twai_message_t msgMeta = {};
      msgMeta.identifier = canID_Meta;
      msgMeta.extd = 1;
      msgMeta.data_length_code = 8;
      memset(msgMeta.data, 0, 8);

      if (batteryMetaValid) {
        // Parse date/time from batteryFirstUseFull (format "yyMMddHHmm")
        int yy = atoi(String(batteryFirstUseFull).substring(0, 2).c_str());
        int mm = atoi(String(batteryFirstUseFull).substring(2, 4).c_str());
        int dd = atoi(String(batteryFirstUseFull).substring(4, 6).c_str());
        int hh = atoi(String(batteryFirstUseFull).substring(6, 8).c_str());
        int mn = atoi(String(batteryFirstUseFull).substring(8, 10).c_str());

        msgMeta.data[0] = yy;
        msgMeta.data[1] = mm;
        msgMeta.data[2] = dd;
        msgMeta.data[3] = hh;
        msgMeta.data[4] = mn;
        msgMeta.data[5] = batteryCycleCount & 0xFF;
        msgMeta.data[6] = batteryNote;  // note or state
      }

      twai_transmit(&msgMeta, pdMS_TO_TICKS(10));

      // ==========================================================
      // 0x133 → ESP32 State
      // state, PDType, readerDetected, authFailCount(2B), totalWriteCount(2B), unused
      twai_message_t msgStatus = {};
      msgStatus.identifier = canID_Status;
      msgStatus.extd = 1;
      msgStatus.data_length_code = 8;
      memset(msgStatus.data, 0, 8);

      msgStatus.data[0] = currentState;
      msgStatus.data[1] = pdType;
      msgStatus.data[2] = (readerDetected ? 1 : 0);

      msgStatus.data[3] = (authFailCount >> 8) & 0xFF;
      msgStatus.data[4] = authFailCount & 0xFF;

      msgStatus.data[5] = (totalWriteCount >> 8) & 0xFF;
      msgStatus.data[6] = totalWriteCount & 0xFF;

      // Byte 7 unused
      twai_transmit(&msgStatus, pdMS_TO_TICKS(10));

    }

    vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update rate
  }
}

// ==================================================
// ===============  CAN MESSAGE HANDLER  =============
// ==================================================


// Called from TaskCANRx() when a CAN message is received
void onCANMessage(const twai_message_t* msg) {
  if (!msg || !msg->extd) return;

  lastCANMsgTime = millis();

  uint32_t id = msg->identifier;
  uint8_t deviceType = (id >> 24) & 0x1F;
  uint8_t manufacturer = (id >> 16) & 0xFF;
  uint16_t apiID = (id >> 6) & 0x3FF;
  uint8_t deviceNumber = id & 0x3F;

  // Route messages
  if (apiID == BATTERY_STATUS_API_ID_1 || apiID == BATTERY_STATUS_API_ID_2 ||
      apiID == RFID_META_API_ID_1 || apiID == RFID_META_API_ID_2 || apiID == RFID_META_API_ID_3) {
    handleJavaCAN(*msg);
    lastJavaMsgTime = millis();
  } 
  else if (apiID == CTRE_PDP_API_VOLTAGE || apiID == CTRE_PDP_API_CURRENT ||
           apiID == REV_PDH_API_ID) {
    handlePD(*msg);
    lastPDMsgTime = millis();
  }
  else if (id == HEARTBEAT_ID) {
    handleHeartbeat(*msg);
    lastHeartbeatTime = millis();
  }

  // Other CAN messages ignored, but count them for canOnline tracking
}

// ==================================================
// ============  GLOBAL DATA SUPERVISOR  =============
// ==================================================
void TaskCANGlobalHandler(void* pvParameters) {
  unsigned long lastLoop = millis();
  unsigned long lastPrintTime = 0;  
  float lastVoltage = 0.0f;
  float dt = 0.0f;

  bool lastPdOnline = true;
  bool lastHeartbeatOnline = true;
  bool lastJavaOnline = true;

  for (;;) {
    unsigned long now = millis();
    dt = (now - lastLoop) / 1000.0f;
    lastLoop = now;


    // --- Timeout detection (1000 ms) ---
    canOnline        = (now - lastCANMsgTime    < 1000);
    pdOnline         = (now - lastPDMsgTime     < 1000);
    javaOnline       = (now - lastJavaMsgTime   < 1000);
    heartbeatOnline  = (now - lastHeartbeatTime < 1000);

    // --- Heartbeat loss safety fallback ---
    if (!heartbeatOnline && currentlyEnabled) {
      wasEnabled = currentlyEnabled;
      currentlyEnabled = false;
      Serial.println("[Heartbeat] LOST — Robot DISABLED (timeout)");
    }

    // --- PD offline fallback ---
    if (!pdOnline) {
      PDvoltage = 0.0f;
      PDcurrent = 0.0f;
    }

    // --- Voltage source preference ---
    if (pdOnline && PDvoltage > 5.0f)
      globalVoltage = PDvoltage;
    else if (javaOnline && rioVoltage > 5.0f)
      globalVoltage = rioVoltage;
    else
      globalVoltage = 0.0f;

    // --- Track lowest voltage ---
    if (globalVoltage > 5.0f &&
        (lowestVoltage == 0.0f || globalVoltage < lowestVoltage))
      lowestVoltage = globalVoltage;

    // --- Energy calculation ---
    static float localEnergyTotal_J = 0.0f;
    float power_W = globalVoltage * PDcurrent;  // watts = volts * amps
    localEnergyTotal_J += power_W * dt;

    // --- Choose energy source ---
    if (useRoboRIOEnergy) {
      // RIO reports energy in kJ directly
      energyTotal = roboRIOenergy;
    } else {
      // Convert J → kJ (float, 1 decimal)
      energyTotal = localEnergyTotal_J / 1000.0f;
    }

    // --- Update integer global energy (kJ) for NFC / CAN TX ---
    energy = (int)roundf(energyTotal);

    // --- Debug print every 10 seconds or on loss events ---
    if (now - lastPrintTime > 3000)  {

      Serial.printf("[CANGlobal] CAN:%d HB:%d PD:%d Java:%d | PDType:%s | Robot:%s | V=%.2fV I=%.2fA E=%d kJ | LowestV=%.2f | %04d-%02d-%02d %02d:%02d:%02d\n",
                    canOnline, heartbeatOnline, pdOnline, javaOnline,
                    (pdType == REV_PDH) ? "REV_PDH" :
                    (pdType == CTRE_PDP) ? "CTRE_PDP" : "NONE",
                    currentlyEnabled ? "EN" : "DIS",
                    globalVoltage, PDcurrent, energy,
                    lowestVoltage,
                    year, month, day, hour, minute, second);

      lastPrintTime = now;
      lastPdOnline = pdOnline;
      lastHeartbeatOnline = heartbeatOnline;
      lastJavaOnline = javaOnline;
    }


    vTaskDelay(pdMS_TO_TICKS(50));  // Run every 50 ms
  }
}



// ==================================================
// =================  HANDLE PD =====================
// ==================================================
void handlePD(const twai_message_t& msg) {
  static uint8_t dev_type = 0, manuf = 0, dev_num = 0;

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
      } 
      else if (this_type == REV_PDH_TYPE_ID && this_manuf == REV_PDH_MANUF_ID && api_id == REV_PDH_API_ID) {
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
      }
      break;

    case REV_PDH:
      if (this_type == dev_type && this_manuf == manuf && this_dev_num == dev_num && api_id == REV_PDH_API_ID) {
        uint16_t vbus_raw = ((msg.data[1] & 0x0F) << 8) | msg.data[0];
        PDvoltage = vbus_raw * 0.0078125f;
        PDcurrent = msg.data[4] * 2.0f;
      }
      break;
  }

  //Serial.printf("[PD] Type:%d | V:%.2fV | I:%.2fA\n", pdType, PDvoltage, PDcurrent);
}

// ==================================================
// ===============  HANDLE JAVA CAN =================
// ==================================================
void handleJavaCAN(const twai_message_t& msg) {
  uint16_t api_id = (msg.identifier >> 6) & 0x3FF;

  static int lastOverrideState = 0;
  static bool overrideApplied = false;

  switch (api_id) {
    case 0x135: {  // RIO → ESP Control Frame
      rioVoltage        = msg.data[0] * 0.1f;   // voltage ×10
      uint8_t newOverride = msg.data[1];        // override command
      useRoboRIOEnergy  = msg.data[2];          // use RIO as PD
      // big-endian decode (MSB first)
      roboRIOenergy = ((uint16_t)msg.data[3] << 8) | (uint16_t)msg.data[4];


      bool rebootReq    = msg.data[5];

      lastJavaMsgTime = millis();
      javaOnline = true;

      // --- Reboot handling ---
      if (rebootReq) {
        Serial.println("[CAN] Reboot requested by RIO");
        ESP.restart();
      }

      // --- Override debounce handling ---
      if (newOverride != 0) {
        if (newOverride != lastOverrideState)
          overrideApplied = false;  // new command detected

        if (!overrideApplied) {
          switch (newOverride) {
            case 1:
              currentState = STATE_WAIT_FOR_TAG;
              Serial.println("[Override] STATE_WAIT_FOR_TAG");
              break;
            case 2:
              currentState = STATE_PARSE_AND_WRITE_INITIAL;
              Serial.println("[Override] STATE_PARSE_AND_WRITE_INITIAL");
              break;
            case 3:
              currentState = STATE_WAIT_FOR_DATA;
              Serial.println("[Override] STATE_WAIT_FOR_DATA");
              break;
            case 4:
              currentState = STATE_WRITE_FINAL;
              Serial.println("[Override] STATE_WRITE_FINAL");
              break;
            default:
              Serial.printf("[Override] Unknown override: %d\n", newOverride);
              break;
          }
          overrideApplied = true;
          lastOverrideState = newOverride;
        }
      } else {
        // override cleared
        if (lastOverrideState != 0) {
          Serial.println("[Override] Released (returning to automatic)");
        }
        lastOverrideState = 0;
        overrideApplied = false;
      }

      break;
    }

    case 0x136:
      // Reserved future telemetry frame
      break;

    default:
      break;
  }
}


void handleHeartbeat(const twai_message_t& msg) {
  if (!msg.extd || msg.identifier != HEARTBEAT_ID || msg.data_length_code != 8)
    return;  // Not a valid heartbeat frame

  // Assemble 8 bytes into one 64-bit big-endian word
  uint64_t bits = 0;
  for (int i = 0; i < 8; ++i)
    bits = (bits << 8) | msg.data[i];

  auto get_bits = [](uint64_t v, int s, int l) -> int {
    return (v >> (64 - s - l)) & ((1ULL << l) - 1);
  };

  // === Decode enable flag ===
  bool newEnabled = get_bits(bits, 38, 1);
  lastHeartbeatTime = millis();
  heartbeatOnline = true;

  if (newEnabled != currentlyEnabled) {
    wasEnabled = currentlyEnabled;
    currentlyEnabled = newEnabled;

    Serial.printf("[Heartbeat] Robot %s\n",
                  currentlyEnabled ? "ENABLED" : "DISABLED");
  }

  // === Decode datetime ===
  year   = get_bits(bits, 26, 6) + 2000 - 36;
  month  = get_bits(bits, 22, 4) + 1;
  day    = get_bits(bits, 17, 5);
  hour   = get_bits(bits, 0, 5);
  minute = get_bits(bits, 5, 6);
  second = std::min(get_bits(bits, 11, 6), 59);
}


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
          digitalWrite(LED_R, HIGH);                   // solid
          break;
        case STATE_WRITE_FINAL:
          digitalWrite(LED_R, (millis() / 100) % 2);   // quick flash
          break;
        default:
          digitalWrite(LED_R, LOW);
          break;
      }
    }

    // ---------------- BLUE LED (PD status) ----------------
    if (pdOnline) {
      digitalWrite(LED_B, HIGH);  // PD OK → solid blue
    } else if (!pdOnline && (millis() / 1000) % 2 == 0) {
      digitalWrite(LED_B, HIGH);  // blink every second if PD lost
    } else {
      digitalWrite(LED_B, LOW);   // off otherwise
    }

    // ---------------- GREEN LED (heartbeat / enable) ----------------
    if (!heartbeatOnline) {
      digitalWrite(LED_G, LOW);  // heartbeat lost → off
    } else if (currentlyEnabled) {
      digitalWrite(LED_G, (millis() / 300) % 2);  // enabled → blink
    } else {
      digitalWrite(LED_G, HIGH);  // disabled → solid
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

    // Try authentication
    if (reader.PCD_Authenticate(MFRC522Constants::PICC_CMD_MF_AUTH_KEY_A, trailer, &keyA, &(reader.uid)) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Auth fail block %d\n", block);
      if (authFailCount < 0xFFFF) authFailCount++;  // increment but cap at 65535
      block++;
      continue;
    }

    // Try to write
    if (reader.MIFARE_Write(block, &buffer[i], 16) != MFRC522Constants::STATUS_OK) {
      Serial.printf("[WRITE] Failed block %d\n", block);
      if (authFailCount < 0xFFFF) authFailCount++;  // increment but cap at 65535
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
  if (totalWriteCount < 0xFFFF) {
  totalWriteCount++;
  }
}
