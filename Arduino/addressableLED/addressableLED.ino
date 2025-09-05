#include <Arduino.h>
#include <FastLED.h>
#include "driver/twai.h"
#include <EEPROM.h>



#include "pattern.h"
#include "mode.h"

// —— LED Configuration ——
const uint16_t NUM_LEDS = 60;
CRGB          leds[NUM_LEDS];
#define DATA_PIN    6
#define BRIGHTNESS  128 //Max brightness limiter. This can't be overwritten on CAN. 
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

// —— Default Startup Settings ——
#define DEFAULT_MODE       1 //This mode and presetwill run on startup until an update is received on CAN
#define DEFAULT_R        255 
#define DEFAULT_G        255
#define DEFAULT_B        255
#define DEFAULT_W          0
#define DEFAULT_BRIGHTNESS 128
#define DEFAULT_ONOFF      1

// Add to shared state
volatile uint8_t  canParam0 = 20; //default canParam on startup
volatile uint8_t  canParam1 = 20; //default canParam on startup

// Button Settings
const uint8_t buttonModes[] = {0, 1, 2, 3, 4, 5, 6, 7, 255}; //What mode IO0 button cycle through (for debugging).
//const uint8_t buttonModes[] = {0}; //use this if you wnat button to turn off all LED. This can be used during competition if you don't want to be annoyed by LEDs.

// —— FRC CAN Constants ——
#define DEVICE_ID           0x0A //DO NOT change these
#define MANUFACTURER_ID     0x08 //DO NOT change these
#define DEFAULT_DEVICE_NUMBER        33 //Can be changed via serial
#define GENERAL_API         0x350
#define CUSTOM_PATTERN_API  0x351  // through 0x358
#define ESP_FEEDBACK_API    0x359

uint8_t DEVICE_NUMBER = DEFAULT_DEVICE_NUMBER;

// —— TWAI (CAN) Pins & Speed ——
#define CAN_TX_PIN  GPIO_NUM_8
#define CAN_RX_PIN  GPIO_NUM_9
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// —— Shared State (initialized to defaults) ——
volatile uint8_t  canMode   = DEFAULT_MODE;
volatile uint8_t  canR      = DEFAULT_R;
volatile uint8_t  canG      = DEFAULT_G;
volatile uint8_t  canB      = DEFAULT_B;
volatile uint8_t  canW      = DEFAULT_W;
volatile uint8_t  canBrig   = DEFAULT_BRIGHTNESS;
volatile bool     canOnOff  = DEFAULT_ONOFF;

volatile bool modeRefresh = true;  // force initial draw


// Custom-pixel frame
volatile bool     customSeen = false;
volatile uint16_t customPix  = 0;
volatile uint8_t  cR = 0, cG = 0, cB = 0, cW = 0, cBrig = 0;

// Fallback auto-swap every 10 s
//const uint32_t FALLBACK_INTERVAL = 10000;
uint32_t       lastSwap         = 0;

// Handle for our LED-task so we can restart it
TaskHandle_t   xHandleLED       = NULL;



const size_t numButtonModes = sizeof(buttonModes) / sizeof(buttonModes[0]);
size_t currentModeIndex = 0;




// —— Helpers ——
/// Build a full 29-bit FRC CAN ID
uint32_t makeCANMsgID(uint8_t deviceID,
                      uint8_t manufacturerID,
                      uint16_t apiID,
                      uint8_t deviceNumber) {
  return ((uint32_t)(deviceID       & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID          & 0x3FF) << 6 ) |
         ( deviceNumber             & 0x3F);
}

/// Send the 0x359 feedback frame



void sendFeedback() {
  twai_message_t tx{};
  tx.identifier       = makeCANMsgID(DEVICE_ID,
                                     MANUFACTURER_ID,
                                     ESP_FEEDBACK_API,
                                     DEVICE_NUMBER);
  tx.extd             = true;
  tx.rtr              = false;
  tx.data_length_code = 8;
  tx.data[0]          = (NUM_LEDS >> 8) & 0xFF;
  tx.data[1]          =  NUM_LEDS       & 0xFF;
  tx.data[2]          =  canMode;
  for (int i = 3; i < 8; i++) tx.data[i] = 0;
  twai_transmit(&tx, pdMS_TO_TICKS(10));
}

// —— Task Prototypes ——
void TaskLEDWrite(void* pvParameters);
void TaskCANRx   (void* pvParameters);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting FRC-CAN + NeoPixel demo");
  EEPROM.begin(4);
  EEPROMReadCANID();
  // — Button on IO0 to cycle modes —
  pinMode(9, INPUT_PULLUP);

  // — FastLED setup —
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  // — TWAI setup —
  twai_general_config_t g_config;
  g_config.mode           = TWAI_MODE_NORMAL;
  g_config.tx_io          = CAN_TX_PIN;
  g_config.rx_io          = CAN_RX_PIN;
  g_config.clkout_io      = TWAI_IO_UNUSED;
  g_config.bus_off_io     = TWAI_IO_UNUSED;
  g_config.tx_queue_len   = 5;
  g_config.rx_queue_len   = 5;
  g_config.alerts_enabled = TWAI_ALERT_RX_DATA;
  g_config.clkout_divider = 0;

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("TWAI init error!");
    while (true) { delay(1000); }
  }
  Serial.println("TWAI @1 Mb ready");

  // — Spawn CAN Rx & feedback task (higher priority) —
  xTaskCreatePinnedToCore(
    TaskCANRx,   "CAN Task", 4096, NULL, 2, NULL, 0
  );

  // — Start LED task for the first time —
  xTaskCreatePinnedToCore(
    TaskLEDWrite, "LED Task", 4096, NULL, 1, &xHandleLED, 0
  );

  xTaskCreatePinnedToCore(TaskCANIDHelper, "TaskCANIDHelper", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Idle; tasks do all the work
  vTaskDelay(portMAX_DELAY);
}

// —— LED Task ——
// Runs the pattern in `canMode`, auto-cycles every FALLBACK_INTERVAL.


void TaskLEDWrite(void* pv) {
  (void)pv;
  for (;;) {
    runCurrentMode();  // calls into mode.cpp
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms tick
  }
}


// —— CAN Rx & Feedback Task ——
// Listens for extended IDs 0x350–0x358, updates globals, restarts LED task on mode change,
// polls IO0 to cycle mode, and *always* sends 0x359 feedback at ~50 Hz.
void TaskCANRx(void* pv) {
  (void)pv;
  twai_message_t rx{};
  bool lastBtn = digitalRead(0);

  for (;;) {
    uint32_t alerts;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(50)) == ESP_OK) {
      if (alerts & TWAI_ALERT_RX_DATA) {
        while (twai_receive(&rx, 0) == ESP_OK) {
          if (!rx.extd) continue;

          uint32_t id  = rx.identifier;
          uint16_t api = (id >> 6)  & 0x3FF;
          uint8_t  dev =  id        & 0x3F;
          uint8_t  mfr = (id >> 16) & 0xFF;
          uint8_t  did = (id >> 24) & 0xFF;

          bool isBroadcast = (did == 0);
          bool isUnicast   = (did == DEVICE_ID);

          if ((isUnicast || isBroadcast) &&
              mfr == MANUFACTURER_ID &&
              dev == DEVICE_NUMBER)
          {
            if (api == GENERAL_API) {
              uint8_t newMode = rx.data[0];
              if (true) {
                canR       = rx.data[1];
                canG       = rx.data[2];
                canB       = rx.data[3];
                canBrig    = rx.data[4];
                canOnOff   = rx.data[5];
                canParam0  = rx.data[6];
                canParam1  = rx.data[7];
                lastSwap   = millis();

                //  Always restart animation on any 0x350 command
                canMode = newMode;
                //if (xHandleLED) vTaskDelete(xHandleLED);
                //xTaskCreatePinnedToCore(
                //  TaskLEDWrite, "LED Task", 4096, NULL, 1, &xHandleLED, 1
                //);
                canMode     = newMode;
                modeRefresh = true;

                //Serial.printf("CAN set or updated mode %u, animation restarted\n", newMode);
              } else {
                Serial.printf("Ignored invalid CAN mode %u\n", newMode);
              }
            }
            else if (api >= CUSTOM_PATTERN_API && api < CUSTOM_PATTERN_API + 8) {
              customSeen = true;
              customPix  = (rx.data[0] << 8) | rx.data[1];
              cR         = rx.data[2];
              cG         = rx.data[3];
              cB         = rx.data[4];
              cW         = rx.data[5];
              cBrig      = rx.data[6];
            }
          }
        }
      }
    }

    // Local button handling
    bool btn = digitalRead(0);
    if (lastBtn == HIGH && btn == LOW) {
      currentModeIndex = (currentModeIndex + 1) % numButtonModes;
      uint8_t newMode = buttonModes[currentModeIndex];
      Serial.printf("Button → cycle to mode %u\n", newMode);
      canMode  = newMode;
      lastSwap = millis();
      if (xHandleLED) {
        vTaskDelete(xHandleLED);
      }
      xTaskCreatePinnedToCore(
        TaskLEDWrite, "LED Task", 4096, NULL, 1, &xHandleLED, 1
      );
    }
    lastBtn = btn;


    sendFeedback();
  }
}


void EEPROMReadCANID() {
  uint8_t saved = EEPROM.read(0);
  if (saved <= 63) DEVICE_NUMBER = saved;
}

void EEPROMSaveCANID() {
  EEPROM.write(0, DEVICE_NUMBER);
  EEPROM.commit();
}


void TaskCANIDHelper(void* parameter) {
  Serial.println("[CANID] Helper task started. Use &CANID SET xx / SAVE / GET");
  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("&CANID SET ")) {
        int val = line.substring(11).toInt();
        if (val >= 0 && val <= 63) {
          DEVICE_NUMBER = val;
          Serial.printf("[CANID] Running DEVICE_NUMBER set to %d\n", DEVICE_NUMBER);
        } else {
          Serial.println("[CANID] Invalid value. Must be 0–63.");
        }
      } else if (line.equals("&CANID SAVE")) {
        EEPROMSaveCANID();
        Serial.println("[CANID] Saved to EEPROM. Rebooting...");
        delay(1000);
        ESP.restart();
      } else if (line.equals("&CANID GET")) {
        uint8_t eepromVal = EEPROM.read(0);
        Serial.printf("[CANID] Current=%d, EEPROM=%d, Default=%d\n", DEVICE_NUMBER, eepromVal, DEFAULT_DEVICE_NUMBER);
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}