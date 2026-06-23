#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <esp_system.h>
#include <esp_err.h>
#include "driver/twai.h"

/*
  Full demo firmware for the Waveshare ESP32-C3 test board.

  Hardware on this board:
  - CAN TX: GPIO8
  - CAN RX: GPIO9
  - WS281X data: GPIO10
  - Button A: GPIO0  (BOOT button, active low)
  - Button B: unavailable in this wiring
  - Reset button exists but is not readable in firmware
  - No relay
  - No analog input

  CAN protocol kept compatible with the older full-demo sketch:
    roboRIO -> ESP32
      0x185 : [R, G, B, aux, 0, 0, 0, 0]
              aux is accepted for compatibility but not used for hardware control
      0x186 : [software_ver, uptime_lo, uptime_hi, 0, 0, 0, 0, 0]
      0x196 : [1, 0, 0, 0, 0, 0, 0, 0] requests reboot

    ESP32 -> roboRIO
      0x195 : [ain_lo, ain_hi, btnA, btnB, 0, 0, 0, 0]
              buttons use INPUT_PULLUP semantics: 1 = released, 0 = pressed
              this board has no ADC, so ain is a random 12-bit test value
              here: btnA = BOOT(GPIO0), btnB = always released
      0x196 : [0, 0, 0, 0, 0, 0, 0, 0]
              periodic "reset flag" frame kept for compatibility
*/

#ifndef CAN_TX_PIN_NUM
#define CAN_TX_PIN_NUM 8
#endif

#ifndef CAN_RX_PIN_NUM
#define CAN_RX_PIN_NUM 9
#endif

static constexpr gpio_num_t CAN_TX_PIN = (gpio_num_t)CAN_TX_PIN_NUM;
static constexpr gpio_num_t CAN_RX_PIN = (gpio_num_t)CAN_RX_PIN_NUM;
static constexpr uint8_t LED_DATA_PIN = 10;
static constexpr uint8_t BUTTON_BOOT_PIN = 0;

static constexpr uint16_t NUM_LEDS = 1;
static constexpr uint8_t LED_BRIGHTNESS_LIMIT = 255;
static constexpr uint8_t BOOT_IDLE_R = 32;
static constexpr uint8_t BOOT_IDLE_G = 16;
static constexpr uint8_t BOOT_IDLE_B = 0;

#define LED_TYPE WS2812B
#define LED_COLOR_ORDER RGB

#define DEVICE_TYPE_ID        0x0A
#define MANUFACTURER_ID       0x08
#define DEFAULT_DEVICE_NUMBER 9

#define API_RX_CONTROL 0x185
#define API_RX_STATUS  0x186
#define API_TX_INPUTS  0x195
#define API_TX_RESET   0x196

static constexpr int EEPROM_BYTES = 8;
static constexpr int EEPROM_ADDR_DEVICE_NUMBER = 0;
static constexpr uint32_t INPUT_FRAME_PERIOD_MS = 50;
static constexpr uint32_t RESET_FRAME_PERIOD_MS = 500;
static constexpr uint32_t BUTTON_SAMPLE_PERIOD_MS = 5;
static constexpr uint32_t STATUS_PRINT_PERIOD_MS = 1000;

CRGB leds[NUM_LEDS];

uint8_t g_deviceNumber = DEFAULT_DEVICE_NUMBER;
uint8_t g_ledR = 0;
uint8_t g_ledG = 0;
uint8_t g_ledB = 0;
bool g_auxFlag = false;
bool g_ledDirty = true;

uint8_t g_rioSoftwareVer = 0;
uint16_t g_rioUptimeSec = 0;
uint16_t g_fakeVoltageBits = 0;

bool g_canStarted = false;
bool g_canRecovering = false;
uint32_t g_last195Ms = 0;
uint32_t g_last196Ms = 0;
uint32_t g_lastButtonSampleMs = 0;
uint32_t g_lastCanRxMs = 0;
uint32_t g_lastStatusPrintMs = 0;
uint32_t g_lastAlertPrintMs = 0;
uint32_t g_lastTxErrorPrintMs = 0;

struct DebouncedButton {
  uint8_t pin;
  bool rawReleased;
  bool released;
  uint32_t lastTransitionMs;
};

DebouncedButton g_buttonBoot{BUTTON_BOOT_PIN, true, true, 0};

uint32_t makeCanId(uint16_t apiId);
void readDeviceNumber();
void saveDeviceNumber();
bool startCan();
void handleSerial();
void serviceCan();
void handleCanFrame(const twai_message_t& msg);
bool sendCanFrame(uint16_t apiId, const uint8_t data[8], bool singleShot = true);
void sampleButtons();
void updateButton(DebouncedButton& button, uint32_t nowMs);
void updateLed();
void writeLed(uint8_t r, uint8_t g, uint8_t b);
void printTrace(const __FlashStringHelper* msg);
void printCanStatus(const char* tag);
const char* resetReasonToString(esp_reset_reason_t reason);
bool canPinsAreUnsafeForEsp32C3();

uint32_t makeCanId(uint16_t apiId) {
  return ((uint32_t)(DEVICE_TYPE_ID & 0xFF) << 24) |
         ((uint32_t)(MANUFACTURER_ID & 0xFF) << 16) |
         ((uint32_t)(apiId & 0x3FF) << 6) |
         (g_deviceNumber & 0x3F);
}

void printTrace(const __FlashStringHelper* msg) {
  Serial.println(msg);
  Serial.flush();
}

void writeLed(uint8_t r, uint8_t g, uint8_t b) {
  leds[0] = CRGB(r, g, b);
  FastLED.show();
}

const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "unknown";
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    case ESP_RST_USB:       return "usb";
    case ESP_RST_JTAG:      return "jtag";
    case ESP_RST_EFUSE:     return "efuse";
    case ESP_RST_PWR_GLITCH:return "power_glitch";
    case ESP_RST_CPU_LOCKUP:return "cpu_lockup";
    default:                return "unmapped";
  }
}

bool canPinsAreUnsafeForEsp32C3() {
  auto isFlashPin = [](gpio_num_t pin) -> bool {
    return pin >= GPIO_NUM_12 && pin <= GPIO_NUM_17;
  };
  return isFlashPin(CAN_TX_PIN) || isFlashPin(CAN_RX_PIN);
}

void printCanStatus(const char* tag) {
  if (!g_canStarted) {
    Serial.printf("[CAN] %s status unavailable (not started)\n", tag);
    Serial.flush();
    return;
  }

  twai_status_info_t status{};
  esp_err_t err = twai_get_status_info(&status);
  if (err != ESP_OK) {
    Serial.printf("[CAN] %s twai_get_status_info failed: %s (%d)\n",
                  tag, esp_err_to_name(err), (int)err);
    Serial.flush();
    return;
  }

  Serial.printf("[CAN] %s state=%d txq=%lu rxq=%lu tec=%lu rec=%lu txfail=%lu rxmiss=%lu rxovr=%lu buserr=%lu\n",
                tag,
                (int)status.state,
                (unsigned long)status.msgs_to_tx,
                (unsigned long)status.msgs_to_rx,
                (unsigned long)status.tx_error_counter,
                (unsigned long)status.rx_error_counter,
                (unsigned long)status.tx_failed_count,
                (unsigned long)status.rx_missed_count,
                (unsigned long)status.rx_overrun_count,
                (unsigned long)status.bus_error_count);
  Serial.flush();
}

void readDeviceNumber() {
  uint8_t stored = EEPROM.read(EEPROM_ADDR_DEVICE_NUMBER);
  if (stored <= 63) {
    g_deviceNumber = stored;
  } else {
    g_deviceNumber = DEFAULT_DEVICE_NUMBER;
  }
}

void saveDeviceNumber() {
  EEPROM.write(EEPROM_ADDR_DEVICE_NUMBER, g_deviceNumber);
  EEPROM.commit();
}

bool startCan() {
  if (canPinsAreUnsafeForEsp32C3()) {
    Serial.printf("[CAN] Refusing init on GPIO%d/GPIO%d\n", CAN_TX_PIN_NUM, CAN_RX_PIN_NUM);
    Serial.println("[CAN] ESP32-C3 GPIO12-17 are SPI flash pins. Using them for TWAI will crash the board.");
    Serial.println("[CAN] Rewire CAN to safe GPIOs such as 4/5 if your hardware allows it.");
    Serial.flush();
    return false;
  }

  Serial.printf("[CAN] Preparing init TX=%d RX=%d DN=%u\n", CAN_TX_PIN, CAN_RX_PIN, g_deviceNumber);
  Serial.flush();

  twai_general_config_t gConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  gConfig.tx_queue_len = 8;
  gConfig.rx_queue_len = 16;
  gConfig.alerts_enabled = TWAI_ALERT_RX_DATA |
                           TWAI_ALERT_TX_FAILED |
                           TWAI_ALERT_BUS_OFF |
                           TWAI_ALERT_BUS_RECOVERED;

  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  Serial.println("[CAN] Calling twai_driver_install");
  Serial.flush();
  esp_err_t err = twai_driver_install(&gConfig, &tConfig, &fConfig);
  if (err != ESP_OK) {
    Serial.printf("[CAN] Driver install failed: %s (%d)\n", esp_err_to_name(err), (int)err);
    Serial.flush();
    return false;
  }
  Serial.println("[CAN] Driver install OK");
  Serial.flush();

  Serial.println("[CAN] Calling twai_start");
  Serial.flush();
  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("[CAN] Start failed: %s (%d)\n", esp_err_to_name(err), (int)err);
    Serial.flush();
    twai_driver_uninstall();
    return false;
  }

  g_canStarted = true;
  g_canRecovering = false;
  Serial.printf("[CAN] Started at 1 Mbps on TX=%d RX=%d\n", CAN_TX_PIN, CAN_RX_PIN);
  Serial.flush();
  printCanStatus("after-start");
  return true;
}

bool sendCanFrame(uint16_t apiId, const uint8_t data[8], bool singleShot) {
  if (!g_canStarted) {
    return false;
  }

  twai_message_t msg{};
  msg.identifier = makeCanId(apiId);
  msg.extd = 1;
  msg.rtr = 0;
  msg.ss = singleShot ? 1 : 0;
  msg.data_length_code = 8;
  memcpy(msg.data, data, 8);
  esp_err_t err = twai_transmit(&msg, 0);
  if (err != ESP_OK) {
    uint32_t now = millis();
    if (now - g_lastTxErrorPrintMs >= 500U) {
      g_lastTxErrorPrintMs = now;
      Serial.printf("[CAN] twai_transmit api=0x%03X failed: %s (%d)\n",
                    apiId, esp_err_to_name(err), (int)err);
      Serial.flush();
    }
    return false;
  }
  return true;
}

void handleCanFrame(const twai_message_t& msg) {
  if (!msg.extd || msg.data_length_code != 8) {
    return;
  }

  uint32_t id = msg.identifier;
  uint8_t deviceType = (id >> 24) & 0xFF;
  uint8_t manufacturer = (id >> 16) & 0xFF;
  uint16_t apiId = (id >> 6) & 0x3FF;
  uint8_t deviceNumber = id & 0x3F;

  if (deviceType != DEVICE_TYPE_ID || manufacturer != MANUFACTURER_ID || deviceNumber != g_deviceNumber) {
    return;
  }

  g_lastCanRxMs = millis();

  if (apiId == API_RX_CONTROL) {
    g_ledR = msg.data[0];
    g_ledG = msg.data[1];
    g_ledB = msg.data[2];
    g_auxFlag = msg.data[3] != 0;
    g_ledDirty = true;

    Serial.printf("[CAN] 0x185 RGB=(%u,%u,%u) aux=%u\n",
                  g_ledR, g_ledG, g_ledB, g_auxFlag ? 1 : 0);
  } else if (apiId == API_RX_STATUS) {
    g_rioSoftwareVer = msg.data[0];
    g_rioUptimeSec = (uint16_t)msg.data[1] | ((uint16_t)msg.data[2] << 8);

    Serial.printf("[CAN] 0x186 sw=%u uptime=%u\n", g_rioSoftwareVer, g_rioUptimeSec);
  } else if (apiId == API_TX_RESET && msg.data[0] == 1) {
    Serial.println("[CAN] Reboot requested");
    delay(50);
    ESP.restart();
  }
}

void serviceCan() {
  if (!g_canStarted) {
    return;
  }

  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts != 0 && (alerts != TWAI_ALERT_RX_DATA)) {
      uint32_t now = millis();
      if (now - g_lastAlertPrintMs >= 200U) {
        g_lastAlertPrintMs = now;
        Serial.printf("[CAN] alerts=0x%08lX recovering=%u\n",
                      (unsigned long)alerts, g_canRecovering ? 1 : 0);
        Serial.flush();
      }
    }

    if (alerts & TWAI_ALERT_RX_DATA) {
      twai_message_t rx{};
      while (twai_receive(&rx, 0) == ESP_OK) {
        handleCanFrame(rx);
      }
    }

    if (alerts & TWAI_ALERT_TX_FAILED) {
      static uint32_t lastPrintMs = 0;
      uint32_t now = millis();
      if (now - lastPrintMs >= 1000U) {
        lastPrintMs = now;
        Serial.println("[CAN] TX failed (likely no ACK on bus)");
        Serial.flush();
      }
    }

    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("[CAN] Bus off, starting recovery");
      Serial.flush();
      twai_initiate_recovery();
      g_canRecovering = true;
      printCanStatus("bus-off");
    }

    if (g_canRecovering && (alerts & TWAI_ALERT_BUS_RECOVERED)) {
      if (twai_start() == ESP_OK) {
        Serial.println("[CAN] Bus recovered");
        Serial.flush();
        printCanStatus("bus-recovered");
      } else {
        Serial.println("[CAN] Bus recovered, restart failed");
        Serial.flush();
      }
      g_canRecovering = false;
    }
  }
}

void updateButton(DebouncedButton& button, uint32_t nowMs) {
  bool rawReleased = digitalRead(button.pin) != LOW;
  if (rawReleased != button.rawReleased) {
    button.rawReleased = rawReleased;
    button.lastTransitionMs = nowMs;
  }

  if ((nowMs - button.lastTransitionMs) >= 15U && button.released != button.rawReleased) {
    button.released = button.rawReleased;
    Serial.printf("[BTN] GPIO%u -> %s\n", button.pin, button.released ? "released" : "pressed");
  }
}

void sampleButtons() {
  uint32_t now = millis();
  if (now - g_lastButtonSampleMs < BUTTON_SAMPLE_PERIOD_MS) {
    return;
  }

  g_lastButtonSampleMs = now;
  updateButton(g_buttonBoot, now);
}

void updateLed() {
  if (!g_ledDirty) {
    return;
  }

  Serial.printf("[LED] apply rgb=(%u,%u,%u)\n", g_ledR, g_ledG, g_ledB);
  Serial.flush();
  writeLed(g_ledR, g_ledG, g_ledB);
  g_ledDirty = false;
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.startsWith("&CANID SET ")) {
    int value = line.substring(11).toInt();
    if (value >= 0 && value <= 63) {
      g_deviceNumber = (uint8_t)value;
      Serial.printf("[CANID] Running device number set to %u\n", g_deviceNumber);
    } else {
      Serial.println("[CANID] Invalid value, must be 0-63");
    }
  } else if (line.equals("&CANID SAVE")) {
    saveDeviceNumber();
    Serial.println("[CANID] Saved, rebooting");
    delay(100);
    ESP.restart();
  } else if (line.equals("&CANID GET")) {
    Serial.printf("[CANID] Current=%u EEPROM=%u Default=%u\n",
                  g_deviceNumber,
                  EEPROM.read(EEPROM_ADDR_DEVICE_NUMBER),
                  DEFAULT_DEVICE_NUMBER);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Full demo firmware: CAN + WS281X + buttons");
  Serial.printf("[BOOT] reset_reason=%s (%d)\n",
                resetReasonToString(esp_reset_reason()),
                (int)esp_reset_reason());
  Serial.flush();

  printTrace(F("[BOOT] step 1: EEPROM begin"));
  EEPROM.begin(EEPROM_BYTES);
  readDeviceNumber();
  Serial.printf("[CANID] Loaded device number %u\n", g_deviceNumber);
  Serial.flush();

  printTrace(F("[BOOT] step 2: pinMode buttons"));
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

  printTrace(F("[BOOT] step 3: FastLED / WS281X init"));
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, LED_COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS_LIMIT);
  FastLED.clear(true);

  printTrace(F("[BOOT] step 4: WS281X boot sweep"));
  writeLed(32, 0, 0);
  delay(120);
  writeLed(0, 32, 0);
  delay(120);
  writeLed(0, 0, 32);
  delay(120);
  writeLed(BOOT_IDLE_R, BOOT_IDLE_G, BOOT_IDLE_B);
  g_ledR = BOOT_IDLE_R;
  g_ledG = BOOT_IDLE_G;
  g_ledB = BOOT_IDLE_B;
  g_ledDirty = false;
  Serial.printf("[LED] boot hold rgb=(%u,%u,%u) on GPIO%u\n",
                g_ledR, g_ledG, g_ledB, LED_DATA_PIN);
  Serial.flush();

  printTrace(F("[BOOT] step 5: start CAN"));
  if (!startCan()) {
    Serial.println("[CAN] Init failed");
    Serial.flush();
  }

  Serial.println("[CANID] Use &CANID SET xx / SAVE / GET");
  Serial.println("[BOOT] CAN TX = GPIO8, CAN RX = GPIO9");
  Serial.println("[BOOT] Button A = BOOT(GPIO0), Button B unavailable");
  Serial.println("[BOOT] setup complete");
  Serial.flush();
}

void loop() {
  uint32_t now = millis();

  handleSerial();
  sampleButtons();
  serviceCan();
  updateLed();

  if (g_canStarted) {
    if (now - g_last195Ms >= INPUT_FRAME_PERIOD_MS) {
      g_last195Ms = now;

      uint8_t frame195[8] = {0};
      g_fakeVoltageBits = (uint16_t)(esp_random() & 0x0FFF);
      frame195[0] = (uint8_t)(g_fakeVoltageBits & 0xFF);
      frame195[1] = (uint8_t)((g_fakeVoltageBits >> 8) & 0xFF);
      frame195[2] = g_buttonBoot.released ? 1 : 0;
      frame195[3] = 1;
      sendCanFrame(API_TX_INPUTS, frame195, true);
    }

    if (now - g_last196Ms >= RESET_FRAME_PERIOD_MS) {
      g_last196Ms = now;

      uint8_t frame196[8] = {0};
      sendCanFrame(API_TX_RESET, frame196, true);
    }
  }

  if (now - g_lastStatusPrintMs >= STATUS_PRINT_PERIOD_MS) {
    g_lastStatusPrintMs = now;
    Serial.printf("[TRACE] ms=%lu can=%u recovering=%u ain=%u btnA=%u btnB=%u rgb=(%u,%u,%u) aux=%u lastRxAge=%lu\n",
                  (unsigned long)now,
                  g_canStarted ? 1 : 0,
                  g_canRecovering ? 1 : 0,
                  g_fakeVoltageBits,
                  g_buttonBoot.released ? 1 : 0,
                  1,
                  g_ledR, g_ledG, g_ledB,
                  g_auxFlag ? 1 : 0,
                  (unsigned long)(g_lastCanRxMs == 0 ? 0 : now - g_lastCanRxMs));
    Serial.flush();
    if (g_canStarted) {
      printCanStatus("periodic");
    }
  }

  delay(1);
}
