/*
  ESP32 FRC-style CAN node (Corrected APIs)
  -----------------------------------------
  Pins:
    RED_PIN   = 15 (LEDC PWM)
    GREEN_PIN = 13 (LEDC PWM)
    BLUE_PIN  = 14 (LEDC PWM)
    RELAY_PIN = 25 (digital out)
    BUTTON_a  = 0  (IO0, INPUT_PULLUP)
    BUTTON_b  = 17 (INPUT_PULLUP)
    CAN TX    = GPIO 4
    CAN RX    = GPIO 5
    AnalogIn  = IO35 (ADC1_CH7)

  FRC CAN ID:
    encode_id = (DEVICE_ID<<24) | (MANUFACTURER_ID<<16) | (API_ID<<6) | DEVICE_NUMBER

  Protocol (corrected):
    roboRIO -> ESP32
      0x185 : [R, G, B, relay_bool, 0,0,0,0]
      0x186 : [software_ver, uptime_1, uptime_2, 0,0,0,0,0]
              uptime = uptime_1 | (uptime_2<<8) seconds; saturate at 0xFFFF
    ESP32 -> roboRIO
      0x195 : [ain_lo, ain_hi, btnA, btnB, 0,0,0,0]
      0x196 : [reset_device, 0,0,0,0,0,0,0]   (we send 0; we also accept incoming reset)

  Tasks:
    - taskCANRx     : handle 0x185 + 0x186 (and accept 0x196 reset for safety)
    - taskCANTx     : publish 0x195 (20 Hz) and 0x196 (2 Hz, reset flag=0)
    - taskAnalogRead: read/average analog input + buttons
    - TaskLEDPWM    : smooth LED PWM toward targets
    - TaskCANIDHelper: serial helper (&CANID SET/SAVE/GET)

  Serial:
    &CANID SET xx   (0..63 live)
    &CANID SAVE     (EEPROM + reboot)
    &CANID GET

  Notes:
    - IO0 is a boot strap; keep pullup. We read as INPUT_PULLUP (1=released).
    - LEDC 12-bit (0..4095), incoming 0..255 scaled by <<4.
*/

#include <Arduino.h>
#include <EEPROM.h>
#include "driver/twai.h"
#include "driver/ledc.h"
 


// =================== Pins ===================
#define RED_PIN    15
#define GREEN_PIN  13
#define BLUE_PIN   14

#define RELAY_PIN  25
#define BUTTON_a   0   // IO0
#define BUTTON_b   17

#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

#define ANALOG_PIN  35 // IO35 (ADC1_CH7)

// =================== FRC CAN Constants ===================
#define DEVICE_ID             0x0A // DO NOT change
#define MANUFACTURER_ID       0x08 // DO NOT change
#define DEFAULT_DEVICE_NUMBER 9    // default DN

// APIs (corrected)
#define API_RX_CONTROL  0x185  // R,G,B,relay
#define API_RX_STATUS   0x186  // software_ver, uptime[0..1]
#define API_TX_INPUTS   0x195  // analog + buttons
#define API_TX_RESET    0x196  // reset flag (ESP->RIO)

// Your ESP-side software version (0..255). Not transmitted per new spec, but kept for reference.
#define SOFTWARE_VER    1

// =================== LEDC (PWM) ===================
// Old:
// static const int LEDC_FREQ = 4000;
// static const int LEDC_RES  = 12;
// static const int CH_RED    = 0;
// static const int CH_GREEN  = 1;
// static const int CH_BLUE   = 2;

// New (IDF):
static const uint32_t LEDC_FREQ_HZ = 4000;         // 4 kHz
static const ledc_timer_bit_t LEDC_RES_BITS = LEDC_TIMER_12_BIT;

static const ledc_mode_t  LEDC_MODE  = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;

static const ledc_channel_t LEDC_CH_RED   = LEDC_CHANNEL_0;
static const ledc_channel_t LEDC_CH_GREEN = LEDC_CHANNEL_1;
static const ledc_channel_t LEDC_CH_BLUE  = LEDC_CHANNEL_2;


// =================== EEPROM ===================
#define EEPROM_BYTES 64

// =================== Shared State ===================
volatile uint8_t  g_deviceNumber = DEFAULT_DEVICE_NUMBER;

volatile uint8_t  g_ledR_8 = 0, g_ledG_8 = 0, g_ledB_8 = 0;
volatile bool     g_relayOn = false;

volatile uint16_t g_ain_bits = 0; // 0..4095
volatile uint8_t  g_btnA = 1;     // 1=released (pullup)
volatile uint8_t  g_btnB = 1;

// LED smoothing (12-bit)
volatile uint16_t g_ledR_12 = 0, g_ledG_12 = 0, g_ledB_12 = 0;
volatile uint16_t g_ledR_target = 0, g_ledG_target = 0, g_ledB_target = 0;

// From 0x186 (RIO->ESP)
volatile uint8_t  g_rioSoftwareVer = 0;
volatile uint16_t g_rioUptimeSec   = 0;
volatile bool     g_rioUptimeSaturated = false;

// Local uptime (for your own debug; saturates at 0xFFFF)
volatile uint16_t g_espUptimeSec   = 0;

// =================== FRC CAN ID helpers ===================
static inline uint32_t encode_id(uint8_t dt, uint8_t man, uint16_t api, uint8_t dn) {
  return ((uint32_t)dt << 24) | ((uint32_t)man << 16) | ((uint32_t)api << 6) | (dn & 0x3F);
}
static inline void decode_id(uint32_t id, uint8_t &dt, uint8_t &man, uint16_t &api, uint8_t &dn) {
  dt  = (id >> 24) & 0xFF;
  man = (id >> 16) & 0xFF;
  api = (id >> 6)  & 0x3FF;
  dn  = id & 0x3F;
}
static inline uint32_t make_can_id(uint16_t api) {
  return encode_id(DEVICE_ID, MANUFACTURER_ID, api, (uint8_t)g_deviceNumber);
}

// =================== EEPROM: Device Number ===================
void EEPROMReadCANID() {
  uint8_t saved = EEPROM.read(0);
  g_deviceNumber = (saved <= 63) ? saved : DEFAULT_DEVICE_NUMBER;
}
void EEPROMSaveCANID() {
  EEPROM.write(0, (uint8_t)g_deviceNumber);
  EEPROM.commit();
}

// =================== TWAI (CAN) ===================
bool can_started = false;

bool CAN_Start() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 10;
  g_config.rx_queue_len = 20;
  g_config.clkout_divider = 0;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[CAN] Driver install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[CAN] Start failed");
    twai_driver_uninstall();
    return false;
  }
  can_started = true;
  Serial.println("[CAN] Started at 1 Mbps");
  return true;
}

bool CAN_Send(uint32_t id, const uint8_t data[8], int dlc = 8) {
  if (!can_started) return false;
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 1;  // 29-bit
  msg.data_length_code = dlc;
  for (int i = 0; i < dlc && i < 8; ++i) msg.data[i] = data[i];
  return twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK;
}

// =================== Tasks ===================
void taskCANRx(void* parameter) {
  Serial.println("[CANRX] Task started");
  while (true) {
    twai_message_t rx;
    esp_err_t r = twai_receive(&rx, pdMS_TO_TICKS(100));
    if (r == ESP_OK && rx.extd) {
      uint8_t dt, man, dn;
      uint16_t api;
      decode_id(rx.identifier, dt, man, api, dn);

      // Only react to our type/manufacturer + matching device number
      if (dt == DEVICE_ID && man == MANUFACTURER_ID && dn == (uint8_t)g_deviceNumber) {
        if (api == API_RX_CONTROL && rx.data_length_code >= 4) {
          // 0x185: R,G,B,relay
          uint8_t R = rx.data[0], G = rx.data[1], B = rx.data[2];
          bool relay = rx.data[3] ? true : false;

          g_ledR_8 = R; g_ledG_8 = G; g_ledB_8 = B; g_relayOn = relay;

          g_ledR_target = ((uint16_t)R) << 4;
          g_ledG_target = ((uint16_t)G) << 4;
          g_ledB_target = ((uint16_t)B) << 4;
        }
        else if (api == API_RX_STATUS && rx.data_length_code >= 3) {
          // 0x186: software_ver, uptime_1, uptime_2
          g_rioSoftwareVer = rx.data[0];
          uint16_t up = (uint16_t)rx.data[0 + 1] | ((uint16_t)rx.data[0 + 2] << 8);
          g_rioUptimeSec = up;
          g_rioUptimeSaturated = (up == 0xFFFF);
        }
        else if (api == API_TX_RESET && rx.data_length_code >= 1) {
          // Accept incoming reset (keep this even though spec lists 0x196 as ESP->RIO)
          if (rx.data[0] == 1) {
            Serial.println("[CANRX] Reset request received. Rebooting...");
            delay(50);
            ESP.restart();
          }
        }
      }
    }

    // Apply relay quickly
    digitalWrite(RELAY_PIN, g_relayOn ? HIGH : LOW);
  }
}

void taskCANTx(void* parameter) {
  Serial.println("[CANTX] Task started");
  uint32_t last195 = 0;
  uint32_t last196 = 0;
  uint32_t lastSecTick = millis();

  while (true) {
    uint32_t now = millis();

    // Local uptime (debug) — saturate at 0xFFFF
    if (now - lastSecTick >= 1000) {
      lastSecTick += 1000;
      if (g_espUptimeSec < 0xFFFF) g_espUptimeSec++;
    }

    // 0x195 at ~20 Hz
    if (now - last195 >= 50) {
      last195 = now;
      uint8_t buf195[8] = {0};
      uint16_t ain = g_ain_bits;
      buf195[0] = (uint8_t)(ain & 0xFF);
      buf195[1] = (uint8_t)((ain >> 8) & 0xFF);
      buf195[2] = (uint8_t)(g_btnA ? 1 : 0);
      buf195[3] = (uint8_t)(g_btnB ? 1 : 0);
      CAN_Send(make_can_id(API_TX_INPUTS), buf195, 8);
    }

    // 0x196 at ~2 Hz (reset flag = 0)
    if (now - last196 >= 500) {
      last196 = now;
      uint8_t buf196[8] = {0};
      buf196[0] = 0; // we are not requesting a reset
      CAN_Send(make_can_id(API_TX_RESET), buf196, 8);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskAnalogRead(void* parameter) {
  Serial.println("[ANALOG] Task started");
  analogReadResolution(12);
  // analogSetPinAttenuation(ANALOG_PIN, ADC_11db); // if needed for your board

  const int N = 8;
  while (true) {
    uint32_t acc = 0;
    for (int i = 0; i < N; ++i) {
      int v = analogRead(ANALOG_PIN);
      if (v < 0) v = 0;
      if (v > 4095) v = 4095;
      acc += (uint32_t)v;
      delayMicroseconds(500);
    }
    g_ain_bits = (uint16_t)(acc / N);

    // Buttons (active-low)
    g_btnA = digitalRead(BUTTON_a) ? 1 : 0;
    g_btnB = digitalRead(BUTTON_b) ? 1 : 0;

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskLEDPWM(void* parameter) {
  Serial.println("[LEDPWM] Task started");
  const uint16_t step = 64; // speed of fade per 10ms

  while (true) {
    // Make local, non-volatile copies first
    uint16_t curR = g_ledR_12;
    uint16_t curG = g_ledG_12;
    uint16_t curB = g_ledB_12;

    uint16_t tgtR = g_ledR_target;
    uint16_t tgtG = g_ledG_target;
    uint16_t tgtB = g_ledB_target;

    // R
    if (curR < tgtR) {
      uint32_t next = (uint32_t)curR + step;
      if (next > tgtR) next = tgtR;
      curR = (uint16_t)next;
    } else if (curR > tgtR) {
      uint16_t next = (curR > step) ? (curR - step) : 0;
      if (next < tgtR) next = tgtR;
      curR = next;
    }

    // G
    if (curG < tgtG) {
      uint32_t next = (uint32_t)curG + step;
      if (next > tgtG) next = tgtG;
      curG = (uint16_t)next;
    } else if (curG > tgtG) {
      uint16_t next = (curG > step) ? (curG - step) : 0;
      if (next < tgtG) next = tgtG;
      curG = next;
    }

    // B
    if (curB < tgtB) {
      uint32_t next = (uint32_t)curB + step;
      if (next > tgtB) next = tgtB;
      curB = (uint16_t)next;
    } else if (curB > tgtB) {
      uint16_t next = (curB > step) ? (curB - step) : 0;
      if (next < tgtB) next = tgtB;
      curB = next;
    }

    // Write back once
    g_ledR_12 = curR;
    g_ledG_12 = curG;
    g_ledB_12 = curB;

    // Apply PWM
    ledc_set_duty(LEDC_MODE, LEDC_CH_RED,   curR);
    ledc_update_duty(LEDC_MODE, LEDC_CH_RED);

    ledc_set_duty(LEDC_MODE, LEDC_CH_GREEN, curG);
    ledc_update_duty(LEDC_MODE, LEDC_CH_GREEN);

    ledc_set_duty(LEDC_MODE, LEDC_CH_BLUE,  curB);
    ledc_update_duty(LEDC_MODE, LEDC_CH_BLUE);


    // Keep relay in sync (optional redundancy vs. CAN RX task)
    digitalWrite(RELAY_PIN, g_relayOn ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}



// =================== Serial CAN-ID Helper ===================
void TaskCANIDHelper(void* parameter) {
  Serial.println("[CANID] Helper task started. Use &CANID SET xx / SAVE / GET");
  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("&CANID SET ")) {
        int val = line.substring(11).toInt();
        if (val >= 0 && val <= 63) {
          g_deviceNumber = (uint8_t)val;
          Serial.printf("[CANID] Running DEVICE_NUMBER set to %d\n", (int)g_deviceNumber);
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
        Serial.printf("[CANID] Current=%d, EEPROM=%d, Default=%d\n",
                      (int)g_deviceNumber, (int)eepromVal, (int)DEFAULT_DEVICE_NUMBER);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =================== Setup / Loop ===================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== ESP32 FRC CAN Node (Corrected APIs) ===");

  // EEPROM
  EEPROM.begin(EEPROM_BYTES);
  EEPROMReadCANID();
  Serial.printf("[BOOT] DEVICE_NUMBER=%d\n", (int)g_deviceNumber);

  // GPIO
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(BUTTON_a, INPUT_PULLUP);
  pinMode(BUTTON_b, INPUT_PULLUP);

  // LEDC
  // --- IDF LEDC timer config ---
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode       = LEDC_MODE;
  tcfg.duty_resolution  = LEDC_RES_BITS;
  tcfg.timer_num        = LEDC_TIMER;
  tcfg.freq_hz          = LEDC_FREQ_HZ;
  tcfg.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);

  // --- IDF LEDC channel config ---
  ledc_channel_config_t chR = {};
  chR.gpio_num   = RED_PIN;
  chR.speed_mode = LEDC_MODE;
  chR.channel    = LEDC_CH_RED;
  chR.intr_type  = LEDC_INTR_DISABLE;
  chR.timer_sel  = LEDC_TIMER;
  chR.duty       = 0;
  chR.hpoint     = 0;
  ledc_channel_config(&chR);

  ledc_channel_config_t chG = chR; chG.gpio_num = GREEN_PIN; chG.channel = LEDC_CH_GREEN;
  ledc_channel_config_t chB = chR; chB.gpio_num = BLUE_PIN;  chB.channel = LEDC_CH_BLUE;
  ledc_channel_config(&chG);
  ledc_channel_config(&chB);


  // CAN
  if (!CAN_Start()) {
    Serial.println("[BOOT] CAN init failed. Retrying in 3s...");
    delay(3000);
    CAN_Start();
  }

  // Tasks
  xTaskCreatePinnedToCore(taskCANRx,       "taskCANRx",       4096, nullptr, 3, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskCANTx,       "taskCANTx",       4096, nullptr, 2, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskAnalogRead,  "taskAnalogRead",  4096, nullptr, 2, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(TaskLEDPWM,      "TaskLEDPWM",      4096, nullptr, 2, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(TaskCANIDHelper, "TaskCANIDHelper", 4096, nullptr, 1, nullptr, APP_CPU_NUM);

  Serial.println("[BOOT] Setup complete.");
}

void loop() {
  // all logic in tasks
}
