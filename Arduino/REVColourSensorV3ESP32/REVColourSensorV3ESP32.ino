/*
  REV Color Sensor V3 (APDS-9151) on ESP32 with FRC-style CAN
  -----------------------------------------------------------
  I2C:
    SDA = IO21
    SCL = IO22
    I2C address = 0x52

  CAN (TWAI):
    CAN_TX = GPIO 4
    CAN_RX = GPIO 5
    Bitrate = 1 Mbps, Extended 29-bit IDs

  RTOS Tasks:
    - TaskSensorRead : reads sensor, handles offline/re-init, applies config
    - TaskCANTx      : periodically transmits sensor data over CAN
    - TaskCANRx      : receives config / reboot frames from RIO

  CAN API usage (FRC-style 29-bit ID):
    CAN_ID = (DEVICE_TYPE_ID << 24) | (MANUFACTURER_ID << 16) | (API_ID << 6) | (DEVICE_NUMBER & 0x3F)

  API IDs and payloads:

    0x184 ESP -> RIO
      Byte 0-1: Red     (16-bit, high then low)
      Byte 2-3: Green   (16-bit)
      Byte 4-5: Blue    (16-bit)
      Byte 6-7: Prox    (16-bit)

    0x185 ESP -> RIO
      Byte 0-1: IR          (16-bit)
      Byte 2  : LEDPulseFreq enum
      Byte 3  : LEDCurrent enum
      Byte 4  : ProxResolution enum
      Byte 5  : ProxRate enum
      Byte 6  : ColorResolution enum
      Byte 7  : ColorRate enum

    0x186 ESP -> RIO
      Byte 0  : GainFactor enum
      Byte 1  : Sensor online (1 = online, 0 = offline)

    0x187 RIO -> ESP
      Byte 0  : ESP reboot (non-zero => ESP.restart())
      Byte 1  : LEDPulseFreq enum
      Byte 2  : LEDCurrent enum
      Byte 3  : ProxResolution enum
      Byte 4  : ProxRate enum
      Byte 5  : ColorResolution enum
      Byte 6  : ColorRate enum
      Byte 7  : GainFactor enum
*/
#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include <EEPROM.h>



// ========= I2C & PIN =========
static constexpr uint8_t I2C_ADDR = 0x52;
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;

// ========= CAN PINS =========
static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

// ========= CAN/FRC IDs =========
#define DEVICE_TYPE_ID        0x0A
#define MANUFACTURER_ID  0x08
//#define DEVICE_NUMBER    33

// =================== CAN Device Number via EEPROM ===================
#define EEPROM_ADDRESS 64
#define DEFAULT_DEVICE_NUMBER 33  // your fallback
volatile uint8_t g_deviceNumber = DEFAULT_DEVICE_NUMBER;

void EEPROMReadCANID() {
  uint8_t saved = EEPROM.read(0);
  g_deviceNumber = (saved <= 63) ? saved : DEFAULT_DEVICE_NUMBER;
}

void EEPROMSaveCANID() {
  EEPROM.write(0, g_deviceNumber);
  EEPROM.commit();
}



#define API_COLOR_DATA1   0x184
#define API_COLOR_DATA2   0x185
#define API_COLOR_STATUS  0x186
#define API_COLOR_CONFIG  0x187

static inline uint32_t makeCANMsgID(uint8_t deviceID,
                                    uint8_t manufacturerID,
                                    uint16_t apiID,
                                    uint8_t deviceNumber)
{
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (uint32_t)(deviceNumber & 0x3F);
}



// ===================================================================
// REV COLOR SENSOR ENUMS + REGISTERS
// ===================================================================
enum Register : uint8_t {
  REG_MAIN_CTRL                = 0x00,
  REG_PROXIMITY_SENSOR_LED     = 0x01,
  REG_PROXIMITY_SENSOR_PULSES  = 0x02,
  REG_PROXIMITY_SENSOR_RATE    = 0x03,
  REG_LIGHT_SENSOR_MEAS_RATE   = 0x04,
  REG_LIGHT_SENSOR_GAIN        = 0x05,
  REG_PART_ID                  = 0x06,
  REG_MAIN_STATUS              = 0x07,
  REG_PROXIMITY_DATA           = 0x08,
  REG_DATA_INFRARED            = 0x0A,
  REG_DATA_GREEN               = 0x0D,
  REG_DATA_BLUE                = 0x10,
  REG_DATA_RED                 = 0x13
};

// MAIN_CTRL bitfields (REV Color Sensor V3 / APDS-9151)
static constexpr uint8_t MAIN_CTRL_PROX_ENABLE = 0x01;   // Enables proximity engine
static constexpr uint8_t MAIN_CTRL_LS_ENABLE   = 0x02;   // Enables light/ALS engine
static constexpr uint8_t MAIN_CTRL_RGB_MODE    = 0x04;   // 1 = RGB mode, 0 = Clear mode


enum LEDPulseFreq : uint8_t {
  LED_FREQ_60k  = 0x18,
  LED_FREQ_70k  = 0x40,
  LED_FREQ_80k  = 0x28,
  LED_FREQ_90k  = 0x30,
  LED_FREQ_100k = 0x38
};
enum LEDCurrent : uint8_t {
  LED_CURR_2mA = 0,
  LED_CURR_5mA,
  LED_CURR_10mA,
  LED_CURR_25mA,
  LED_CURR_50mA,
  LED_CURR_75mA,
  LED_CURR_100mA,
  LED_CURR_125mA
};
enum ProxResolution : uint8_t {
  PROX_RES_8b  = 0x00,
  PROX_RES_9b  = 0x08,
  PROX_RES_10b = 0x10,
  PROX_RES_11b = 0x18
};
enum ProxRate : uint8_t {
  PROX_RATE_6ms   = 1,
  PROX_RATE_12ms  = 2,
  PROX_RATE_25ms  = 3,
  PROX_RATE_50ms  = 4,
  PROX_RATE_100ms = 5,
  PROX_RATE_200ms = 6,
  PROX_RATE_400ms = 7
};
enum ColorResolution : uint8_t {
  COLOR_RES_20b = 0x00,
  COLOR_RES_19b = 0x10,
  COLOR_RES_18b = 0x20,
  COLOR_RES_17b = 0x30,
  COLOR_RES_16b = 0x40,
  COLOR_RES_13b = 0x50
};
enum ColorRate : uint8_t {
  COLOR_RATE_25ms   = 0,
  COLOR_RATE_50ms   = 1,
  COLOR_RATE_100ms  = 2,
  COLOR_RATE_200ms  = 3,
  COLOR_RATE_500ms  = 4,
  COLOR_RATE_1000ms = 5,
  COLOR_RATE_2000ms = 7
};
enum GainFactor : uint8_t {
  GAIN_1X = 0,
  GAIN_3X = 1,
  GAIN_6X = 2,
  GAIN_9X = 3,
  GAIN_18X = 4
};

// ===================================================================
// SHARED SENSOR DATA
// ===================================================================
struct ColorSample {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t ir;
  uint16_t prox;
  bool     online;
  bool     haveSample;
};

static ColorSample gSample = {0,0,0,0,0,false,false};

static bool sensorOnline = false;
static bool lastReadError = false;
static bool rebootRequested = false;
static bool configDirty = false;

static uint8_t g_ledFreq  = LED_FREQ_100k;
static uint8_t g_ledCurr  = LED_CURR_50mA;
static uint8_t g_proxRes  = PROX_RES_11b;
static uint8_t g_proxRate = PROX_RATE_100ms;
static uint8_t g_colorRes = COLOR_RES_20b;
static uint8_t g_colorRate= COLOR_RATE_25ms;  // fastest allowed
static uint8_t g_gain     = GAIN_3X;

static portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE cfgMux    = portMUX_INITIALIZER_UNLOCKED;

static unsigned long lastReinitAttemptMs = 0;

// ===================================================================
// HELPER FUNCTIONS: I2C
// ===================================================================
bool i2cWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    lastReadError = true;
    return false;
  }
  size_t n = Wire.requestFrom((int)I2C_ADDR, (int)len, true);
  if (n != len) {
    lastReadError = true;
    return false;
  }
  for (size_t i = 0; i < len; i++)
    buf[i] = Wire.read();
  return true;
}

uint16_t read11(uint8_t reg) {
  uint8_t b[2];
  if (!i2cRead(reg, b, 2)) return 0;
  return (uint16_t)((b[0]) | (uint16_t(b[1]) << 8)) & 0x07FF;
}

uint32_t read20(uint8_t reg) {
  uint8_t b[3];
  if (!i2cRead(reg, b, 3)) return 0;
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

bool readPartID(uint8_t &pid) { return i2cRead(REG_PART_ID, &pid, 1); }
bool readStatus (uint8_t &st) { return i2cRead(REG_MAIN_STATUS, &st, 1); }

bool sensorInit() {
  if (!i2cWrite8(REG_MAIN_CTRL,
                 MAIN_CTRL_PROX_ENABLE | MAIN_CTRL_LS_ENABLE | MAIN_CTRL_RGB_MODE))
    return false;

  if (!i2cWrite8(REG_PROXIMITY_SENSOR_LED,
                 (uint8_t)g_ledFreq | (uint8_t)g_ledCurr))
    return false;

  if (!i2cWrite8(REG_PROXIMITY_SENSOR_PULSES, 32))
    return false;

  if (!i2cWrite8(REG_PROXIMITY_SENSOR_RATE,
                 (uint8_t)g_proxRes | (uint8_t)g_proxRate))
    return false;

  if (!i2cWrite8(REG_LIGHT_SENSOR_MEAS_RATE,
                 (uint8_t)g_colorRes | (uint8_t)g_colorRate))
    return false;

  if (!i2cWrite8(REG_LIGHT_SENSOR_GAIN, (uint8_t)g_gain))
    return false;

  return true;
}

void restartI2CBus() {
  Serial.println("[I2C] Restarting bus...");

  Wire.end();
  delay(4);

  // Re-init I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(4);

  Serial.println("[I2C] Bus restart complete.");
}


// ===================================================================
// CAN INIT
// ===================================================================
bool initCAN() {
  twai_general_config_t gcfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  tcfg = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gcfg, &tcfg, &fcfg) != ESP_OK) {
    Serial.println("[CAN] Install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[CAN] Start failed");
    return false;
  }
  Serial.println("[CAN] Started OK");
  return true;
}

// ===================================================================
// RTOS TASK: SENSOR READ (40 Hz = 25 ms) ALWAYS FASTEST POSSIBLE
// ===================================================================
void TaskSensorRead(void *pvParameters)
{
  Serial.println("[Sensor] Task start");

  const TickType_t period = pdMS_TO_TICKS(25); // ALWAYS 25 ms (fastest possible)
  TickType_t last = xTaskGetTickCount();

  // Track how many consecutive bad frames we’ve seen
  static uint8_t badFrameCount = 0;
  // Track whether we have ever seen at least one valid sample
  static bool haveGoodSample = false;

  for (;;) {
    vTaskDelayUntil(&last, period);

    bool doReboot = false;
    bool doConfig = false;

    portENTER_CRITICAL(&cfgMux);
    doReboot = rebootRequested;
    rebootRequested = false;
    doConfig = configDirty;
    configDirty = false;
    portEXIT_CRITICAL(&cfgMux);

    if (doReboot) {
      Serial.println("[Sensor] Reboot requested");
      vTaskDelay(pdMS_TO_TICKS(50));
      ESP.restart();
    }

    if (doConfig && sensorOnline) {
      if (!sensorInit())
        Serial.println("[Sensor] Failed re-config");
      else
        Serial.println("[Sensor] Re-config OK");
    }

    // ---- READ ----
    lastReadError = false;  // make sure your global is cleared before read
    uint8_t st = 0;
    readStatus(st);         // optional, used only to set lastReadError on I2C failure

    uint32_t red20   = read20(REG_DATA_RED);
    uint32_t green20 = read20(REG_DATA_GREEN);
    uint32_t blue20  = read20(REG_DATA_BLUE);
    uint32_t ir20    = read20(REG_DATA_INFRARED);
    uint16_t prox11  = read11(REG_PROXIMITY_DATA);

    bool allZero =
        (red20 == 0 && green20 == 0 && blue20 == 0 &&
         ir20  == 0 && prox11 == 0);

    bool allMax  =
        (red20   == 0x003FFFFF &&
         green20 == 0x003FFFFF &&
         blue20  == 0x003FFFFF &&
         ir20    == 0x003FFFFF &&
         prox11  == 0x07FF);

    bool good = !lastReadError && !allZero && !allMax;

    if (good) {
      // Good frame → reset bad-frame counter, mark online, update sample
      badFrameCount = 0;
      sensorOnline = true;
      haveGoodSample = true;

      ColorSample temp;
      temp.red   = (uint16_t)(red20   >> 4);  // shrink 20-bit to 16-bit
      temp.green = (uint16_t)(green20 >> 4);
      temp.blue  = (uint16_t)(blue20  >> 4);
      temp.ir    = (uint16_t)(ir20    >> 4);
      temp.prox  = prox11;
      temp.online = true;
      temp.haveSample = true;

      portENTER_CRITICAL(&sampleMux);
      gSample = temp;
      portEXIT_CRITICAL(&sampleMux);

      continue;
    }

    // --- BAD SAMPLE PATH (all-zero, all-max, or I2C error) ---

    badFrameCount++;

    // Don’t immediately drop offline on the first couple of bad frames:
    // this avoids thrashing if there’s a transient glitch or engine not yet ready.
    if (badFrameCount < 3 && !haveGoodSample) {
      // Early boot / not yet good once → just wait for sensor to settle
      continue;
    }

    // Mark sensor offline logically
    sensorOnline = false;

    // Mark sample offline (we keep last R/G/B/etc values, just flip the flag)
    portENTER_CRITICAL(&sampleMux);
    gSample.online = false;
    portEXIT_CRITICAL(&sampleMux);

    // Periodic re-init attempt (max every 500 ms)
    unsigned long now = millis();
    if (now - lastReinitAttemptMs >= 500) {
      lastReinitAttemptMs = now;

      Serial.println("[Sensor] Offline → attempting reinit");
      restartI2CBus();
      uint8_t pid = 0;

      if (sensorInit() && readPartID(pid)) {
        delay(5);   // allow sensor to start engines

        uint8_t st2 = 0;
        if (readStatus(st2) && (st2 & 0x06)) {   // bit1 ALS ready, bit2 RGB ready
          Serial.print("[Sensor] Reinit OK (engines ready), PID=");
          Serial.println(pid, HEX);
          sensorOnline = true;
          // We will set gSample.online=true on the next good frame
          badFrameCount = 0;
        } else {
          Serial.println("[Sensor] Reinit FAILED (engines not ready)");
          sensorOnline = false;
        }
      } else {
        Serial.println("[Sensor] Reinit FAILED");
        sensorOnline = false;
      }
    }
    // loop continues, next iteration will either get a good frame
    // (and flip online + update sample) or try reinit again later
  }
}

// ===================================================================
// RTOS TASK: CAN TX (ALWAYS 25 ms, fixed 40 Hz)
// ===================================================================
void TaskCANTx(void *pvParameters)
{
  Serial.println("[CAN-TX] Task start");

  const TickType_t period = pdMS_TO_TICKS(6);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);

    ColorSample s;
    uint8_t lf, lc, pr, prate, cr, crate, gn;
    bool onlineFlag;

    portENTER_CRITICAL(&sampleMux);
    s = gSample;
    portEXIT_CRITICAL(&sampleMux);

    portENTER_CRITICAL(&cfgMux);
    lf = g_ledFreq;
    lc = g_ledCurr;
    pr = g_proxRes;
    prate = g_proxRate;
    cr = g_colorRes;
    crate = g_colorRate;
    gn = g_gain;
    onlineFlag = s.online;
    portEXIT_CRITICAL(&cfgMux);

    if (s.haveSample) {
      // 0x184
      {
        twai_message_t msg = {};
        msg.extd = 1;
        msg.data_length_code = 8;
        msg.identifier = makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, API_COLOR_DATA1, g_deviceNumber);

        msg.data[0] = s.red   >> 8;
        msg.data[1] = s.red   & 0xFF;
        msg.data[2] = s.green >> 8;
        msg.data[3] = s.green & 0xFF;
        msg.data[4] = s.blue  >> 8;
        msg.data[5] = s.blue  & 0xFF;
        msg.data[6] = s.prox  >> 8;
        msg.data[7] = s.prox  & 0xFF;

        twai_transmit(&msg, 0);
      }

      // 0x185
      {
        twai_message_t msg = {};
        msg.extd = 1;
        msg.data_length_code = 8;
        msg.identifier = makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, API_COLOR_DATA2, g_deviceNumber);

        msg.data[0] = s.ir >> 8;
        msg.data[1] = s.ir & 0xFF;
        msg.data[2] = lf;
        msg.data[3] = lc;
        msg.data[4] = pr;
        msg.data[5] = prate;
        msg.data[6] = cr;
        msg.data[7] = crate;

        twai_transmit(&msg, 0);
      }

      // 0x186
      {
        twai_message_t msg = {};
        msg.extd = 1;
        msg.data_length_code = 8;
        msg.identifier = makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, API_COLOR_STATUS, g_deviceNumber);

        msg.data[0] = gn;
        msg.data[1] = onlineFlag ? 1 : 0;

        twai_transmit(&msg, 0);
      }
    }
  }
}

// ===================================================================
// CAN RX HANDLER
// ===================================================================
void handleConfigFrame(const twai_message_t &msg)
{
  if (msg.data_length_code < 8)
    return;

  bool reboot = msg.data[0] != 0;

  portENTER_CRITICAL(&cfgMux);
  rebootRequested = reboot;
  g_ledFreq  = msg.data[1];
  g_ledCurr  = msg.data[2];
  g_proxRes  = msg.data[3];
  g_proxRate = msg.data[4];
  g_colorRes = msg.data[5];
  g_colorRate= msg.data[6];
  g_gain     = msg.data[7];
  configDirty = true;
  portEXIT_CRITICAL(&cfgMux);

  Serial.println("[CAN-RX] Config frame received");
}



void TaskCANRx(void *pvParameters)
{
  Serial.println("[CAN-RX] Task start");

  for (;;) {
    twai_message_t msg;

    // Block until a CAN frame arrives
    if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {

      // Only accept extended (29-bit) frames
      if (!msg.extd)
        continue;

      uint32_t id = msg.identifier;

      // =============================
      // Inline FRC CAN ID decoding
      // =============================
      uint8_t  deviceID       = (id >> 24) & 0xFF;
      uint8_t  manufacturerID = (id >> 16) & 0xFF;
      uint16_t apiID          = (id >> 6)  & 0x3FF;
      uint8_t  deviceNumber   =  id        & 0x3F;

      // =============================
      // Addressing filters
      // =============================
      if (deviceID       != DEVICE_TYPE_ID)        continue;
      if (manufacturerID != MANUFACTURER_ID)  continue;
      if (deviceNumber   != g_deviceNumber)   continue;

      // =============================
      // API handling
      // =============================
      if (apiID == API_COLOR_CONFIG) {
        handleConfigFrame(msg);
      }

      // (Add more API handlers here)
    }
  }
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
          g_deviceNumber = (uint8_t)val;
          Serial.printf("[CANID] Running DEVICE_NUMBER set to %d\n", g_deviceNumber);
        } else {
          Serial.println("[CANID] Invalid value. Must be 0–63.");
        }
      }

      else if (line.equals("&CANID SAVE")) {
        EEPROMSaveCANID();
        Serial.println("[CANID] Saved to EEPROM. Rebooting...");
        delay(1000);
        ESP.restart();
      }

      else if (line.equals("&CANID GET")) {
        uint8_t eepromVal = EEPROM.read(0);
        Serial.printf("[CANID] Current=%d, EEPROM=%d, Default=%d\n",
                      g_deviceNumber, eepromVal, DEFAULT_DEVICE_NUMBER);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}



// ===================================================================
// SETUP & LOOP
// ===================================================================
void setup()
{
  Serial.begin(115200);
  delay(200);

  // --- NEW: EEPROM init and read CAN device number ---
  EEPROM.begin(EEPROM_ADDRESS);
  EEPROMReadCANID();
  Serial.printf("[BOOT] DEVICE_NUMBER=%d\n", g_deviceNumber);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t pid = 0;
  if (!sensorInit() || !readPartID(pid)) {
    Serial.println("Sensor init FAILED");
    sensorOnline = false;
  }
  else {
    Serial.print("Sensor init OK, PID=");
    Serial.println(pid, HEX);
    sensorOnline = true;
  }

  initCAN();

  xTaskCreatePinnedToCore(TaskCANIDHelper, "TaskCANIDHelper", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(TaskSensorRead, "SensorRead", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANTx,      "CANTx",      4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx,      "CANRx",      4096, NULL, 2, NULL, 0);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}