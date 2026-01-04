/*
  REV Color Sensor V3 (APDS-9151) + AndyMark Color Sensor (TMD37253M) on ESP32
  ----------------------------------------------------------------------------
  I2C:
    SDA = IO21
    SCL = IO22
    REV address = 0x52
    AndyMark address = 0x39

  CAN (TWAI):
    CAN_TX = GPIO 4
    CAN_RX = GPIO 5
    Bitrate = 1 Mbps, Extended 29-bit IDs

  CAN API usage (FRC-style 29-bit ID):
    CAN_ID = (DEVICE_ID << 24) | (MANUFACTURER_ID << 16) | (API_ID << 6) | (DEVICE_NUMBER & 0x3F)

  REV (APDS-9151):
    0x184 ESP -> RIO: Red/Green/Blue/Prox (16-bit each)
    0x185 ESP -> RIO: IR + config enums
    0x186 ESP -> RIO: Gain + online flag
    0x187 RIO -> ESP: config + reboot

  AndyMark (TMD37253M):
    0x194 ESP -> RIO: Red/Green/Blue/Prox (16-bit each)
    0x195 ESP -> RIO: Clear + sensorGood
    0x197 RIO -> ESP: reboot
*/
#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include <EEPROM.h>

// ========= I2C & PIN =========
static constexpr uint8_t REV_I2C_ADDR = 0x52;
static constexpr uint8_t AM_I2C_ADDR  = 0x39;
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;

// ========= CAN PINS =========
static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

// ========= CAN/FRC IDs =========
#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08

#define API_REV_DATA1    0x184
#define API_REV_DATA2    0x185
#define API_REV_STATUS   0x186
#define API_REV_CONFIG   0x187

#define API_AM_DATA      0x194
#define API_AM_STATUS    0x195
#define API_AM_REBOOT    0x197

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

// =================== CAN Device Number via EEPROM ===================
#define EEPROM_ADDRESS 64
#define DEFAULT_DEVICE_NUMBER 33
volatile uint8_t g_deviceNumber = DEFAULT_DEVICE_NUMBER;

void EEPROMReadCANID() {
  uint8_t saved = EEPROM.read(0);
  g_deviceNumber = (saved <= 63) ? saved : DEFAULT_DEVICE_NUMBER;
}

void EEPROMSaveCANID() {
  EEPROM.write(0, g_deviceNumber);
  EEPROM.commit();
}

// ===================================================================
// I2C BUS MUTEX
// ===================================================================
static SemaphoreHandle_t gI2cMutex;

static bool i2cTake(TickType_t timeoutTicks = pdMS_TO_TICKS(20)) {
  return xSemaphoreTake(gI2cMutex, timeoutTicks) == pdTRUE;
}

static void i2cGive() {
  xSemaphoreGive(gI2cMutex);
}

static void restartI2CBusLocked() {
  Wire.end();
  delay(4);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(4);
}

// ===================================================================
// REV COLOR SENSOR ENUMS + REGISTERS (APDS-9151)
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

static constexpr uint8_t MAIN_CTRL_PROX_ENABLE = 0x01;
static constexpr uint8_t MAIN_CTRL_LS_ENABLE   = 0x02;
static constexpr uint8_t MAIN_CTRL_RGB_MODE    = 0x04;

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

struct RevSample {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t ir;
  uint16_t prox;
  bool     online;
  bool     haveSample;
};

static RevSample gRevSample = {0,0,0,0,0,false,false};

static bool revSensorOnline = false;
static bool revLastReadError = false;
static bool rebootRequested = false;
static bool configDirty = false;

static uint8_t g_ledFreq  = LED_FREQ_100k;
static uint8_t g_ledCurr  = LED_CURR_50mA;
static uint8_t g_proxRes  = PROX_RES_11b;
static uint8_t g_proxRate = PROX_RATE_100ms;
static uint8_t g_colorRes = COLOR_RES_20b;
static uint8_t g_colorRate= COLOR_RATE_25ms;
static uint8_t g_gain     = GAIN_3X;

static portMUX_TYPE revSampleMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE revCfgMux    = portMUX_INITIALIZER_UNLOCKED;

static unsigned long lastRevReinitAttemptMs = 0;

// ===================================================================
// AndyMark TMD37253M REGISTERS
// ===================================================================
#define AM_ENABLE_REG  0x80
#define AM_ATIME_REG   0x81
#define AM_WTIME_REG   0x83
#define AM_CONTROL_REG 0x8F
#define AM_STATUS_REG  0x93
#define AM_CDATA_REG   0x94
#define AM_RDATA_REG   0x96
#define AM_GDATA_REG   0x98
#define AM_BDATA_REG   0x9A
#define AM_PDATA_REG   0x9C

struct AmSample {
  uint16_t clear;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t prox;
  bool sensorGood;
  bool haveSample;
};

static AmSample gAmSample = {0,0,0,0,0,false,false};
static bool amI2cOffline = false;
static uint32_t lastAmI2cRecovery = 0;
static portMUX_TYPE amSampleMux = portMUX_INITIALIZER_UNLOCKED;

// ===================================================================
// REV I2C HELPERS
// ===================================================================
bool revI2cWrite8(uint8_t reg, uint8_t val) {
  if (!i2cTake()) return false;
  Wire.beginTransmission(REV_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  bool ok = (Wire.endTransmission(true) == 0);
  i2cGive();
  return ok;
}

bool revI2cRead(uint8_t reg, uint8_t *buf, size_t len) {
  if (!i2cTake()) return false;
  Wire.beginTransmission(REV_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    revLastReadError = true;
    i2cGive();
    return false;
  }
  size_t n = Wire.requestFrom((int)REV_I2C_ADDR, (int)len, true);
  if (n != len) {
    revLastReadError = true;
    i2cGive();
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  i2cGive();
  return true;
}

uint16_t revRead11(uint8_t reg) {
  uint8_t b[2];
  if (!revI2cRead(reg, b, 2)) return 0;
  return (uint16_t)((b[0]) | (uint16_t(b[1]) << 8)) & 0x07FF;
}

uint32_t revRead20(uint8_t reg) {
  uint8_t b[3];
  if (!revI2cRead(reg, b, 3)) return 0;
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

bool revReadPartID(uint8_t &pid) { return revI2cRead(REG_PART_ID, &pid, 1); }
bool revReadStatus (uint8_t &st) { return revI2cRead(REG_MAIN_STATUS, &st, 1); }

bool revSensorInit() {
  if (!revI2cWrite8(REG_MAIN_CTRL,
                    MAIN_CTRL_PROX_ENABLE | MAIN_CTRL_LS_ENABLE | MAIN_CTRL_RGB_MODE))
    return false;

  if (!revI2cWrite8(REG_PROXIMITY_SENSOR_LED,
                    (uint8_t)g_ledFreq | (uint8_t)g_ledCurr))
    return false;

  if (!revI2cWrite8(REG_PROXIMITY_SENSOR_PULSES, 32))
    return false;

  if (!revI2cWrite8(REG_PROXIMITY_SENSOR_RATE,
                    (uint8_t)g_proxRes | (uint8_t)g_proxRate))
    return false;

  if (!revI2cWrite8(REG_LIGHT_SENSOR_MEAS_RATE,
                    (uint8_t)g_colorRes | (uint8_t)g_colorRate))
    return false;

  if (!revI2cWrite8(REG_LIGHT_SENSOR_GAIN, (uint8_t)g_gain))
    return false;

  return true;
}

void revRestartI2CBus() {
  Serial.println("[I2C] Restarting bus (REV)...");
  if (!i2cTake(pdMS_TO_TICKS(100))) {
    Serial.println("[I2C] Mutex timeout");
    return;
  }
  restartI2CBusLocked();
  i2cGive();
  Serial.println("[I2C] Bus restart complete.");
}

// ===================================================================
// ANDYMARK I2C HELPERS
// ===================================================================
bool amSafeWriteReg(uint8_t reg, uint8_t value)
{
  if (!i2cTake()) return false;
  Wire.beginTransmission(AM_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  bool ok = (Wire.endTransmission() == 0);
  i2cGive();
  if (!ok) amI2cOffline = true;
  return ok;
}

uint16_t amSafeRead16(uint8_t reg)
{
  if (!i2cTake()) return 0;
  Wire.beginTransmission(AM_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    amI2cOffline = true;
    i2cGive();
    return 0;
  }
  Wire.requestFrom(AM_I2C_ADDR, 2);
  if (Wire.available() != 2) {
    amI2cOffline = true;
    i2cGive();
    return 0;
  }
  uint16_t v = Wire.read();
  v |= (Wire.read() << 8);
  i2cGive();
  return v;
}

bool amInitializeSensor()
{
  if (!amSafeWriteReg(AM_ENABLE_REG, 0x01)) return false;
  delay(10);
  if (!amSafeWriteReg(AM_ENABLE_REG, 0x07)) return false;
  if (!amSafeWriteReg(AM_ATIME_REG,  0xFF)) return false;
  if (!amSafeWriteReg(AM_WTIME_REG,  0x00)) return false;
  if (!amSafeWriteReg(AM_CONTROL_REG,0x02)) return false;
  if (!amSafeWriteReg(0x8E, 0x11)) return false;
  if (!amSafeWriteReg(AM_CONTROL_REG,0x0F)) return false;
  delay(200);
  return true;
}

void amTryI2cRecovery()
{
  uint32_t now = millis();
  if (amI2cOffline && (now - lastAmI2cRecovery > 3000)) {
    lastAmI2cRecovery = now;
    if (!i2cTake(pdMS_TO_TICKS(100))) {
      return;
    }
    restartI2CBusLocked();
    bool ok = amInitializeSensor();
    i2cGive();
    if (ok) {
      amI2cOffline = false;
    }
  }
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
// RTOS TASK: REV SENSOR READ (40 Hz = 25 ms)
// ===================================================================
void TaskRevSensorRead(void *pvParameters)
{
  Serial.println("[REV] Task start");

  const TickType_t period = pdMS_TO_TICKS(25);
  TickType_t last = xTaskGetTickCount();

  static uint8_t badFrameCount = 0;
  static bool haveGoodSample = false;

  for (;;) {
    vTaskDelayUntil(&last, period);

    bool doReboot = false;
    bool doConfig = false;

    portENTER_CRITICAL(&revCfgMux);
    doReboot = rebootRequested;
    rebootRequested = false;
    doConfig = configDirty;
    configDirty = false;
    portEXIT_CRITICAL(&revCfgMux);

    if (doReboot) {
      Serial.println("[REV] Reboot requested");
      vTaskDelay(pdMS_TO_TICKS(50));
      ESP.restart();
    }

    if (doConfig && revSensorOnline) {
      if (!revSensorInit())
        Serial.println("[REV] Failed re-config");
      else
        Serial.println("[REV] Re-config OK");
    }

    // ---- READ ----
    revLastReadError = false;
    uint8_t st = 0;
    revReadStatus(st);

    uint32_t red20   = revRead20(REG_DATA_RED);
    uint32_t green20 = revRead20(REG_DATA_GREEN);
    uint32_t blue20  = revRead20(REG_DATA_BLUE);
    uint32_t ir20    = revRead20(REG_DATA_INFRARED);
    uint16_t prox11  = revRead11(REG_PROXIMITY_DATA);

    bool allZero =
        (red20 == 0 && green20 == 0 && blue20 == 0 &&
         ir20  == 0 && prox11 == 0);

    bool allMax  =
        (red20   == 0x003FFFFF &&
         green20 == 0x003FFFFF &&
         blue20  == 0x003FFFFF &&
         ir20    == 0x003FFFFF &&
         prox11  == 0x07FF);

    bool good = !revLastReadError && !allZero && !allMax;

    if (good) {
      badFrameCount = 0;
      revSensorOnline = true;
      haveGoodSample = true;

      RevSample temp;
      temp.red   = (uint16_t)(red20   >> 4);
      temp.green = (uint16_t)(green20 >> 4);
      temp.blue  = (uint16_t)(blue20  >> 4);
      temp.ir    = (uint16_t)(ir20    >> 4);
      temp.prox  = prox11;
      temp.online = true;
      temp.haveSample = true;

      portENTER_CRITICAL(&revSampleMux);
      gRevSample = temp;
      portEXIT_CRITICAL(&revSampleMux);

      continue;
    }

    badFrameCount++;

    if (badFrameCount < 3 && !haveGoodSample) {
      continue;
    }

    revSensorOnline = false;

    portENTER_CRITICAL(&revSampleMux);
    gRevSample.online = false;
    portEXIT_CRITICAL(&revSampleMux);

    unsigned long now = millis();
    if (now - lastRevReinitAttemptMs >= 500) {
      lastRevReinitAttemptMs = now;

      Serial.println("[REV] Offline - attempting reinit");
      revRestartI2CBus();
      uint8_t pid = 0;

      if (revSensorInit() && revReadPartID(pid)) {
        delay(5);
        uint8_t st2 = 0;
        if (revReadStatus(st2) && (st2 & 0x06)) {
          Serial.print("[REV] Reinit OK, PID=");
          Serial.println(pid, HEX);
          revSensorOnline = true;
          badFrameCount = 0;
        } else {
          Serial.println("[REV] Reinit FAILED (engines not ready)");
          revSensorOnline = false;
        }
      } else {
        Serial.println("[REV] Reinit FAILED");
        revSensorOnline = false;
      }
    }
  }
}

// ===================================================================
// RTOS TASK: ANDYMARK SENSOR READ (200 Hz)
// ===================================================================
void TaskAmSensorRead(void *param)
{
  const TickType_t rate = pdMS_TO_TICKS(5);
  while (1) {
    amTryI2cRecovery();

    AmSample temp;
    temp.clear     = amSafeRead16(AM_CDATA_REG);
    temp.red       = amSafeRead16(AM_RDATA_REG);
    temp.green     = amSafeRead16(AM_GDATA_REG);
    temp.blue      = amSafeRead16(AM_BDATA_REG);
    temp.prox      = amSafeRead16(AM_PDATA_REG);
    temp.sensorGood = (temp.clear | temp.red | temp.green | temp.blue | temp.prox) != 0;
    temp.haveSample = true;

    portENTER_CRITICAL(&amSampleMux);
    gAmSample = temp;
    portEXIT_CRITICAL(&amSampleMux);

    vTaskDelay(rate);
  }
}

// ===================================================================
// RTOS TASK: SERIAL PRINT (200 ms)
// ===================================================================
void TaskSerialPrint(void *param)
{
  const TickType_t rate = pdMS_TO_TICKS(200);

  while (1) {
    AmSample s;
    portENTER_CRITICAL(&amSampleMux);
    s = gAmSample;
    portEXIT_CRITICAL(&amSampleMux);

    Serial.printf("%u %u %u %u %u %d\n",
                  s.clear, s.red, s.green, s.blue, s.prox,
                  s.sensorGood ? 1 : 0);

    vTaskDelay(rate);
  }
}

// ===================================================================
// RTOS TASK: REV CAN TX (40 Hz)
// ===================================================================
void TaskRevCANTx(void *pvParameters)
{
  Serial.println("[CAN-TX] REV task start");

  const TickType_t period = pdMS_TO_TICKS(6);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);

    RevSample s;
    uint8_t lf, lc, pr, prate, cr, crate, gn;
    bool onlineFlag;

    portENTER_CRITICAL(&revSampleMux);
    s = gRevSample;
    portEXIT_CRITICAL(&revSampleMux);

    portENTER_CRITICAL(&revCfgMux);
    lf = g_ledFreq;
    lc = g_ledCurr;
    pr = g_proxRes;
    prate = g_proxRate;
    cr = g_colorRes;
    crate = g_colorRate;
    gn = g_gain;
    onlineFlag = s.online;
    portEXIT_CRITICAL(&revCfgMux);

    if (s.haveSample) {
      twai_message_t msg1 = {};
      msg1.extd = 1;
      msg1.data_length_code = 8;
      msg1.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_REV_DATA1, g_deviceNumber);

      msg1.data[0] = s.red   >> 8;
      msg1.data[1] = s.red   & 0xFF;
      msg1.data[2] = s.green >> 8;
      msg1.data[3] = s.green & 0xFF;
      msg1.data[4] = s.blue  >> 8;
      msg1.data[5] = s.blue  & 0xFF;
      msg1.data[6] = s.prox  >> 8;
      msg1.data[7] = s.prox  & 0xFF;

      twai_transmit(&msg1, 0);

      twai_message_t msg2 = {};
      msg2.extd = 1;
      msg2.data_length_code = 8;
      msg2.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_REV_DATA2, g_deviceNumber);

      msg2.data[0] = s.ir >> 8;
      msg2.data[1] = s.ir & 0xFF;
      msg2.data[2] = lf;
      msg2.data[3] = lc;
      msg2.data[4] = pr;
      msg2.data[5] = prate;
      msg2.data[6] = cr;
      msg2.data[7] = crate;

      twai_transmit(&msg2, 0);

      twai_message_t msg3 = {};
      msg3.extd = 1;
      msg3.data_length_code = 8;
      msg3.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_REV_STATUS, g_deviceNumber);

      msg3.data[0] = gn;
      msg3.data[1] = onlineFlag ? 1 : 0;

      twai_transmit(&msg3, 0);
    }
  }
}

// ===================================================================
// RTOS TASK: ANDYMARK CAN TX (200 Hz)
// ===================================================================
void TaskAmCANTx(void *param)
{
  const TickType_t rate = pdMS_TO_TICKS(5);

  while (1) {
    AmSample s;
    portENTER_CRITICAL(&amSampleMux);
    s = gAmSample;
    portEXIT_CRITICAL(&amSampleMux);

    twai_message_t m194 = {};
    m194.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_AM_DATA, g_deviceNumber);
    m194.extd = 1;
    m194.data_length_code = 8;

    m194.data[0] = (s.red >> 8) & 0xFF;
    m194.data[1] = s.red & 0xFF;
    m194.data[2] = (s.green >> 8) & 0xFF;
    m194.data[3] = s.green & 0xFF;
    m194.data[4] = (s.blue >> 8) & 0xFF;
    m194.data[5] = s.blue & 0xFF;
    m194.data[6] = (s.prox >> 8) & 0xFF;
    m194.data[7] = s.prox & 0xFF;

    twai_transmit(&m194, pdMS_TO_TICKS(2));

    twai_message_t m195 = {};
    m195.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_AM_STATUS, g_deviceNumber);
    m195.extd = 1;
    m195.data_length_code = 3;

    m195.data[0] = (s.clear >> 8) & 0xFF;
    m195.data[1] = s.clear & 0xFF;
    m195.data[2] = s.sensorGood ? 1 : 0;

    twai_transmit(&m195, pdMS_TO_TICKS(2));

    vTaskDelay(rate);
  }
}

// ===================================================================
// CAN RX HANDLER
// ===================================================================
void handleRevConfigFrame(const twai_message_t &msg)
{
  if (msg.data_length_code < 8)
    return;

  bool reboot = msg.data[0] != 0;

  portENTER_CRITICAL(&revCfgMux);
  rebootRequested = reboot;
  g_ledFreq  = msg.data[1];
  g_ledCurr  = msg.data[2];
  g_proxRes  = msg.data[3];
  g_proxRate = msg.data[4];
  g_colorRes = msg.data[5];
  g_colorRate= msg.data[6];
  g_gain     = msg.data[7];
  configDirty = true;
  portEXIT_CRITICAL(&revCfgMux);

  Serial.println("[CAN-RX] REV config frame received");
}

void TaskCANRx(void *pvParameters)
{
  Serial.println("[CAN-RX] Task start");

  for (;;) {
    twai_message_t msg;

    if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
      if (!msg.extd)
        continue;

      uint32_t id = msg.identifier;
      uint8_t  deviceID       = (id >> 24) & 0xFF;
      uint8_t  manufacturerID = (id >> 16) & 0xFF;
      uint16_t apiID          = (id >> 6)  & 0x3FF;
      uint8_t  deviceNumber   =  id        & 0x3F;

      if (deviceID       != DEVICE_ID)        continue;
      if (manufacturerID != MANUFACTURER_ID)  continue;
      if (deviceNumber   != g_deviceNumber)   continue;

      if (apiID == API_REV_CONFIG) {
        handleRevConfigFrame(msg);
      } else if (apiID == API_AM_REBOOT) {
        if (msg.data_length_code >= 1 && msg.data[0] != 0) {
          Serial.println("[CAN] Reboot requested by RIO");
          delay(20);
          ESP.restart();
        }
      }
    }
  }
}

// ===================================================================
// CANID HELPER
// ===================================================================
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
          Serial.println("[CANID] Invalid value. Must be 0-63.");
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

  gI2cMutex = xSemaphoreCreateMutex();

  EEPROM.begin(EEPROM_ADDRESS);
  EEPROMReadCANID();
  Serial.printf("[BOOT] DEVICE_NUMBER=%d\n", g_deviceNumber);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t pid = 0;
  if (!revSensorInit() || !revReadPartID(pid)) {
    Serial.println("[REV] Sensor init FAILED");
    revSensorOnline = false;
  } else {
    Serial.print("[REV] Sensor init OK, PID=");
    Serial.println(pid, HEX);
    revSensorOnline = true;
  }

  if (!amInitializeSensor()) {
    Serial.println("[AM] Boot in OFFLINE mode.");
    amI2cOffline = true;
  }

  initCAN();

  xTaskCreatePinnedToCore(TaskCANIDHelper, "TaskCANIDHelper", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(TaskRevSensorRead, "RevSensorRead", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskAmSensorRead,  "AmSensorRead",  4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskSerialPrint,   "AMSerial",      4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskRevCANTx,      "RevCANTx",      4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskAmCANTx,       "AmCANTx",       4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx,         "CANRx",         4096, NULL, 2, NULL, 0);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
