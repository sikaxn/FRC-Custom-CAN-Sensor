#include <Wire.h>
#include <Arduino.h>
#include "driver/twai.h"

// --------------------------------------------------
// I2C Defines / Pins
// --------------------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22
#define TMD37253M_I2C_ADDR 0x39

// Registers
#define ENABLE_REG 0x80
#define ATIME_REG  0x81
#define WTIME_REG  0x83
#define CONTROL_REG 0x8F
#define STATUS_REG  0x93
#define CDATA_REG   0x94
#define RDATA_REG   0x96
#define GDATA_REG   0x98
#define BDATA_REG   0x9A
#define PDATA_REG   0x9C

// --------------------------------------------------
// FRC-style CAN Constants
// --------------------------------------------------
#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define DEVICE_NUMBER    33

#define API_194   0x194    // ESP → RIO color/prox
#define API_195   0x195    // ESP → RIO clear/sensorGood
#define API_197   0x197    // RIO → ESP reboot

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID,
                      uint16_t apiID, uint8_t deviceNumber)
{
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

// --------------------------------------------------
// Global Sensor Data
// --------------------------------------------------
uint16_t clear     = 0;
uint16_t red       = 0;
uint16_t green     = 0;
uint16_t blue      = 0;
uint16_t proximity = 0;
bool sensorGood    = false;
static bool i2cOffline = false;
static uint32_t lastI2cRecovery = 0;

// --------------------------------------------------
// Safe I2C Access
// --------------------------------------------------
bool safeWriteReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TMD37253M_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) {
    i2cOffline = true;
    return false;
  }
  return true;
}

uint16_t safeRead16(uint8_t reg)
{
  Wire.beginTransmission(TMD37253M_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    i2cOffline = true;
    return 0;
  }
  Wire.requestFrom(TMD37253M_I2C_ADDR, 2);
  if (Wire.available() != 2) {
    i2cOffline = true;
    return 0;
  }
  uint16_t v = Wire.read();
  v |= (Wire.read() << 8);
  return v;
}

bool initializeSensor()
{
  if (!safeWriteReg(ENABLE_REG, 0x01)) return false;
  delay(10);
  if (!safeWriteReg(ENABLE_REG, 0x07)) return false;
  if (!safeWriteReg(ATIME_REG,  0xDB)) return false;
  if (!safeWriteReg(WTIME_REG,  0xFF)) return false;
  if (!safeWriteReg(CONTROL_REG,0x02)) return false;
  if (!safeWriteReg(0x8E, 0x11)) return false;
  if (!safeWriteReg(CONTROL_REG,0x0F)) return false;
  delay(200);
  return true;
}

void tryI2cRecovery()
{
  uint32_t now = millis();
  if (i2cOffline && (now - lastI2cRecovery > 3000)) {
    lastI2cRecovery = now;
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    if (initializeSensor()) {
      i2cOffline = false;
    }
  }
}

// --------------------------------------------------
// RTOS Task: Sensor Read (50 Hz)
// --------------------------------------------------
void TaskSensorRead(void *param)
{
  const TickType_t rate = pdMS_TO_TICKS(5); 
  while (1) {
    tryI2cRecovery();

    clear     = safeRead16(CDATA_REG);
    red       = safeRead16(RDATA_REG);
    green     = safeRead16(GDATA_REG);
    blue      = safeRead16(BDATA_REG);
    proximity = safeRead16(PDATA_REG);

    sensorGood = (clear | red | green | blue | proximity) != 0;

    vTaskDelay(rate);
  }
}

// --------------------------------------------------
// RTOS Task: Serial Print (200ms)
// --------------------------------------------------
void TaskSerialPrint(void *param)
{
  const TickType_t rate = pdMS_TO_TICKS(200);

  while (1) {
    if (sensorGood)
      Serial.printf("[GOOD] C=%u R=%u G=%u B=%u P=%u\n",
                    clear, red, green, blue, proximity);
    else
      Serial.println("[BAD] No sensor data");

    vTaskDelay(rate);
  }
}

// --------------------------------------------------
// RTOS Task: CAN TX (20 Hz)
// --------------------------------------------------
void TaskCANTx(void *param)
{
  const TickType_t rate = pdMS_TO_TICKS(5); 

  while (1) {
    // ---- Message 0x194 ----
    twai_message_t m194 = {};
    m194.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_194, DEVICE_NUMBER);
    m194.extd = 1;
    m194.data_length_code = 8;

    m194.data[0] = (red >> 8) & 0xFF;
    m194.data[1] = red & 0xFF;
    m194.data[2] = (green >> 8) & 0xFF;
    m194.data[3] = green & 0xFF;
    m194.data[4] = (blue >> 8) & 0xFF;
    m194.data[5] = blue & 0xFF;
    m194.data[6] = (proximity >> 8) & 0xFF;
    m194.data[7] = proximity & 0xFF;

    twai_transmit(&m194, pdMS_TO_TICKS(2));

    // ---- Message 0x195 ----
    twai_message_t m195 = {};
    m195.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_195, DEVICE_NUMBER);
    m195.extd = 1;
    m195.data_length_code = 3;

    m195.data[0] = (clear >> 8) & 0xFF;
    m195.data[1] = clear & 0xFF;
    m195.data[2] = sensorGood ? 1 : 0;

    twai_transmit(&m195, pdMS_TO_TICKS(2));

    vTaskDelay(rate);
  }
}

// --------------------------------------------------
// RTOS Task: CAN RX (fast loop)
// --------------------------------------------------
void TaskCANRx(void *param)
{
  twai_message_t rx;

  while (1) {
    if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {

      uint16_t api = (rx.identifier >> 6) & 0x3FF;

      if (api == API_197) {
        if (rx.data_length_code >= 1 && rx.data[0] != 0) {
          Serial.println("[CAN] Reboot requested by RIO");
          delay(20);
          ESP.restart();
        }
      }
    }
  }
}

// --------------------------------------------------
// Setup CAN
// --------------------------------------------------
void setupCAN()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!initializeSensor()) {
    Serial.println("[I2C] Boot in OFFLINE mode.");
    i2cOffline = true;
  }

  setupCAN();

  xTaskCreatePinnedToCore(TaskSensorRead,  "T_Sensor", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskSerialPrint, "T_Print",  4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANTx,       "T_CANTx",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx,       "T_CANRx",  4096, NULL, 2, NULL, 0);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
