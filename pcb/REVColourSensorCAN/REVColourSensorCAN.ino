/*
  ESP32 + REV Color Sensor V3 (APDS-9151) + CAN (TWAI) + Serial CAN-ID config
  ---------------------------------------------------------------------------
  I2C: SDA=IO21, SCL=IO22
  CAN: TX=IO4,  RX=IO5  (external transceiver required, e.g., TJA1051/SN65HVD230)
  Bitrate: 1 Mbps
  Update rate: ~40 Hz (25 ms)
  CAN Standard 11-bit ID = 0x184 + deviceNumber (0..63)

  Payload (8 bytes):
    [0-1] R16  (hi, lo)   from 20-bit >> 4
    [2-3] G16  (hi, lo)   from 20-bit >> 4
    [4-5] B16  (hi, lo)   from 20-bit >> 4
    [6]   Prox (0..255)   clamp of 11-bit
    [7]   IR high byte    of (IR20>>4)

  Serial @115200:
    &CANID GET
    &CANID SET xx
    &CANID SAVE
    &CANID HELP
*/

#include <Wire.h>
#include <EEPROM.h>
#include "driver/twai.h"

// ========= Pins =========
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
static constexpr int CAN_TX_PIN = 4;  // IO4 = CAN_TX
static constexpr int CAN_RX_PIN = 5;  // IO5 = CAN_RX

// ========= CAN settings =========
static constexpr uint16_t CAN_BASE_STD_ID = 0x184;   // base API (11-bit standard)
static constexpr uint32_t CAN_BITRATE     = 1000000; // 1 Mbps

// ========= Device number storage (EEPROM) =========
static constexpr uint8_t  DEFAULT_DEVICE_NUMBER = 3; // 0..63
static volatile uint8_t   g_deviceNumber = DEFAULT_DEVICE_NUMBER;

static constexpr int      EEPROM_BYTES   = 64;
static constexpr uint8_t  CANID_MAGIC    = 0xC1;
static constexpr int      EE_ADDR_MAGIC  = 0;  // 1 byte
static constexpr int      EE_ADDR_CANID  = 1;  // 1 byte (0..63)

// ========= Task periods =========
static constexpr uint32_t COLOR_TASK_PERIOD_MS = 25; // ~40 Hz
static constexpr uint32_t CANTX_TASK_PERIOD_MS = 25; // ~40 Hz

// ========= REV Color Sensor V3 (APDS-9151 @ 0x52) =========
static constexpr uint8_t I2C_ADDR = 0x52;

enum Register : uint8_t {
  REG_MAIN_CTRL                   = 0x00,
  REG_PROXIMITY_SENSOR_LED       = 0x01,
  REG_PROXIMITY_SENSOR_PULSES    = 0x02,
  REG_PROXIMITY_SENSOR_RATE      = 0x03,
  REG_LIGHT_SENSOR_MEAS_RATE     = 0x04,
  REG_LIGHT_SENSOR_GAIN          = 0x05,
  REG_PART_ID                    = 0x06,
  REG_MAIN_STATUS                = 0x07,
  REG_PROXIMITY_DATA             = 0x08, // 11-bit (2 bytes, LSB first)
  REG_DATA_INFRARED              = 0x0A, // 20-bit (3 bytes, LSB first)
  REG_DATA_GREEN                 = 0x0D, // 20-bit
  REG_DATA_BLUE                  = 0x10, // 20-bit
  REG_DATA_RED                   = 0x13  // 20-bit
};

// MAIN_CTRL bits
static constexpr uint8_t MAIN_CTRL_PROX_ENABLE = 0x01;
static constexpr uint8_t MAIN_CTRL_LS_ENABLE   = 0x02;
static constexpr uint8_t MAIN_CTRL_RGB_MODE    = 0x04;

// Resolutions
enum ProxResolution  : uint8_t { PROX_RES_11b = 0x18 };
enum ColorResolution : uint8_t { COLOR_RES_20b = 0x00 };

// Full rate enums (include 25 ms options)
enum ProxRate : uint8_t {
  PROX_RATE_6ms   = 1,
  PROX_RATE_12ms  = 2,
  PROX_RATE_25ms  = 3,
  PROX_RATE_50ms  = 4,
  PROX_RATE_100ms = 5,
  PROX_RATE_200ms = 6,
  PROX_RATE_400ms = 7
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

// LED / gain settings
enum GainFactor   : uint8_t { GAIN_3X = 1 };
enum LEDCurrent   : uint8_t { LED_CURR_50mA = 4 };
enum LEDPulseFreq : uint8_t { LED_FREQ_100k = 0x38 };

// ========= I2C helpers =========
bool i2cWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  size_t n = Wire.requestFrom((int)I2C_ADDR, (int)len, (int)true);
  if (n != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
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
  return (uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16)) & 0x003FFFFF;
}

bool sensorInit() {
  // Enable proximity + light sensors in RGB mode
  if (!i2cWrite8(REG_MAIN_CTRL, MAIN_CTRL_PROX_ENABLE | MAIN_CTRL_LS_ENABLE | MAIN_CTRL_RGB_MODE)) return false;

  // Proximity LED: 100 kHz, 50 mA
  if (!i2cWrite8(REG_PROXIMITY_SENSOR_LED, (uint8_t)LED_FREQ_100k | (uint8_t)LED_CURR_50mA)) return false;

  // Moderate pulses at higher loop rate
  if (!i2cWrite8(REG_PROXIMITY_SENSOR_PULSES, 16)) return false;

  // Proximity: 11-bit @ 25 ms (≈40 Hz cadence)
  if (!i2cWrite8(REG_PROXIMITY_SENSOR_RATE, (uint8_t)PROX_RES_11b | (uint8_t)PROX_RATE_25ms)) return false;

  // Color: 20-bit @ 25 ms (max speed for RGB/IR)
  if (!i2cWrite8(REG_LIGHT_SENSOR_MEAS_RATE, (uint8_t)COLOR_RES_20b | (uint8_t)COLOR_RATE_25ms)) return false;

  // Gain: 3x (tune as needed)
  if (!i2cWrite8(REG_LIGHT_SENSOR_GAIN, (uint8_t)GAIN_3X)) return false;

  return true;
}

// ========= Shared color packet =========
typedef struct {
  uint16_t r16;
  uint16_t g16;
  uint16_t b16;
  uint8_t  prox8;
  uint8_t  ir_hi8;
} ColorPacket;

static ColorPacket g_pkt = {0};
static SemaphoreHandle_t g_pktMutex;

// ========= TWAI (CAN) =========
bool canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 10;
  g_config.rx_queue_len = 10;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

// NOTE: Avoid Arduino's auto-prototype issue by NOT using ColorPacket in the signature
static inline void packBytes(uint16_t r16, uint16_t g16, uint16_t b16,
                             uint8_t prox8, uint8_t ir_hi8, uint8_t out[8]) {
  out[0] = (uint8_t)(r16 >> 8);
  out[1] = (uint8_t)(r16 & 0xFF);
  out[2] = (uint8_t)(g16 >> 8);
  out[3] = (uint8_t)(g16 & 0xFF);
  out[4] = (uint8_t)(b16 >> 8);
  out[5] = (uint8_t)(b16 & 0xFF);
  out[6] = prox8;
  out[7] = ir_hi8;
}

// ========= EEPROM helpers =========
void EEPROMInit() {
  EEPROM.begin(EEPROM_BYTES);
}

void EEPROMSaveCANID() {
  EEPROM.write(EE_ADDR_MAGIC, CANID_MAGIC);
  EEPROM.write(EE_ADDR_CANID, (uint8_t)(g_deviceNumber & 0x3F));
  EEPROM.commit();
}

uint8_t EEPROMLoadCANID() {
  uint8_t magic = EEPROM.read(EE_ADDR_MAGIC);
  uint8_t val   = EEPROM.read(EE_ADDR_CANID);
  if (magic == CANID_MAGIC && val <= 63) {
    return val;
  }
  return DEFAULT_DEVICE_NUMBER;
}

// =================== Tasks ===================
void ColorSensorTask(void *arg) {
  (void)arg;
  for (;;) {
    uint32_t r20 = read20(REG_DATA_RED);
    uint32_t g20 = read20(REG_DATA_GREEN);
    uint32_t b20 = read20(REG_DATA_BLUE);
    uint32_t ir20 = read20(REG_DATA_INFRARED);
    uint16_t prox11 = read11(REG_PROXIMITY_DATA);

    uint16_t r16 = (uint16_t)(r20 >> 4);
    uint16_t g16 = (uint16_t)(g20 >> 4);
    uint16_t b16 = (uint16_t)(b20 >> 4);
    uint16_t ir16 = (uint16_t)(ir20 >> 4);

    uint8_t prox8 = (prox11 > 255) ? 255 : (uint8_t)prox11;
    uint8_t ir_hi8 = (uint8_t)(ir16 >> 8);

    if (xSemaphoreTake(g_pktMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pkt.r16 = r16;
      g_pkt.g16 = g16;
      g_pkt.b16 = b16;
      g_pkt.prox8 = prox8;
      g_pkt.ir_hi8 = ir_hi8;
      xSemaphoreGive(g_pktMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(COLOR_TASK_PERIOD_MS));
  }
}

void CANTxTask(void *arg) {
  (void)arg;
  for (;;) {
    ColorPacket local = {};
    if (xSemaphoreTake(g_pktMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local = g_pkt;
      xSemaphoreGive(g_pktMutex);
    }

    uint8_t data[8];
    packBytes(local.r16, local.g16, local.b16, local.prox8, local.ir_hi8, data);

    uint16_t stdId = (uint16_t)((CAN_BASE_STD_ID + (g_deviceNumber & 0x3F)) & 0x7FF);
    twai_message_t msg = {};
    msg.identifier = stdId; // standard 11-bit
    msg.extd = 0;           // not extended
    msg.rtr = 0;            // data frame
    msg.data_length_code = 8;
    for (int i = 0; i < 8; ++i) msg.data[i] = data[i];

    (void)twai_transmit(&msg, pdMS_TO_TICKS(10)); // ignore transient errors

    vTaskDelay(pdMS_TO_TICKS(CANTX_TASK_PERIOD_MS));
  }
}

// =================== Serial CAN-ID Helper ===================
void TaskCANIDHelper(void* parameter) {
  Serial.println("[CANID] Helper task started. Use &CANID SET xx / SAVE / GET / HELP");
  while (true) {
    while (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.equalsIgnoreCase("&CANID HELP")) {
        Serial.println("[CANID] Commands:");
        Serial.println("  &CANID GET");
        Serial.println("  &CANID SET xx   (0..63)");
        Serial.println("  &CANID SAVE     (persist + reboot)");
        continue;
      }

      if (line.startsWith("&CANID SET ")) {
        int val = line.substring(11).toInt();
        if (val >= 0 && val <= 63) {
          g_deviceNumber = (uint8_t)val;
          Serial.printf("[CANID] Running DEVICE_NUMBER set to %d (ID=0x%03X)\n",
                        (int)g_deviceNumber,
                        (unsigned int)((CAN_BASE_STD_ID + (g_deviceNumber & 0x3F)) & 0x7FF));
        } else {
          Serial.println("[CANID] Invalid value. Must be 0–63.");
        }
      } else if (line.equalsIgnoreCase("&CANID SAVE")) {
        EEPROMSaveCANID();
        Serial.println("[CANID] Saved to EEPROM. Rebooting...");
        delay(200);
        ESP.restart();
      } else if (line.equalsIgnoreCase("&CANID GET")) {
        uint8_t eepromVal = EEPROMLoadCANID();
        Serial.printf("[CANID] Current=%d, EEPROM=%d, Default=%d, EffectiveID=0x%03X\n",
                      (int)g_deviceNumber, (int)eepromVal, (int)DEFAULT_DEVICE_NUMBER,
                      (unsigned int)((CAN_BASE_STD_ID + (g_deviceNumber & 0x3F)) & 0x7FF));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ========= Setup / Loop =========
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println(F("ESP32 + REV Color Sensor V3 + CAN 0x184 + Serial CANID (40 Hz)"));

  EEPROMInit();
  g_deviceNumber = EEPROMLoadCANID();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!sensorInit()) {
    Serial.println(F("Color sensor init FAILED"));
  } else {
    Serial.println(F("Color sensor init OK"));
  }

  if (!canInit()) {
    Serial.println(F("CAN init FAILED (check transceiver & pins)"));
  } else {
    uint16_t effId = (uint16_t)((CAN_BASE_STD_ID + (g_deviceNumber & 0x3F)) & 0x7FF);
    Serial.printf("CAN init OK @ 1 Mbps, std base 0x%03X, device %d => ID 0x%03X\n",
                  CAN_BASE_STD_ID, (int)g_deviceNumber, effId);
  }

  g_pktMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(ColorSensorTask, "ColorSensorTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(CANTxTask,      "CANTxTask",      3072, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCANIDHelper,"TaskCANIDHelper",3072, nullptr, 1, nullptr, 1);

  Serial.println("[CANID] Type &CANID HELP for commands.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
