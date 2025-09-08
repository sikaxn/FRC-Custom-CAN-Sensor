/*
  REV Color Sensor V3 (APDS-9151) on ESP32
  SDA=IO21, SCL=IO22, I2C addr 0x52

  Prints 20-bit raw R/G/B/IR and 11-bit Proximity.

  References:
  - REVLib C++ header (registers/bitfields): ColorSensorV3.h
  - REV Color Sensor V3 docs (I2C address 0x52)
*/

#include <Wire.h>

// ========= I2C & PIN =========
static constexpr uint8_t I2C_ADDR = 0x52;   // 7-bit
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;

// ========= Registers (from REV ColorSensorV3.h) =========
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

// MAIN_CTRL bitfields
static constexpr uint8_t MAIN_CTRL_PROX_ENABLE = 0x01;
static constexpr uint8_t MAIN_CTRL_LS_ENABLE   = 0x02;
static constexpr uint8_t MAIN_CTRL_RGB_MODE    = 0x04; // RGB (vs clear/ALS)

// LED freq (upper bits) and current (lower 3 bits) encodings per REV header
// We'll OR the fields together when writing REG_PROXIMITY_SENSOR_LED.
enum LEDPulseFreq : uint8_t {
  LED_FREQ_60k = 0x18,
  LED_FREQ_70k = 0x40,
  LED_FREQ_80k = 0x28,
  LED_FREQ_90k = 0x30,
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

// Proximity resolution (upper bits) and measurement rate (lower 3 bits)
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

// Color resolution (upper bits) and measurement rate (lower 3 bits)
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

// Gain (0..4) -> 1x,3x,6x,9x,18x
enum GainFactor : uint8_t {
  GAIN_1X = 0,
  GAIN_3X = 1,
  GAIN_6X = 2,
  GAIN_9X = 3,
  GAIN_18X = 4
};

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
  // little-endian, mask to 11 bits
  return (uint16_t)((b[0]) | (uint16_t(b[1]) << 8)) & 0x07FF;
}

uint32_t read20(uint8_t reg) {
  uint8_t b[3];
  if (!i2cRead(reg, b, 3)) return 0;
  // little-endian, mask to 20 bits
  return (uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16)) & 0x003FFFFF;
}

bool readPartID(uint8_t &pid) {
  return i2cRead(REG_PART_ID, &pid, 1);
}

bool readStatus(uint8_t &st) {
  return i2cRead(REG_MAIN_STATUS, &st, 1);
}

// ========= Sensor init =========
bool sensorInit() {
  // Enable both proximity and light sensors, RGB mode
  if (!i2cWrite8(REG_MAIN_CTRL, MAIN_CTRL_PROX_ENABLE | MAIN_CTRL_LS_ENABLE | MAIN_CTRL_RGB_MODE))
    return false;

  // Proximity LED: 100kHz + 50 mA (safe default), adjust as needed
  if (!i2cWrite8(REG_PROXIMITY_SENSOR_LED, (uint8_t)LED_FREQ_100k | (uint8_t)LED_CURR_50mA))
    return false;

  // Proximity pulses (# of IR pulses per measurement) – 32 is a common default
  if (!i2cWrite8(REG_PROXIMITY_SENSOR_PULSES, 32))
    return false;

  // Proximity rate: 11-bit resolution @ 100 ms
  if (!i2cWrite8(REG_PROXIMITY_SENSOR_RATE, (uint8_t)PROX_RES_11b | (uint8_t)PROX_RATE_100ms))
    return false;

  // Color measurement: 20-bit @ 50 ms
  if (!i2cWrite8(REG_LIGHT_SENSOR_MEAS_RATE, (uint8_t)COLOR_RES_20b | (uint8_t)COLOR_RATE_50ms))
    return false;

  // Color gain: 3x is the WPILib default
  if (!i2cWrite8(REG_LIGHT_SENSOR_GAIN, (uint8_t)GAIN_3X))
    return false;

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(F("REV Color Sensor V3 (APDS-9151) demo"));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // High-speed I2C per sensor docs

  // Basic presence check
  uint8_t pid = 0;
  if (!sensorInit()) {
    Serial.println(F("Init failed: I2C write error"));
  } else if (!readPartID(pid)) {
    Serial.println(F("Init ok, but failed to read PART_ID"));
  } else {
    Serial.print(F("Init ok. PART_ID=0x"));
    Serial.println(pid, HEX);
  }
}

void loop() {
  // Optional: check status to see if new data is ready
  uint8_t st = 0;
  if (readStatus(st)) {
    // bit0 = PS data ready, bit3 = LS data ready (per REV header)
    // Not strictly required to gate reads; we’ll just read regardless.
  }

  // Read values (LSB first, 20-bit for colors/IR; 11-bit for proximity)
  uint32_t red   = read20(REG_DATA_RED);
  uint32_t green = read20(REG_DATA_GREEN);
  uint32_t blue  = read20(REG_DATA_BLUE);
  uint32_t ir    = read20(REG_DATA_INFRARED);
  uint16_t prox  = read11(REG_PROXIMITY_DATA);

  // Print as CSV-ish single line for easy logging/parsing
  Serial.print(F("R:"));  Serial.print(red);
  Serial.print(F(" G:")); Serial.print(green);
  Serial.print(F(" B:")); Serial.print(blue);
  Serial.print(F(" IR:"));Serial.print(ir);
  Serial.print(F(" PROX:")); Serial.print(prox);
  Serial.println();

  delay(100); // ~10 Hz
}
