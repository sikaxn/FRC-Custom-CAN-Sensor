#include <Wire.h>
#include <Arduino.h>
#include <TMD3725.h>

// ESP32 pins for I2C
static constexpr uint8_t SDA_PIN = 21;
static constexpr uint8_t SCL_PIN = 22;

TMD3725 tmd3725;

int reginfo[35];
int colorArray[9];

// State
bool sensorOnline = false;
unsigned long lastRetryMs = 0;
const unsigned long RETRY_INTERVAL_MS = 1000;

// Stuck-zero detection
unsigned int zeroFrameCount = 0;
const unsigned int ZERO_FRAME_LIMIT = 5;   // how many all-zero frames before we force re-init

// Saturation detection (near full-scale)
const uint16_t SAT_LIMIT = 65000;  // anything >= this we treat as saturated (16-bit)

// Last-good cached sample
bool     haveGoodSample = false;
uint16_t lastRed   = 0;
uint16_t lastGreen = 0;
uint16_t lastBlue  = 0;
uint16_t lastClear = 0;
uint16_t lastIR    = 0;
uint8_t  lastProx  = 0;

// Helper: combine two bytes into 16-bit little-endian
static inline uint16_t make16(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

void tryInitSensor() {
  Serial.println(F("Attempting TMD3725 init..."));
  if (tmd3725.begin()) {
    if (tmd3725.init(reginfo) == 0) {
      sensorOnline = true;
      zeroFrameCount = 0;
      Serial.println(F("TMD3725 init OK, sensorOnline = true\n"));
      return;
    } else {
      Serial.println(F("TMD3725 init() returned error\n"));
    }
  } else {
    Serial.println(F("tmd3725.begin() failed\n"));
  }
  sensorOnline = false;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(F("TMD3725 -> REV-style RGB/IR/PROX with saturation handling"));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  tryInitSensor();
}

void loop() {
  // If we think sensor is offline, periodically try to re-init
  if (!sensorOnline) {
    // While offline: print last good values if we have them; otherwise zeros
    if (haveGoodSample) {
      Serial.print(F("[OFFLINE] R:"));   Serial.print(lastRed);
      Serial.print(F(" G:"));            Serial.print(lastGreen);
      Serial.print(F(" B:"));            Serial.print(lastBlue);
      Serial.print(F(" IR:"));           Serial.print(lastIR);
      Serial.print(F(" PROX:"));         Serial.print(lastProx);
      Serial.println();
    } else {
      Serial.println(F("R:0 G:0 B:0 IR:0 PROX:0"));
    }

    if (millis() - lastRetryMs > RETRY_INTERVAL_MS) {
      lastRetryMs = millis();
      tryInitSensor();
    }
    delay(50);
    return;
  }

  // --- Read optics data from library ---
  int rc = tmd3725.get_optics_data(colorArray);
  if (rc != 0) {
    // Read failed – treat as suspect frame, increment zero count
    zeroFrameCount++;
    if (zeroFrameCount >= ZERO_FRAME_LIMIT) {
      //Serial.println(F("❌ Repeated get_optics_data() errors → forcing re-init"));
      sensorOnline = false;
    }
    delay(50);
    return;
  }

  // Decode raw channels
  uint16_t clear = make16((uint8_t)colorArray[0], (uint8_t)colorArray[1]);
  uint16_t red   = make16((uint8_t)colorArray[2], (uint8_t)colorArray[3]);
  uint16_t green = make16((uint8_t)colorArray[4], (uint8_t)colorArray[5]);
  uint16_t blue  = make16((uint8_t)colorArray[6], (uint8_t)colorArray[7]);
  uint8_t  prox  = (uint8_t)colorArray[8];

  // Compute IR similar to library calib logic: (R+G+B - C)/2
  int32_t rawr = red;
  int32_t rawg = green;
  int32_t rawb = blue;
  int32_t rawc = clear;
  int32_t ir   = ((rawr + rawg + rawb) - rawc) / 2;
  if (ir < 0) ir = 0;
  if (ir > 65535) ir = 65535;

  // --- Detect “all-zero” stuck condition ---
  bool allZero = (red == 0 && green == 0 && blue == 0 && clear == 0 && prox == 0);

  if (allZero) {
    zeroFrameCount++;
  } else {
    zeroFrameCount = 0;
  }

  // --- Detect saturation (near full-scale) ---
  bool saturated =
      (red   >= SAT_LIMIT ||
       green >= SAT_LIMIT ||
       blue  >= SAT_LIMIT ||
       clear >= SAT_LIMIT ||
       prox  == 255);

  if (saturated) {
    //Serial.println(F("⚠ Saturation detected (values near full-scale)."));
    // We still print the values below, but if we immediately go to zeros afterwards,
    // the allZero logic will kick in and re-init.
  }

  // If we’ve seen N consecutive all-zero frames, treat sensor as stuck and re-init
  if (zeroFrameCount >= ZERO_FRAME_LIMIT) {
    //Serial.println(F("❌ Sensor appears stuck (repeated all-zero frames) → forcing re-init"));
    sensorOnline = false;
    lastRetryMs = millis();
    delay(10);
    return;
  }

  // Cache last good sample (ignore pure all-zero frames so offline doesn't just replay zeros)
  if (!allZero) {
    haveGoodSample = true;
    lastRed   = red;
    lastGreen = green;
    lastBlue  = blue;
    lastClear = clear;
    lastIR    = (uint16_t)ir;
    lastProx  = prox;
  }

  // --- REV-style print line ---
  Serial.print(F("R:"));   Serial.print(red);
  Serial.print(F(" G:"));  Serial.print(green);
  Serial.print(F(" B:"));  Serial.print(blue);
  Serial.print(F(" IR:")); Serial.print((uint16_t)ir);
  Serial.print(F(" PROX:")); Serial.print(prox);
  Serial.println();

  delay(100);  // ~10 Hz
}
