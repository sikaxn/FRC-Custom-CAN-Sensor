#if !ARDUINO_USB_CDC_ON_BOOT
#error "ESP32-S3 USB CDC is required. Set Tools > USB CDC On Boot to Enabled."
#endif

#include <Wire.h>

constexpr uint8_t SDA_PIN = 5;
constexpr uint8_t SCL_PIN = 4;
constexpr uint32_t SCAN_INTERVAL_MS = 2000;

void scanI2C() {
  uint8_t devicesFound = 0;

  Serial.println("Scanning I2C bus...");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("  Device found at 0x%02X\n", address);
      ++devicesFound;
    } else if (error == 4) {
      Serial.printf("  Unknown error at 0x%02X\n", address);
    }
  }

  if (devicesFound == 0) {
    Serial.println("  No I2C devices found.");
  } else {
    Serial.printf("  %u device(s) found.\n", devicesFound);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give the USB CDC serial port time to enumerate.

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.printf("I2C scanner started (SDA=GP%u, SCL=GP%u)\n\n", SDA_PIN, SCL_PIN);
}

void loop() {
  scanI2C();
  delay(SCAN_INTERVAL_MS);
}
