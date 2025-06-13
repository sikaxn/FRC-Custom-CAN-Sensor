#include <Wire.h>
#include <VL53L1X.h>

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Number of sensors
#define NUM_SENSORS 4

VL53L1X sensor[NUM_SENSORS];

// === Per-sensor XSHUT GPIO pins ===
const uint8_t xshutPins[NUM_SENSORS] = {16, 17, 18, 19};

// === Per-sensor I2C addresses (must be unique) ===
const uint8_t sensorAddresses[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33};

// === Per-sensor ROICenter (Pololu supports only center, not size) ===
const uint8_t roiCenters[NUM_SENSORS] = {199, 199, 199, 199};

// === Per-sensor timing budget (μs) ===
const uint32_t timingBudgets[NUM_SENSORS] = {50000, 50000, 50000, 50000};

// === Per-sensor ranging mode ===
const VL53L1X::DistanceMode rangingModes[NUM_SENSORS] = {
  VL53L1X::Long,
  VL53L1X::Long,
  VL53L1X::Long,
  VL53L1X::Long
};

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Power down all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  delay(100);  // Ensure all sensors are off

  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(xshutPins[i], HIGH);  // Power on one sensor
    delay(300);  // Give time to boot fully

    sensor[i].setTimeout(500);
    if (!sensor[i].init()) {
      Serial.printf("Sensor %d failed to initialize!\n", i);
      while (1);
    }

    sensor[i].setAddress(sensorAddresses[i]);

    sensor[i].setROICenter(roiCenters[i]);
    sensor[i].setDistanceMode(rangingModes[i]);
    sensor[i].setMeasurementTimingBudget(timingBudgets[i]);

    sensor[i].startContinuous(50);
  }

  Serial.println("Sensor0\tSensor1\tSensor2\tSensor3");  // Plotter labels
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int dist = sensor[i].read();
    if (sensor[i].timeoutOccurred()) dist = 0;

    Serial.print(dist);
    if (i < NUM_SENSORS - 1) Serial.print('\t');
  }
  Serial.println();
  delay(50);
}
