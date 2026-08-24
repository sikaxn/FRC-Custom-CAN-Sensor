#include <Wire.h>
#include <VL53L1X.h>

// ESP32 wiring
constexpr uint8_t XSHUT_PIN = 6;
constexpr uint8_t SDA_PIN = 5;
constexpr uint8_t SCL_PIN = 4;

VL53L1X sensor;
bool sensorReady = false;
unsigned long lastInitAttemptMs = 0;

bool initializeSensor() {
  lastInitAttemptMs = millis();
  Serial.println("Initializing VL53L1X...");

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("ERROR: VL53L1X not detected; will retry in 2 seconds.");
    return false;
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);  // 50 ms
  sensor.startContinuous(50);                 // one measurement about every 50 ms
  Serial.println("VL53L1X ready; printing distance in millimeters.");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // gives the USB serial monitor time to attach after reset
  Serial.println();
  Serial.println("\nVL53L1X test starting");
  Serial.printf("Pins: XSHUT=%u, SDA=%u, SCL=%u\n", XSHUT_PIN, SDA_PIN, SCL_PIN);

  // XSHUT is active-low. Reset the sensor, then release it to boot.
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  Serial.println("Sensor held in reset");
  delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.println("I2C started at 400 kHz");

  digitalWrite(XSHUT_PIN, HIGH);
  Serial.println("Sensor reset released; waiting for boot");
  delay(100);

  sensorReady = initializeSensor();
}

void loop() {
  // Keep USB CDC serial responsive even when the sensor is absent or faulty.
  if (!sensorReady) {
    if (millis() - lastInitAttemptMs >= 2000) {
      sensorReady = initializeSensor();
    }
    delay(10);
    return;
  }

  uint16_t distanceMm = sensor.read();

  if (sensor.timeoutOccurred()) {
    Serial.println("ERROR: sensor read timed out");
    sensor.stopContinuous();
    sensorReady = false;
  } else {
    Serial.print("Distance: ");
    Serial.print(distanceMm);
    Serial.println(" mm");
  }

  delay(100);
}
