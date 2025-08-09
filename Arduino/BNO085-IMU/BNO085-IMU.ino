#include <Wire.h>
#include "ICM_20948.h"

ICM_20948_I2C imu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);  // ESP32 SDA/SCL
  Wire.setClock(400000);
  delay(500);

  if (!imu.begin(Wire, 0x68)) {
    Serial.println("IMU init failed");
    while (1) delay(1000);
  }

  Serial.println("IMU ready");
}

void loop() {
  if (imu.dataReady()) {
    imu.getAGMT();  // Mag will now return 0/NaN, but accel/gyro works

    Serial.print("Accel X: "); Serial.print(imu.accX(), 2);
    Serial.print(" Y: "); Serial.print(imu.accY(), 2);
    Serial.print(" Z: "); Serial.print(imu.accZ(), 2);

    Serial.print(" | Gyro X: "); Serial.print(imu.gyrX(), 2);
    Serial.print(" Y: "); Serial.print(imu.gyrY(), 2);
    Serial.print(" Z: "); Serial.println(imu.gyrZ(), 2);
  } else {
    Serial.println("Waiting for data...");
  }

  delay(100);
}
