#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define ICM_ADDR 0x68

// Bank select
#define REG_BANK_SEL  0x7F
#define BANK0         0x00
#define BANK2         0x20

// BANK0
#define WHO_AM_I      0x00
#define PWR_MGMT_1    0x06
#define PWR_MGMT_2    0x07
#define INT_STATUS    0x19
#define ACCEL_XOUT_H  0x2D   // 0x2D..0x32 accel XYZ
#define GYRO_XOUT_H   0x33   // 0x33..0x38 gyro XYZ
#define TEMP_OUT_H    0x39   // 0x39..0x3A temp

// BANK2 (config)
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG  0x14
#define ACCEL_SMPLRT  0x10
#define GYRO_SMPLRT   0x00

uint8_t rd(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false)) return 0xFF;
  Wire.requestFrom(ICM_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission(true) == 0;
}
bool bank(uint8_t b) { return wr(REG_BANK_SEL, b); }

int16_t rd16(uint8_t regH) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(regH);
  if (Wire.endTransmission(false)) return 0;
  Wire.requestFrom(ICM_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  int16_t v = (int16_t)((Wire.read() << 8) | Wire.read());
  return v;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  bank(BANK0);
  uint8_t who = rd(WHO_AM_I);
  Serial.printf("WHO_AM_I: 0x%02X (expect 0xEA)\n", who);

  // Reset, wake, auto clock
  wr(PWR_MGMT_1, 0x80); delay(50);
  wr(PWR_MGMT_1, 0x01); delay(10);
  wr(PWR_MGMT_2, 0x00); // enable accel+gyro

  // Configure ranges & rates
  bank(BANK2);
  wr(GYRO_CONFIG_1, 0x11);   // DLPF on, 500 dps, ~92 Hz bw
  wr(ACCEL_CONFIG , 0x11);   // DLPF on, 4g, ~111 Hz bw
  wr(GYRO_SMPLRT  , 9);      // ~100 Hz
  wr(ACCEL_SMPLRT , 9);      // ~100 Hz

  bank(BANK0);
  Serial.println("ICM20948 accel/gyro ready.");
}

void loop() {
  // Read accel
  int16_t ax = rd16(ACCEL_XOUT_H);
  int16_t ay = rd16(ACCEL_XOUT_H + 2);
  int16_t az = rd16(ACCEL_XOUT_H + 4);

  // Read gyro
  int16_t gx = rd16(GYRO_XOUT_H);
  int16_t gy = rd16(GYRO_XOUT_H + 2);
  int16_t gz = rd16(GYRO_XOUT_H + 4);

  // Read temp (optional)
  int16_t t  = rd16(TEMP_OUT_H);

  // Convert (approx; use your own calibration)
  const float acc_sens = 4.0f / 32768.0f;    // g/LSB for ±4g
  const float gyr_sens = 500.0f / 32768.0f;  // dps/LSB for ±500 dps
  float axg = ax * acc_sens, ayg = ay * acc_sens, azg = az * acc_sens;
  float gxd = gx * gyr_sens, gyd = gy * gyr_sens, gzd = gz * gyr_sens;
  float tempC = ((float)t - 0.0f)/333.87f + 21.0f; // rough formula

  Serial.printf("Accel[g]: %7.3f %7.3f %7.3f | Gyro[dps]: %7.2f %7.2f %7.2f | Temp[C]: %5.1f\n",
                axg, ayg, azg, gxd, gyd, gzd, tempC);
  delay(10);
}
