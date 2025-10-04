#include <Wire.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define ICM_ADDR 0x68

// --------- REG MAP (same as before) ----------
#define REG_BANK_SEL  0x7F
#define BANK0         0x00
#define BANK2         0x20
// BANK0
#define WHO_AM_I      0x00
#define PWR_MGMT_1    0x06
#define PWR_MGMT_2    0x07
#define ACCEL_XOUT_H  0x2D
#define GYRO_XOUT_H   0x33
#define TEMP_OUT_H    0x39
// BANK2
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG  0x14
#define ACCEL_SMPLRT  0x10
#define GYRO_SMPLRT   0x00

// --------- TUNABLES ----------
#define G_CAL_SAMPLES   800     // startup gyro bias samples (hold IMU still)
#define GYRO_DEADBAND   0.35f   // dps: ignore tiny rates
#define ACC_LP_TAU      0.06f   // s: accel low-pass time constant (0.1–0.3 good)
#define STIL_GYRO_DPS   1.5f    // dps: all |g*| below → candidate still
#define STIL_ACC_G_WIN  0.05f   // g: |mag-1.0| below → candidate still
#define STIL_HOLD_MS    500     // ms: must stay “still” this long
#define ALPHA_MOVE      0.995f  // complementary blend when moving
#define ALPHA_STILL     0.97f   // stronger accel correction when still (smaller = stronger)
#define BIAS_LEARN      0.0025f // how fast gyro bias adapts when still

// --------- GLOBAL STATE ----------
float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
float gBiasX = 0, gBiasY = 0, gBiasZ = 0;      // gyro bias (dps)
float axLP = 0, ayLP = 0, azLP = 1;            // low-passed accel (g)
unsigned long lastMicros = 0;
unsigned long stillStartMs = 0;
bool stillLatch = false;
bool printedHeader = false;

// --------- I2C helpers ----------
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
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

// --------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  bank(BANK0);
  // Reset, wake, auto clock
  wr(PWR_MGMT_1, 0x80); delay(60);
  wr(PWR_MGMT_1, 0x01); delay(10);
  wr(PWR_MGMT_2, 0x00); // enable accel+gyro

  // (Optional but recommended) Reduce bandwidth to reject vibration:
  // You previously used 0x11 (~92Hz). Try a lower cutoff (board/driver variants differ,
  // but setting DLPF config to a "lower" bandwidth helps). If your 0x11 is stable,
  // try these alternatives and compare:
  bank(BANK2);
 // wr(GYRO_CONFIG_1, 0x15);   // try lower gyro BW (e.g., ~12–23 Hz on many variants)
 // wr(ACCEL_CONFIG , 0x15);   // try lower accel BW (e.g., ~11–23 Hz)
wr(GYRO_CONFIG_1, 0x11);
wr(ACCEL_CONFIG , 0x11);

  wr(GYRO_SMPLRT  , 4);      // ~200 Hz
  wr(ACCEL_SMPLRT , 4);      // ~200 Hz
  bank(BANK0);

  Wire.setClock(400000);

  // ---- Gyro bias calibration at startup (IMU must be still!) ----
  const float gyr_sens = 500.0f / 32768.0f; // dps/LSB for ±500 dps
  long bx=0, by=0, bz=0;
  for (int i=0;i<G_CAL_SAMPLES;i++) {
    int16_t gx_raw = rd16(GYRO_XOUT_H);
    int16_t gy_raw = rd16(GYRO_XOUT_H + 2);
    int16_t gz_raw = rd16(GYRO_XOUT_H + 4);
    bx += gx_raw; by += gy_raw; bz += gz_raw;
    delay(1);
  }
  gBiasX = (bx / (float)G_CAL_SAMPLES) * gyr_sens;
  gBiasY = (by / (float)G_CAL_SAMPLES) * gyr_sens;
  gBiasZ = (bz / (float)G_CAL_SAMPLES) * gyr_sens;

  lastMicros = micros();
}

// --------- LOOP ----------
void loop() {
  if (!printedHeader) {
    Serial.println("ax\tay\taz\tgx\tgy\tgz\ttemp\troll\tpitch\tyaw");
    printedHeader = true;
  }

  // Read raw sensors
  int16_t ax_raw = rd16(ACCEL_XOUT_H);
  int16_t ay_raw = rd16(ACCEL_XOUT_H + 2);
  int16_t az_raw = rd16(ACCEL_XOUT_H + 4);
  int16_t gx_raw = rd16(GYRO_XOUT_H);
  int16_t gy_raw = rd16(GYRO_XOUT_H + 2);
  int16_t gz_raw = rd16(GYRO_XOUT_H + 4);
  int16_t t_raw  = rd16(TEMP_OUT_H);

  // Convert to eng units
  const float acc_sens = 4.0f / 32768.0f;   // g/LSB (±4g)
  const float gyr_sens = 500.0f / 32768.0f; // dps/LSB (±500 dps)
  float ax = ax_raw * acc_sens;
  float ay = ay_raw * acc_sens;
  float az = az_raw * acc_sens;
  float gx = gx_raw * gyr_sens - gBiasX;
  float gy = gy_raw * gyr_sens - gBiasY;
  float gz = gz_raw * gyr_sens - gBiasZ;
  float tempC = ((float)t_raw - 0.0f)/333.87f + 21.0f;

  // Low-pass accel (EMA)
  float dt = (micros() - lastMicros) / 1e6f;
  lastMicros = micros();
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;
  float acc_alpha = dt / (ACC_LP_TAU + dt);   // 0..1
  axLP += acc_alpha * (ax - axLP);
  ayLP += acc_alpha * (ay - ayLP);
  azLP += acc_alpha * (az - azLP);

  // Gyro deadband
  if (fabsf(gx) < GYRO_DEADBAND) gx = 0;
  if (fabsf(gy) < GYRO_DEADBAND) gy = 0;
  if (fabsf(gz) < GYRO_DEADBAND) gz = 0;

  // Stationary detection
  float gmag = sqrtf(axLP*axLP + ayLP*ayLP + azLP*azLP);
  bool gyroQuiet = (fabsf(gx) < STIL_GYRO_DPS && fabsf(gy) < STIL_GYRO_DPS && fabsf(gz) < STIL_GYRO_DPS);
  bool acc1g     = (fabsf(gmag - 1.0f) < STIL_ACC_G_WIN);
  bool maybeStill = gyroQuiet && acc1g;

  unsigned long nowMs = millis();
  if (maybeStill) {
    if (!stillLatch) { stillLatch = true; stillStartMs = nowMs; }
  } else {
    stillLatch = false;
  }
  bool isStill = stillLatch && (nowMs - stillStartMs >= STIL_HOLD_MS);

  // Complementary filter
  float alpha = isStill ? ALPHA_STILL : ALPHA_MOVE;

  // integrate gyro
  float rollGyro  = rollDeg  + gx * dt;
  float pitchGyro = pitchDeg + gy * dt;
  yawDeg += gz * dt;

  // accel angles (deg)
  float accRollDeg  = atan2f(ayLP, (azLP != 0.0f ? azLP : 1e-6f)) * 180.0f / PI;
  float accPitchDeg = atan2f(-axLP, sqrtf(ayLP*ayLP + azLP*azLP))   * 180.0f / PI;

  // blend
  rollDeg  = alpha * rollGyro  + (1.0f - alpha) * accRollDeg;
  pitchDeg = alpha * pitchGyro + (1.0f - alpha) * accPitchDeg;

  // When still, adapt bias slowly and bleed yaw rate
  if (isStill) {
    // Learn residual bias so next time we’re steadier
    gBiasX = (1.0f - BIAS_LEARN) * gBiasX + BIAS_LEARN * (gBiasX + gx); // drives gx→0
    gBiasY = (1.0f - BIAS_LEARN) * gBiasY + BIAS_LEARN * (gBiasY + gy);
    gBiasZ = (1.0f - BIAS_LEARN) * gBiasZ + BIAS_LEARN * (gBiasZ + gz);
    // Also reduce tiny yaw creep
    if (fabsf(gz) < STIL_GYRO_DPS) gz = 0;
  }

  // Output data (tabs only, no text)
  Serial.printf("%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\t%.1f\t%.2f\t%.2f\t%.2f\n",
                axLP, ayLP, azLP,         // low-passed accel helps downstream too
                gx, gy, gz,
                tempC,
                rollDeg, pitchDeg, yawDeg);

  delay(5); // ~100–150 Hz overall
}
