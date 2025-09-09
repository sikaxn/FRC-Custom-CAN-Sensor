/*
  ESP32 + REV Through Bore Encoder (ABI + ABS PWM) - Serial Plotter
  Pins:
    ABS PWM -> GPIO 26 (digital input)
    ENC A   -> GPIO 32
    ENC B   -> GPIO 33
    ENC I   -> GPIO 34 (index, input-only)

  Serial Plotter columns:
    count angleDeg velDegPerSec absDeg absUs indexPulse
*/

#include <Arduino.h>
#include <ESP32Encoder.h>

// ----- Pins -----
static const int PIN_ABS   = 26;  // PWM duty-cycle absolute output
static const int PIN_A     = 32;  // Quadrature A
static const int PIN_B     = 33;  // Quadrature B
static const int PIN_INDEX = 34;  // Index I (rising)

// ----- Encoder config -----
static const int32_t CPR = 8192;        // 2048 CPR x4 = 8192 counts/rev
static const uint32_t PRINT_MS = 50;    // print period for plotter
static const uint32_t ABS_TIMEOUT_US = 3000; // pulseIn timeout; > period (~1025us)

// ----- Globals -----
ESP32Encoder enc;
volatile bool g_indexPulse = false;  // set in ISR, consumed in loop
int32_t lastCount = 0;
uint32_t lastPrint = 0;

// Optional smoothing for ABS PWM (EMA)
float absUsEMA = NAN;
const float ABS_ALPHA = 0.35f;  // 0..1 (higher = less smoothing)

// ----- ISR -----
void IRAM_ATTR onIndexISR() { g_indexPulse = true; }

// ----- Helpers -----
static inline float countsToDegrees(int32_t counts) {
  int32_t mod = counts % CPR;
  if (mod < 0) mod += CPR;
  return 360.0f * (float)mod / (float)CPR;
}

// Reads the ABS PWM HIGH width (µs). Returns NAN if no valid pulse.
float readAbsPulseUs() {
  // Wait for a HIGH pulse; pulseIn returns 0 on timeout.
  unsigned long w = pulseIn(PIN_ABS, HIGH, ABS_TIMEOUT_US);
  if (w == 0) return NAN; // no pulse seen within timeout
  // Clamp to the documented 1..1024 µs window
  if (w < 1) w = 1;
  if (w > 1024) w = 1024;
  return (float)w;
}

float absUsToDegrees(float absUs) {
  // 1..1024 µs -> 0..360°
  // Map 1 µs -> 0°, 1024 µs -> 360° (spec says linear across 1024 steps)
  return (absUs - 1.0f) * (360.0f / 1023.0f);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_A,     INPUT);
  pinMode(PIN_B,     INPUT);
  pinMode(PIN_INDEX, INPUT);
  pinMode(PIN_ABS,   INPUT);

  // Quadrature via ESP32Encoder (uses PCNT hardware)
  //ESP32Encoder::useInternalWeakPullResistors = UP; // optional
  enc.attachFullQuad(PIN_A, PIN_B);
  enc.clearCount();

  // Index rising -> flag
  attachInterrupt(digitalPinToInterrupt(PIN_INDEX), onIndexISR, RISING);

  // Serial Plotter header
  Serial.println("count angleDeg velDegPerSec absDeg absUs indexPulse");
}

void loop() {
  // Apply index zeroing once per pulse, and also expose a one-shot marker
  static bool indexOneShot = false;
  if (g_indexPulse) {
    enc.setCount(0);
    g_indexPulse = false;
    indexOneShot = true; // visualize for a single print frame
  }

  const uint32_t now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;

    // Quadrature position & velocity
    int32_t count = (int32_t)enc.getCount();
    int32_t delta = count - lastCount;
    lastCount = count;

    float angleDeg = countsToDegrees(count);
    float cps = (float)delta / ((float)PRINT_MS / 1000.0f);
    float dps = cps * (360.0f / (float)CPR);

    // ABS PWM reading with light smoothing (helps jitter)
    float absUs = readAbsPulseUs();
    if (!isnan(absUs)) {
      if (isnan(absUsEMA)) absUsEMA = absUs;
      else absUsEMA = ABS_ALPHA * absUs + (1.0f - ABS_ALPHA) * absUsEMA;
    }
    float absDeg = isnan(absUsEMA) ? NAN : absUsToDegrees(absUsEMA);

    // --- Serial Plotter line (space-separated) ---
    // count angleDeg velDegPerSec absDeg absUs indexPulse
    Serial.print(count); Serial.print(' ');
    Serial.print(angleDeg, 2); Serial.print(' ');
    Serial.print(dps, 2); Serial.print(' ');
    if (isnan(absDeg)) Serial.print("nan");
    else Serial.print(absDeg, 2);
    Serial.print(' ');
    if (isnan(absUsEMA)) Serial.print("nan");
    else Serial.print(absUsEMA, 1);
    Serial.print(' ');
    Serial.println(indexOneShot ? 1 : 0);

    indexOneShot = false; // clear one-shot marker after printing
  }
}
