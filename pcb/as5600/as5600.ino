/*
  ESP32 ↔ AS5600 (I2C) with Serial Plotter, two buttons, and RGB LED

  I2C: SDA=21, SCL=22
  Buttons (to GND, INPUT_PULLUP): IO0=BOOT, IO17=SW0 → set software zero
  RGB LED: IO13=R, IO14=B, IO15=G

  Serial Plotter line (10 Hz):
    AbsDeg:<float> RelDeg:<float> AGC:<int> Magn:<int> State:<int>
  State numeric: 0=OK, 1=NoMag, 2=TooWeak, 3=TooStrong, 4=CommFail

  Blink rule: If NoMag AND abs angle == 0 ticks → RED blinks @ 2 Hz.

  If your LED colors still look inverted, set LED_ACTIVE_HIGH to 1.
*/

#include <Wire.h>

// ====== Pins ======
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define PIN_BTN_BOOT   0
#define PIN_BTN_SW0   17
#define PIN_LED_R     13
#define PIN_LED_B     14
#define PIN_LED_G     15

// ====== LED polarity (set to 0 for common-anode / active-LOW; 1 for active-HIGH) ======
#define LED_ACTIVE_HIGH 0

// ====== AS5600 Registers ======
#define AS5600_ADDR           0x36
#define AS5600_STATUS         0x0B   // MD bit5, ML bit4, MH bit3
#define AS5600_RAW_ANGLE_MSB  0x0C
#define AS5600_RAW_ANGLE_LSB  0x0D
#define AS5600_ANGLE_MSB      0x0E
#define AS5600_ANGLE_LSB      0x0F
#define AS5600_AGC            0x1A
#define AS5600_MAG_MSB        0x1B
#define AS5600_MAG_LSB        0x1C

// ====== Types ======
enum MagnetState { MAG_OK=0, MAG_NO_MAGNET=1, MAG_TOO_WEAK=2, MAG_TOO_STRONG=3, MAG_COMM_FAIL=4 };

// ====== Globals ======
uint16_t zeroOffset = 0;       // software zero (0..4095)
unsigned long lastSampleMs = 0;
const uint32_t SAMPLE_PERIOD_MS = 100; // 10 Hz for Serial Plotter

// Blink timing (2 Hz)
unsigned long lastBlinkMs = 0;
bool blinkOn = false;

// ====== Helpers ======
inline uint8_t ledLevel(bool on) {
  // Map logical ON/OFF to pin level based on polarity
  return (LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
}
void setRGB(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, ledLevel(r));
  digitalWrite(PIN_LED_G, ledLevel(g));
  digitalWrite(PIN_LED_B, ledLevel(b));
}

float ticksToDegrees(uint16_t t) { return (t * 360.0f) / 4096.0f; }
uint16_t wrap12(int32_t v) { v %= 4096; if (v < 0) v += 4096; return (uint16_t)v; }

// Debounce (active-LOW press)
bool buttonPressed(uint8_t pin) {
  bool cur = digitalRead(pin);
  struct Deb { uint32_t t = 0; uint8_t s = HIGH; };
  static Deb db0, db1;
  Deb &db = (pin == PIN_BTN_BOOT) ? db0 : db1;

  if (cur != db.s && (millis() - db.t) > 20) {
    db.t = millis();
    db.s = cur;
    if (db.s == LOW) return true;
  }
  return false;
}

// ====== I2C helpers ======
bool readByte(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)AS5600_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}
bool readWord(uint8_t regMSB, uint16_t &val) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(regMSB);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return false;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  val = ((uint16_t)msb << 8) | lsb;
  return true;
}

// ====== AS5600 getters ======
bool getStatusReg(uint8_t &s)          { return readByte(AS5600_STATUS, s); }
bool getAGC(uint8_t &agc)              { return readByte(AS5600_AGC, agc); }
bool getMagnitude(uint16_t &mag)       { return readWord(AS5600_MAG_MSB, mag); }
bool getRawAngle(uint16_t &raw)        { if(!readWord(AS5600_RAW_ANGLE_MSB, raw)) return false; raw &= 0x0FFF; return true; }
bool getScaledAngle(uint16_t &ang)     { if(!readWord(AS5600_ANGLE_MSB, ang))     return false; ang &= 0x0FFF; return true; }

MagnetState getMagnetState() {
  uint8_t s;
  if (!getStatusReg(s)) return MAG_COMM_FAIL;
  bool md = s & 0x20; // magnet detected
  bool ml = s & 0x10; // too weak
  bool mh = s & 0x08; // too strong
  if (!md) return MAG_NO_MAGNET;
  if (ml)  return MAG_TOO_WEAK;
  if (mh)  return MAG_TOO_STRONG;
  return MAG_OK;
}

void showStateOnLed(MagnetState ms, bool blinkRedNow) {
  // Priority blink: if requested, blink RED regardless of normal mapping
  if (blinkRedNow) {
    setRGB(true, false, false);   // RED on
    return;
  }

  switch (ms) {
    case MAG_OK:         setRGB(false, true,  false); break; // GREEN
    case MAG_NO_MAGNET:  setRGB(true,  false, false); break; // RED
    case MAG_TOO_WEAK:   setRGB(false, false, true ); break; // BLUE
    case MAG_TOO_STRONG: setRGB(true,  false, true ); break; // MAGENTA
    default:             setRGB(true,  true,  false); break; // YELLOW (comm fail)
  }
}

// ====== Arduino ======
void setup() {
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
  pinMode(PIN_BTN_SW0,  INPUT_PULLUP);

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  setRGB(false, false, false);

  Serial.begin(115200);
  // No extra banner prints—Serial Plotter likes clean lines

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // Initialize zero to current angle (if available)
  uint16_t a = 0;
  if (getScaledAngle(a)) zeroOffset = a;
}

void loop() {
  // Buttons: set software zero to current angle
  if (buttonPressed(PIN_BTN_BOOT) || buttonPressed(PIN_BTN_SW0)) {
    uint16_t a;
    if (getScaledAngle(a)) zeroOffset = a;
  }

  // Read sensor values
  uint16_t raw = 0, ang = 0, mag = 0;
  uint8_t agc = 0;
  bool okRaw = getRawAngle(raw);
  bool okAng = getScaledAngle(ang);
  bool okAGC = getAGC(agc);
  bool okMag = getMagnitude(mag);

  MagnetState ms = getMagnetState();

  // Blink rule: NoMag AND absolute angle == 0
  bool blinkCondition = (ms == MAG_NO_MAGNET) && okAng && (ang == 0);
  bool blinkNow = false;
  if (blinkCondition) {
    if (millis() - lastBlinkMs >= 250) { // 2 Hz (toggle every 250 ms)
      lastBlinkMs = millis();
      blinkOn = !blinkOn;
    }
    blinkNow = blinkOn;
  } else {
    // Ensure solid color when not blinking
    blinkOn = false;
  }

  // Update LED
  showStateOnLed(ms, blinkNow);

  // Print to Serial Plotter at ~10 Hz
  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;

    // Prepare values (0 if read failed)
    float degAbs = okAng ? ticksToDegrees(ang) : 0.0f;
    uint16_t relTicks = okAng ? wrap12((int32_t)ang - (int32_t)zeroOffset) : 0;
    float degRel = ticksToDegrees(relTicks);
    uint16_t magn = okMag ? mag : 0;
    uint8_t agcv = okAGC ? agc : 0;

    // One clean line for Serial Plotter:
    // (use labels so multiple series show up)
    Serial.print("AbsDeg:");  Serial.print(degAbs, 2);
    Serial.print(" RelDeg:"); Serial.print(degRel, 2);
    Serial.print(" AGC:");    Serial.print(agcv);
    Serial.print(" Magn:");   Serial.print(magn);
    Serial.print(" State:");  Serial.print((int)ms);
    Serial.println();
  }
}
