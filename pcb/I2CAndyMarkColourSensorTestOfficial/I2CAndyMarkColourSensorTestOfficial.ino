#include <Wire.h>

// Pin / Device Defines
#define SDA_PIN 21
#define SCL_PIN 22
#define TMD37253M_I2C_ADDR 0x39  // I2C address

// Registers
#define ENABLE_REG 0x80
#define ATIME_REG  0x81
#define WTIME_REG  0x83
#define CONTROL_REG 0x8F
#define CDATA_REG   0x94
#define RDATA_REG   0x96
#define GDATA_REG   0x98
#define BDATA_REG   0x9A
#define PDATA_REG   0x9C

// I2C state tracking
static bool i2cOffline = false;
static uint32_t lastI2cRecoveryAttempt = 0;

// Safe register write
bool safeWriteReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TMD37253M_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0)
  {
    i2cOffline = true;
    return false;
  }
  return true;
}

// Safe 16-bit register read
uint16_t safeRead16(uint8_t reg)
{
  Wire.beginTransmission(TMD37253M_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    i2cOffline = true;
    return 0;
  }
  Wire.requestFrom(TMD37253M_I2C_ADDR, 2);
  if (Wire.available() != 2)
  {
    i2cOffline = true;
    return 0;
  }

  uint16_t v = Wire.read();
  v |= (Wire.read() << 8);
  return v;
}

// Try recovering I2C bus every 3 seconds
void tryI2cRecovery()
{
  uint32_t now = millis();
  if (i2cOffline && (now - lastI2cRecoveryAttempt > 3000))
  {
    lastI2cRecoveryAttempt = now;
    Serial.println("[I2C] Attempting offline recovery...");

    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);

    if (initializeSensor())
    {
      i2cOffline = false;
      Serial.println("[I2C] Recovery SUCCESS, sensor re-initialized.");
    }
    else
    {
      Serial.println("[I2C] Recovery FAILED, will retry.");
    }
  }
}

// Sensor init now returns false on failure instead of hanging
bool initializeSensor()
{
  Serial.println("[Sensor] Re-initializing TMD37253M...");

  if (!safeWriteReg(ENABLE_REG, 0x01)) return false;
  delay(10);
  if (!safeWriteReg(ENABLE_REG, 0x07)) return false;
  if (!safeWriteReg(ATIME_REG,  0xDB)) return false;
  if (!safeWriteReg(WTIME_REG,  0xFF)) return false;
  if (!safeWriteReg(CONTROL_REG,0x02)) return false;
  if (!safeWriteReg(0x8E, 0x11)) return false;
  if (!safeWriteReg(CONTROL_REG,0x0F)) return false;

  delay(200);
  return true;
}

// Reads now fail safely when offline
bool readSensorData(uint16_t &clear, uint16_t &red, uint16_t &green, uint16_t &blue, uint16_t &proximity)
{
  if (i2cOffline)
  {
    clear = red = green = blue = proximity = 0;
    return false;
  }

  clear = safeRead16(CDATA_REG);
  red   = safeRead16(RDATA_REG);
  green = safeRead16(GDATA_REG);
  blue  = safeRead16(BDATA_REG);
  proximity = safeRead16(PDATA_REG);

  return (clear | red | green | blue | proximity) != 0;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!initializeSensor())
  {
    Serial.println("[I2C] Offline at boot, continuing in fallback mode.");
    i2cOffline = true;
  }

  
}

void loop()
{
  tryI2cRecovery();

  uint16_t C,R,G,B,P;
  if (readSensorData(C,R,G,B,P))
  {
    Serial.printf("%u, %u, %u, %u, %u\n", C,R,G,B,P);
  }
  else
  {
    Serial.println("0, 0, 0, 0, 0");
  }

  delay(500);
}
