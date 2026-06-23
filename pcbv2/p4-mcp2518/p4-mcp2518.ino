#include <SPI.h>
#include <ACAN2517FD.h>

// =====================================================
// USER PIN CONFIG
// Change these to GPIOs that are actually exposed
// and not used by your Waveshare ESP32-P4-Module-DEV-KIT
// =====================================================
static const uint8_t PIN_SPI_SCK  = 36;  // example only
static const uint8_t PIN_SPI_MISO = 37;  // example only
static const uint8_t PIN_SPI_MOSI = 38;  // example only
static const uint8_t PIN_CAN_CS   = 45;  // example only
static const uint8_t PIN_CAN_INT  = 46;  // example only

// SPI object
SPIClass spi(FSPI);   // If this does not compile on your core version,
// try SPIClass spi(SPI);

// MCP2518FD object
ACAN2517FD can(PIN_CAN_CS, spi, PIN_CAN_INT);

// =====================================================
// FRC CAN constants
// =====================================================
static const uint8_t  DEVICE_ID       = 0x0A;   // team device type area
static const uint8_t  MANUFACTURER_ID = 0x08;   // team use
static const uint8_t  DEVICE_NUMBER   = 33;
static const uint16_t API_ID          = 0x100;

uint32_t makeCANMsgID(uint8_t deviceID,
                      uint8_t manufacturerID,
                      uint16_t apiID,
                      uint8_t deviceNumber) {
  return ((uint32_t)deviceID << 24) |
         ((uint32_t)manufacturerID << 16) |
         ((uint32_t)apiID << 6) |
         (deviceNumber & 0x3F);
}

static const uint32_t TX_CAN_ID =
    makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, API_ID, DEVICE_NUMBER);

// Toggle byte every second
uint32_t lastToggleMs = 0;
uint8_t flipByte = 0;

// MCP2518FD interrupt trampoline
void IRAM_ATTR onCANInterrupt() {
  can.isr();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32-P4 + MCP2518FD FRC CAN sender starting...");
  Serial.printf("TX CAN ID = 0x%08lX\n", TX_CAN_ID);

  pinMode(PIN_CAN_INT, INPUT);

  // Start SPI on chosen pins
  spi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);

  // Most MCP2518FD boards use a 40 MHz crystal.
  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz,
      1000UL * 1000UL,       // nominal/arbitration bitrate: 1 Mbps
      DataBitRateFactor::x1  // data phase same speed
  );

  // Force classic CAN 2.0 mode, not FD frames
  settings.mCANFDEnabled = false;

  // Small FIFOs are enough here
  settings.mDriverTransmitFIFOSize = 8;
  settings.mDriverReceiveFIFOSize  = 8;

  uint32_t errorCode = can.begin(settings, onCANInterrupt);

  if (errorCode != 0) {
    Serial.printf("CAN init failed, error=0x%08lX\n", errorCode);
    while (true) {
      delay(1000);
    }
  }

  Serial.println("CAN init OK");
}

void loop() {
  uint32_t now = millis();

  if (now - lastToggleMs >= 1000) {
    lastToggleMs = now;
    flipByte ^= 0x01;

    CANFDMessage frame;
    frame.id  = TX_CAN_ID;
    frame.ext = true;   // FRC-style 29-bit extended ID
    frame.len = 8;

    frame.data[0] = flipByte;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    // classic CAN 2.0 frame
    frame.fdf = false;
    frame.brs = false;

    bool ok = can.tryToSend(frame);

    Serial.printf("Send %s, byte0=%u\n", ok ? "OK" : "FAIL", flipByte);
  }
}