#include <SPI.h>
#include <mcp_can.h>
#include <Arduino_FreeRTOS.h>

//─────────────────────────────
//  PIN ASSIGNMENTS
//─────────────────────────────
#define CAN_CS_PIN       10    // Uno CS pin for MCP2515

//─────────────────────────────
//  DEVICE / API CONSTANTS
//─────────────────────────────
#define DEVICE_ID         0x0A
#define MANUFACTURER_ID   0x08
#define DEVICE_NUMBER     33

#define DT7_API1_ID       0x185
#define DT7_API2_ID       0x186
#define DT7_API3_ID       0x187

//─────────────────────────────
//  DBUS / CAN SETTINGS
//─────────────────────────────
#define DBUS_FRAME_SIZE   18
#define DBUS_BAUD         100000   // DBUS @100 kbps, 8E1
#define DBUS_TIMEOUT_MS   100      // ms

MCP_CAN CAN(CAN_CS_PIN);

static uint8_t  dbusBuf[DBUS_FRAME_SIZE];
static uint8_t  dbusIdx          = 0;
static unsigned long lastDbusByte  = 0;
static unsigned long lastDbusFrame = 0;
static bool     dbusConnected     = false;

static uint32_t DT7_FRAME1_ID;
static uint32_t DT7_FRAME2_ID;
static uint32_t DT7_FRAME3_ID;

// CRC-8 (poly 0xD5)
static uint8_t computeCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

// Build 29-bit FRC CAN ID
static uint32_t makeCANMsgID(uint8_t dev, uint8_t man, uint16_t api, uint8_t num) {
  return ((uint32_t)dev << 24)
       | ((uint32_t)man << 16)
       | ((uint32_t)(api & 0x3FF) << 6)
       | (num & 0x3F);
}

//─────────────────────────────
//  DT7 Receiver Task
//─────────────────────────────
void TaskDT7Receiver(void* pvParameters) {
  for (;;) {
    // 1) Read from hardware Serial
    while (Serial.available()) {
      dbusBuf[dbusIdx++] = Serial.read();
      lastDbusByte = millis();

      // 2) On full 18-byte frame, send DT7 packets
      if (dbusIdx == DBUS_FRAME_SIZE) {
        lastDbusFrame = millis();
        dbusConnected = true;
        dbusIdx = 0;

        uint8_t crc = computeCRC8(dbusBuf, DBUS_FRAME_SIZE);
        CAN.sendMsgBuf(DT7_FRAME1_ID, 1, 8, dbusBuf);
        CAN.sendMsgBuf(DT7_FRAME2_ID, 1, 8, dbusBuf + 8);
        uint8_t tail[3] = { dbusBuf[16], dbusBuf[17], crc };
        CAN.sendMsgBuf(DT7_FRAME3_ID, 1, 3, tail);
      }
    }

    // 3) If bytes stalled >20 ms, reset buffer
    if (dbusIdx > 0 && (millis() - lastDbusByte) > 20) {
      dbusIdx = 0;
    }

    // 4) If never connected or timed out (>100 ms), send error frames
    if (!dbusConnected || (millis() - lastDbusFrame > DBUS_TIMEOUT_MS)) {
      dbusConnected = false;
      uint8_t zeros8[8] = { 0xFF,0,0,0,0,0,0,0 };
      uint8_t zeros3[3] = { 0xFF,0,0 };
      CAN.sendMsgBuf(DT7_FRAME1_ID, 1, 8, zeros8);
      CAN.sendMsgBuf(DT7_FRAME2_ID, 1, 8, zeros8);
      CAN.sendMsgBuf(DT7_FRAME3_ID, 1, 3, zeros3);
    }

    vTaskDelay(1);
  }
}

void setup() {
  // Use hardware Serial for DBUS (100 kbps, 8E1)
  Serial.begin(DBUS_BAUD, SERIAL_8E1);

  // Init CAN @1 Mbps (8 MHz crystal)
  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  CAN.setMode(MCP_NORMAL);

  // Precompute DT7 CAN IDs
  DT7_FRAME1_ID = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, DT7_API1_ID, DEVICE_NUMBER);
  DT7_FRAME2_ID = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, DT7_API2_ID, DEVICE_NUMBER);
  DT7_FRAME3_ID = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, DT7_API3_ID, DEVICE_NUMBER);

  // Start DT7 task
  xTaskCreate(TaskDT7Receiver, "DT7Recv", 256, NULL, 1, NULL);
}

void loop() {
  // empty—FreeRTOS handles the task
}
