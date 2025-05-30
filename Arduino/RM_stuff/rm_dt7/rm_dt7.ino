#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>

#define CAN_CS_PIN 53
MCP_CAN CAN(CAN_CS_PIN);
Servo myServo;

// === CAN Constants ===
#define DEVICE_ID        0x0A  //DONOT CHANGE
#define MANUFACTURER_ID  0x08  //DONOT CHANGE
#define DEVICE_NUMBER    33  // Device Number 0-63
#define STATUS_API_ID        0x180
#define COLOR_SENSOR_API_ID  0x184
#define CONTROL_API_ID       0x190
#define DT7_API1_ID          0x185
#define DT7_API2_ID          0x186
#define DT7_API3_ID          0x187
#define HEARTBEAT_ID         0x01011840

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

// === Precomputed CAN IDs ===
uint32_t STATUS_ID;
uint32_t COLOR_SENSOR_ID;
uint32_t CONTROL_ID;
uint32_t DT7_FRAME1_ID;
uint32_t DT7_FRAME2_ID;
uint32_t DT7_FRAME3_ID;

const int buttonPins[3] = {3, 4, 5};
const int ledPins[2] = {6, 7};
const int analogPin = A0;
const int servoPin = 9;

bool sensorOK = false;
int prevAngle = -1;
volatile bool last_enabled = false;
volatile unsigned long heartbeat_last_seen = 0;

// === DBUS ===
#define DBUS_FRAME_SIZE 18
#define DBUS_BAUD 100000
#define DBUS_TIMEOUT_MS 100
uint8_t dbusBuf[DBUS_FRAME_SIZE];
uint8_t dbusIdx = 0;
unsigned long lastDbusByte = 0;
unsigned long lastDbusFrame = 0;
bool dbusConnected = false;

uint8_t computeCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

// === Color Sensor I2C ===
#define REV_SENSOR_ADDR  0x52
#define REG_PART_ID      0x06
#define REG_MAIN_CTRL    0x00
#define REG_PROX_DATA    0x08
#define REG_IR_DATA      0x0A
#define REG_GREEN_DATA   0x0D
#define REG_BLUE_DATA    0x10
#define REG_RED_DATA     0x13

void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(REV_SENSOR_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(REV_SENSOR_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(REV_SENSOR_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

uint32_t read20(uint8_t reg) {
  Wire.beginTransmission(REV_SENSOR_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(REV_SENSOR_ADDR, 3);
  return ((uint32_t)Wire.read()) |
         ((uint32_t)Wire.read() << 8) |
         ((uint32_t)Wire.read() << 16);
}

// === Tasks ===

void TaskCANSend(void* pvParameters) {
  for (;;) {
    byte btn = 0;
    for (int i = 0; i < 3; i++) {
      if (!digitalRead(buttonPins[i])) btn |= (1 << i);
    }
    int analogVal = analogRead(analogPin);
    byte tx[8] = {btn, (analogVal >> 8), analogVal & 0xFF};
    CAN.sendMsgBuf(STATUS_ID, 1, 8, tx);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskCANReceive(void* pvParameters) {
  for (;;) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId;
      byte len;
      byte rx[8];
      CAN.readMsgBuf(&rxId, &len, rx);
      rxId &= 0x1FFFFFFF;

      if (rxId == HEARTBEAT_ID && len == 8) {
        heartbeat_last_seen = millis();
        last_enabled = (rx[4] >> 4) & 0x01;
      }

      if (rxId == CONTROL_ID && len >= 2) {
        digitalWrite(ledPins[0], rx[0] & 0x01);
        digitalWrite(ledPins[1], rx[0] & 0x02);
        if (rx[1] != prevAngle) {
          myServo.write(rx[1]);
          prevAngle = rx[1];
        }
      }
    }
    vTaskDelay(1);
  }
}

void TaskHeartbeatLED(void* pvParameters) {
  for (;;) {
    bool connected = (millis() - heartbeat_last_seen) < 500;
    digitalWrite(13, connected);
    vTaskDelay(200);
  }
}

void TaskColorSensor(void* pvParameters) {
  uint8_t retry = 0;
  for (;;) {
    byte tx[8] = {0};
    if (sensorOK) {
      uint32_t r = read20(REG_RED_DATA);
      uint32_t g = read20(REG_GREEN_DATA);
      uint32_t b = read20(REG_BLUE_DATA);
      uint32_t ir = read20(REG_IR_DATA);
      uint8_t p = read8(REG_PROX_DATA);

      if (r == 0 && g == 0 && b == 0 && ir == 0 && p == 0) {
        sensorOK = false; tx[0] = 0xFF;
      } else {
        tx[0] = r >> 8; tx[1] = r;
        tx[2] = g >> 8; tx[3] = g;
        tx[4] = b >> 8; tx[5] = b;
        tx[6] = p; tx[7] = ir >> 8;
        retry = 0;
      }
    } else {
      tx[0] = 0xFF;
      if (++retry >= 10 && read8(REG_PART_ID) == 0xC2) {
        write8(REG_MAIN_CTRL, 0b00000111);
        sensorOK = true;
      }
    }
    CAN.sendMsgBuf(COLOR_SENSOR_ID, 1, 8, tx);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskDT7Receiver(void* pvParameters) {
  Serial1.begin(DBUS_BAUD, SERIAL_8E1);
  for (;;) {
    while (Serial1.available()) {
      dbusBuf[dbusIdx++] = Serial1.read();
      lastDbusByte = millis();

      if (dbusIdx == DBUS_FRAME_SIZE) {
        lastDbusFrame = millis(); dbusConnected = true; dbusIdx = 0;
        uint8_t crc = computeCRC8(dbusBuf, DBUS_FRAME_SIZE);
        CAN.sendMsgBuf(DT7_FRAME1_ID, 1, 8, dbusBuf);
        CAN.sendMsgBuf(DT7_FRAME2_ID, 1, 8, dbusBuf + 8);
        uint8_t tail[3] = {dbusBuf[16], dbusBuf[17], crc};
        CAN.sendMsgBuf(DT7_FRAME3_ID, 1, 3, tail);
      }
    }

    if (dbusIdx > 0 && millis() - lastDbusByte > 20) dbusIdx = 0;
    if (!dbusConnected || (millis() - lastDbusFrame > DBUS_TIMEOUT_MS)) {
      dbusConnected = false;
      uint8_t tx[8] = {0xFF};
      CAN.sendMsgBuf(DT7_FRAME1_ID, 1, 8, tx);
      CAN.sendMsgBuf(DT7_FRAME2_ID, 1, 8, tx);
      CAN.sendMsgBuf(DT7_FRAME3_ID, 1, 3, tx);
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (int i = 0; i < 3; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  pinMode(6, OUTPUT); pinMode(7, OUTPUT); pinMode(13, OUTPUT);
  myServo.attach(servoPin); myServo.write(90);

  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN initialized");

  // Precompute and print CAN IDs
  STATUS_ID        = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, STATUS_API_ID, DEVICE_NUMBER);
  COLOR_SENSOR_ID  = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, COLOR_SENSOR_API_ID, DEVICE_NUMBER);
  CONTROL_ID       = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, CONTROL_API_ID, DEVICE_NUMBER);
  DT7_FRAME1_ID    = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, DT7_API1_ID, DEVICE_NUMBER);
  DT7_FRAME2_ID    = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, DT7_API2_ID, DEVICE_NUMBER);
  DT7_FRAME3_ID    = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, DT7_API3_ID, DEVICE_NUMBER);

  Serial.print("STATUS_ID: 0x"); Serial.println(STATUS_ID, HEX);
  Serial.print("COLOR_SENSOR_ID: 0x"); Serial.println(COLOR_SENSOR_ID, HEX);
  Serial.print("CONTROL_ID: 0x"); Serial.println(CONTROL_ID, HEX);
  Serial.print("DT7_FRAME1_ID: 0x"); Serial.println(DT7_FRAME1_ID, HEX);
  Serial.print("DT7_FRAME2_ID: 0x"); Serial.println(DT7_FRAME2_ID, HEX);
  Serial.print("DT7_FRAME3_ID: 0x"); Serial.println(DT7_FRAME3_ID, HEX);

  if (read8(REG_PART_ID) == 0xC2) {
    write8(REG_MAIN_CTRL, 0b00000111);
    sensorOK = true;
  }

  xTaskCreate(TaskCANSend, "CANSend", 128, NULL, 1, NULL);
  xTaskCreate(TaskCANReceive, "CANRecv", 128, NULL, 2, NULL);
  xTaskCreate(TaskHeartbeatLED, "Heartbeat", 128, NULL, 1, NULL);
  xTaskCreate(TaskColorSensor, "Color", 256, NULL, 1, NULL);
  xTaskCreate(TaskDT7Receiver, "DT7Recv", 256, NULL, 1, NULL);
}

void loop() {}
