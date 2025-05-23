#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>

// === CAN and Pins ===
#define CAN_CS_PIN 53
MCP_CAN CAN(CAN_CS_PIN);
Servo myServo;

// === CAN Constants ===
#define DEVICE_ID        0x0A  //DONOT CHANGE
#define MANUFACTURER_ID  0x08  //DONOT CHANGE
#define DEVICE_NUMBER    33
#define STATUS_API_ID        0x180
#define COLOR_SENSOR_API_ID  0x184
#define CONTROL_API_ID       0x190
#define HEARTBEAT_ID         0x01011840

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

uint32_t STATUS_ID;
uint32_t COLOR_SENSOR_ID;
uint32_t CONTROL_ID;

const int buttonPins[3] = {3, 4, 5};
const int ledPins[2]    = {6, 7};
const int analogPin     = A0;
const int servoPin      = 9;

bool prevLed0 = false;
bool prevLed1 = false;
int  prevAngle = -1;

volatile bool        last_enabled = false;
volatile unsigned long heartbeat_last_seen = 0;

bool sensorOK = false;

// REV Color Sensor V3 Registers
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
  Wire.write(reg);
  Wire.write(val);
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

void TaskCANSend(void*);
void TaskCANReceive(void*);
void TaskHeartbeatLED(void*);
void TaskColorSensor(void*);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (int i = 0; i < 3; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  for (int i = 0; i < 2; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(13, OUTPUT);
  myServo.attach(servoPin);
  myServo.write(90);

  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed...");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN initialized.");

  STATUS_ID        = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, STATUS_API_ID, DEVICE_NUMBER);
  COLOR_SENSOR_ID  = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, COLOR_SENSOR_API_ID, DEVICE_NUMBER);
  CONTROL_ID       = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, CONTROL_API_ID, DEVICE_NUMBER);

  Serial.print("STATUS_ID: 0x"); Serial.println(STATUS_ID, HEX);
  Serial.print("COLOR_SENSOR_ID: 0x"); Serial.println(COLOR_SENSOR_ID, HEX);
  Serial.print("CONTROL_ID: 0x"); Serial.println(CONTROL_ID, HEX);

  uint8_t id = read8(REG_PART_ID);
  Serial.print("Sensor ID: 0x"); Serial.println(id, HEX);
  if (id == 0xC2) {
    write8(REG_MAIN_CTRL, 0b00000111);
    sensorOK = true;
    Serial.println("Color sensor initialized.");
  } else {
    Serial.println("Color sensor not detected.");
  }

  xTaskCreate(TaskCANSend,      "CAN Send",      128, NULL, 1, NULL);
  xTaskCreate(TaskCANReceive,   "CAN Receive",   128, NULL, 2, NULL);
  xTaskCreate(TaskHeartbeatLED, "Heartbeat LED", 128, NULL, 1, NULL);
  xTaskCreate(TaskColorSensor,  "Color Sensor",  256, NULL, 1, NULL);
}

void loop() {}

void TaskCANSend(void* pvParameters) {
  for (;;) {
    byte btn = 0;
    for (int i = 0; i < 3; i++) {
      if (!digitalRead(buttonPins[i])) btn |= (1 << i);
    }
    int analogVal = analogRead(analogPin);

    byte tx[8] = {0};
    tx[0] = btn;
    tx[1] = (analogVal >> 8) & 0xFF;
    tx[2] = analogVal & 0xFF;

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
        bool led0 = rx[0] & 0x01;
        bool led1 = rx[0] & 0x02;
        byte angle = rx[1];

        digitalWrite(ledPins[0], led0);
        digitalWrite(ledPins[1], led1);

        if (angle != prevAngle) {
          myServo.write(angle);
          prevAngle = angle;
        }
      }
    }
    vTaskDelay(1);
  }
}

void TaskHeartbeatLED(void* pvParameters) {
  for (;;) {
    unsigned long now = millis();
    bool connected = (now - heartbeat_last_seen) < 500;

    if (!connected) {
      digitalWrite(13, LOW);
      vTaskDelay(100);
    } else if (last_enabled) {
      digitalWrite(13, HIGH);
      vTaskDelay(100);
      digitalWrite(13, LOW);
      vTaskDelay(100);
    } else {
      digitalWrite(13, HIGH);
      vTaskDelay(200);
    }
  }
}

void TaskColorSensor(void* pvParameters) {
  uint8_t retryCounter = 0;
  for (;;) {
    byte tx[8] = {0};

    if (sensorOK) {
      uint32_t r  = read20(REG_RED_DATA);
      uint32_t g  = read20(REG_GREEN_DATA);
      uint32_t b  = read20(REG_BLUE_DATA);
      uint32_t ir = read20(REG_IR_DATA);
      uint8_t  p  = read8(REG_PROX_DATA);

      bool allZeroChan = (r == 0 && g == 0 && b == 0 && ir == 0 && p == 0);
      if (allZeroChan) {
        sensorOK = false;
        Serial.println("Sensor disconnected.");
        tx[0] = 0xFF;
      } else {
        r  = (r  > 65535UL ? 65535UL : r);
        g  = (g  > 65535UL ? 65535UL : g);
        b  = (b  > 65535UL ? 65535UL : b);
        ir = (ir > 65535UL ? 65535UL : ir);
        p  = (p  >    255   ?   255   : p);

        tx[0] = (r >> 8); tx[1] = r & 0xFF;
        tx[2] = (g >> 8); tx[3] = g & 0xFF;
        tx[4] = (b >> 8); tx[5] = b & 0xFF;
        tx[6] = p;
        tx[7] = (ir >> 8);

        retryCounter = 0;
      }
    } else {
      tx[0] = 0xFF;
      if (++retryCounter >= 10) {
        retryCounter = 0;
        if (read8(REG_PART_ID) == 0xC2) {
          write8(REG_MAIN_CTRL, 0b00000111);
          sensorOK = true;
          Serial.println("Sensor reconnected.");
        }
      }
    }

    CAN.sendMsgBuf(COLOR_SENSOR_ID, 1, 8, tx);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}