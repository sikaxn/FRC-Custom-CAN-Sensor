#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>

// === CAN and Pins ===
#define CAN_CS_PIN 53
#define CONTROL_ID 0x0A086400
#define STATUS_ID  0x0A086000
#define HEARTBEAT_ID 0x01011840

MCP_CAN CAN(CAN_CS_PIN);
Servo myServo;

// Pins
const int buttonPins[3] = {3, 4, 5};
const int ledPins[2] = {6, 7};
const int analogPin = A0;
const int servoPin = 9;

// State
bool prevLed0 = false;
bool prevLed1 = false;
bool prevEnabled = false;
int prevAngle = -1;

volatile bool last_enabled = false;
volatile unsigned long heartbeat_last_seen = 0;

// === Tasks ===
void TaskCANSend(void* pvParameters);
void TaskCANReceive(void* pvParameters);
void TaskHeartbeatLED(void* pvParameters);

// === Setup ===
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  for (int i = 0; i < 2; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(13, OUTPUT);  // Dedicated heartbeat LED
  myServo.attach(servoPin);
  myServo.write(90);

  // Init CAN @ 1 Mbps, 8 MHz crystal
  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed...");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN initialized.");

  // Start RTOS tasks
  xTaskCreate(TaskCANSend, "CAN Send", 128, NULL, 1, NULL);
  xTaskCreate(TaskCANReceive, "CAN Receive", 128, NULL, 2, NULL);
  xTaskCreate(TaskHeartbeatLED, "Heartbeat LED", 128, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS runs all tasks
}

// === Send status every 100 ms ===
void TaskCANSend(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    byte btn = 0;
    for (int i = 0; i < 3; i++) {
      if (!digitalRead(buttonPins[i])) {
        btn |= (1 << i);
      }
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

// === Handle received CAN frames ===
void TaskCANReceive(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      long unsigned int rxId;
      byte len;
      byte rx[8];
      CAN.readMsgBuf(&rxId, &len, rx);

      rxId &= 0x1FFFFFFF;

      // Heartbeat from roboRIO
      if (rxId == HEARTBEAT_ID && len == 8) {
        heartbeat_last_seen = millis();
        byte mode = rx[4];  // Bit 4 = Enabled
        last_enabled = (mode >> 4) & 0x01;
      }

      // Control command from roboRIO
      if (rxId == CONTROL_ID && len >= 2) {
        byte ledByte = rx[0];
        byte angle = rx[1];

        bool led0 = ledByte & 0x01;
        bool led1 = ledByte & 0x02;

        digitalWrite(ledPins[0], led0 ? HIGH : LOW);
        digitalWrite(ledPins[1], led1 ? HIGH : LOW);

        if (led0 != prevLed0) {
          Serial.print("LED0: "); Serial.println(led0 ? "ON" : "OFF");
          prevLed0 = led0;
        }

        if (led1 != prevLed1) {
          Serial.print("LED1: "); Serial.println(led1 ? "ON" : "OFF");
          prevLed1 = led1;
        }

        if (angle != prevAngle) {
          myServo.write(angle);
          prevAngle = angle;
        }
      }
    }
    vTaskDelay(1);
  }
}

// === Control built-in LED (pin 13) based on heartbeat ===
void TaskHeartbeatLED(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    unsigned long now = millis();
    bool active = (now - heartbeat_last_seen) < 500;

    if (!active) {
      digitalWrite(13, LOW);  // Not connected
      vTaskDelay(100 / portTICK_PERIOD_MS);
    } else if (last_enabled) {
      digitalWrite(13, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(13, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    } else {
      digitalWrite(13, HIGH);  // Connected but disabled
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}
