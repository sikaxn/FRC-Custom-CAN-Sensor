#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// MCP2515 Pins
#define CAN_CS_PIN 53
#define CAN_INT_PIN 2

// FRC-style CAN IDs
#define CONTROL_ID 0x0A086400
#define STATUS_ID  0x0A086000

MCP_CAN CAN(CAN_CS_PIN);
Servo myServo;

// Pins
const int buttonPins[3] = {3, 4, 5};
const int ledPins[2] = {6, 7};
const int analogPin = A0;
const int servoPin = 9;

bool prevLed0 = false;
bool prevLed1 = false;

volatile bool canRxFlag = false;
SemaphoreHandle_t canSemaphore;

// RTOS task declarations
void TaskCANSend(void* pvParameters);
void TaskCANReceive(void* pvParameters);

// MCP2515 interrupt handler
void canISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  canRxFlag = true;
  xSemaphoreGiveFromISR(canSemaphore, &xHigherPriorityTaskWoken);
  // Arduino_FreeRTOS version of portYIELD_FROM_ISR takes no args
  portYIELD_FROM_ISR();
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  for (int i = 0; i < 2; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(CAN_INT_PIN, INPUT);

  myServo.attach(servoPin);
  myServo.write(90);

  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed...");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN initialized");

  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
  canSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(TaskCANSend, "CANSend", 128, NULL, 1, NULL);
  xTaskCreate(TaskCANReceive, "CANRecv", 128, NULL, 2, NULL);
}

void loop() {
  // FreeRTOS handles everything
}

// Periodic task to send status frame
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

// Task triggered by INT pin to read CAN frames
void TaskCANReceive(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (xSemaphoreTake(canSemaphore, portMAX_DELAY) == pdTRUE) {
      while (CAN.checkReceive() == CAN_MSGAVAIL) {
        long unsigned int rxId;
        byte len;
        byte rx[8];

        CAN.readMsgBuf(&rxId, &len, rx);

        if (rxId == CONTROL_ID && len >= 2) {
          byte ledByte = rx[0];
          byte angle = rx[1];

          bool led0 = ledByte & 0x01;
          bool led1 = ledByte & 0x02;

          digitalWrite(ledPins[0], led0 ? HIGH : LOW);
          digitalWrite(ledPins[1], led1 ? HIGH : LOW);
          digitalWrite(13, led0 ? HIGH : LOW);

          if (led0 != prevLed0) {
            Serial.print("LED0: "); Serial.println(led0 ? "ON" : "OFF");
            prevLed0 = led0;
          }
          if (led1 != prevLed1) {
            Serial.print("LED1: "); Serial.println(led1 ? "ON" : "OFF");
            prevLed1 = led1;
          }

          myServo.write(angle);
        }
      }
    }
  }
}
