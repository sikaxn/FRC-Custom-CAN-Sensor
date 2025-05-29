#include <SPI.h>
#include <mcp_can.h>
#include <LiquidCrystal.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

// === Constants ===
#define CAN_CS 10
#define LCD_COLS 20
#define LCD_ROWS 4

#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define DEVICE_NUMBER    33
#define LCD_API_ID       0x187

#define CMD_CLEAR       0x01
#define CMD_CURSOR      0x02
#define CMD_WRITE       0x03
#define CMD_WRITE_NEXT  0x04

// === LCD & CAN ===
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
MCP_CAN CAN(CAN_CS);

// === Spinner Animation ===
byte spinner[8][8] = {
  {0b00100, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
  {0b00100, 0b00010, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
  {0b00000, 0b00001, 0b00010, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000},
  {0b00000, 0b00000, 0b00000, 0b00001, 0b00010, 0b00100, 0b00000, 0b00000},
  {0b00000, 0b00000, 0b00000, 0b00000, 0b00100, 0b00100, 0b00000, 0b00000},
  {0b00000, 0b00000, 0b00000, 0b00100, 0b00010, 0b00001, 0b00000, 0b00000},
  {0b00000, 0b00000, 0b00000, 0b00010, 0b00001, 0b00100, 0b00000, 0b00000},
  {0b00000, 0b00000, 0b00100, 0b00010, 0b00001, 0b00000, 0b00000, 0b00000}
};

// === State ===
volatile int cursorX = 0, cursorY = 0;
volatile bool lcdUpdated = false;
unsigned long lastUpdateTime = 0;
int spinnerFrame = 0;

QueueHandle_t lcdQueue;

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

void processLCDMessage(byte *data, byte len) {
  byte cmd = data[0];
  byte x = data[1];
  byte y = data[2];

  switch (cmd) {
    case CMD_CLEAR:
      lcd.clear();
      break;
    case CMD_CURSOR:
      cursorX = x;
      cursorY = y;
      lcd.setCursor(cursorX, cursorY);
      break;
    case CMD_WRITE:
      cursorX = x;
      cursorY = y;
      lcd.setCursor(cursorX, cursorY);
      for (int i = 3; i < len; i++) {
        lcd.write(data[i]);
        cursorX++;
      }
      break;
    case CMD_WRITE_NEXT:
      lcd.setCursor(cursorX, cursorY);
      for (int i = 3; i < len; i++) {
        lcd.write(data[i]);
        cursorX++;
      }
      break;
  }

  lcdUpdated = true;
  lastUpdateTime = millis();
}

void TaskCANReceiver(void *pvParameters) {
  for (;;) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      long unsigned int canId;
      byte len = 0;
      byte buf[8];
      CAN.readMsgBuf(&canId, &len, buf);

      uint32_t targetId = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, LCD_API_ID, DEVICE_NUMBER);
      if (canId == targetId) {
        byte *msg = (byte *)pvPortMalloc(len);
        memcpy(msg, buf, len);
        xQueueSend(lcdQueue, &msg, portMAX_DELAY);
      }
    }
    vTaskDelay(1);
  }
}

void TaskLCDUpdater(void *pvParameters) {
  for (;;) {
    byte *msg;
    if (xQueueReceive(lcdQueue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
      processLCDMessage(msg, 8);
      vPortFree(msg);
    }

    // Spinner animation while updating
    if (lcdUpdated && millis() - lastUpdateTime < 1000) {
      lcd.createChar(7, spinner[spinnerFrame]);
      lcd.setCursor(LCD_COLS - 1, 0);
      lcd.write(byte(7));
      spinnerFrame = (spinnerFrame + 1) % 8;
    } else {
      lcd.setCursor(LCD_COLS - 1, 0);
      lcd.print(" "); // clear spinner
    }

    vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}

void setup() {
  lcd.begin(LCD_COLS, LCD_ROWS);
  for (int i = 0; i < 8; i++) lcd.createChar(i, spinner[i]);
  lcd.print("I/M CAN");
  delay(1000);
  lcd.clear();

  lcdQueue = xQueueCreate(4, sizeof(byte *));
  Serial.begin(115200);

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    lcd.print("CAN ready");
    CAN.setMode(MCP_NORMAL);
  } else {
    lcd.print("CAN FAIL");
    while (1);
  }

  xTaskCreate(TaskCANReceiver, "CAN Receiver", 256, NULL, 2, NULL);
  xTaskCreate(TaskLCDUpdater, "LCD Updater", 256, NULL, 1, NULL);
}

void loop() {
  // Empty, FreeRTOS handles tasks
}
