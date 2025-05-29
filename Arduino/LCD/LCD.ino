#include <SPI.h>
#include <mcp_can.h>
#include <LiquidCrystal.h>

#define CAN_CS 10
#define CAN_INT 21  // INT0 on Mega
MCP_CAN CAN(CAN_CS);
volatile bool canInterrupt = false;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#define LCD_COLS 20
#define LCD_ROWS 4

#define CMD_CLEAR       0x01
#define CMD_CURSOR      0x02
#define CMD_WRITE       0x03
#define CMD_WRITE_NEXT  0x04
#define CMD_ACK         0xAA  // Our new ACK command

#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define DEVICE_NUMBER    33
#define LCD_API_ID       0x187
#define ACK_API_ID       0x188

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) |
         ((uint32_t)(manufacturerID & 0xFF) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

uint32_t targetID, ackID;
int cursorX = 0, cursorY = 0;

void MCP2515_ISR() {
  canInterrupt = true;
}

void sendAck(uint8_t transactionID) {
  byte data[8] = {CMD_ACK, transactionID, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(ackID, 1, 8, data);  // Extended ID
  Serial.print("→ Sent ACK with transaction ID: ");
  Serial.println(transactionID);
}

void setup() {
  Serial.begin(115200);
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.print("LCD Ready");

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("CAN FAIL");
    while (1);
  }

  pinMode(CAN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT), MCP2515_ISR, FALLING);

  targetID = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, LCD_API_ID, DEVICE_NUMBER);
  ackID = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, ACK_API_ID, DEVICE_NUMBER);
}

void loop() {
if (canInterrupt) {
  canInterrupt = false;

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    long unsigned int canId;
    byte len = 0;
    byte buf[8];
    CAN.readMsgBuf(&canId, &len, buf);
    uint32_t cleanID = canId & 0x1FFFFFFF;

    if (cleanID == targetID) {
      Serial.print("[CAN RX] ID=0x");
      Serial.print(cleanID, HEX);
      Serial.print(" LEN=");
      Serial.print(len);
      Serial.print(" DATA=");
      for (int i = 0; i < len; i++) {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();

      byte cmd = buf[0];
      byte isFinal = buf[1];
      byte row = buf[2];
      byte col = buf[3];
      byte tid = buf[7];

      switch (cmd) {
        case CMD_CLEAR:
          lcd.clear();
          Serial.println("→ LCD Cleared");
          break;
        case CMD_CURSOR:
          cursorX = col;
          cursorY = row;
          lcd.setCursor(cursorX, cursorY);
          Serial.print("→ Cursor set: ");
          Serial.print(cursorX);
          Serial.print(", ");
          Serial.println(cursorY);
          break;
        case CMD_WRITE:
          cursorX = col;
          cursorY = row;
          lcd.setCursor(cursorX, cursorY);
          Serial.print("→ Write: ");
          for (int i = 4; i < 7; i++) {
            lcd.write(buf[i]);
            Serial.write(buf[i]);
            cursorX++;
          }
          Serial.println();
          break;
        case CMD_WRITE_NEXT:
          lcd.setCursor(cursorX, cursorY);
          Serial.print("→ Write next: ");
          for (int i = 4; i < 7; i++) {
            lcd.write(buf[i]);
            Serial.write(buf[i]);
            cursorX++;
          }
          Serial.println();
          if (isFinal) {
            sendAck(tid);
          }
          break;
        default:
          Serial.print("→ Unknown CMD: ");
          Serial.println(cmd, HEX);
          break;
      }
    }
  }
}}
