# Battery Tracking + LED

This is work in progress

This firmware will combine all those function in the end. it is still in the work.

![image](../../drawing/BatteryLEDCombo_bb.png)

# Driver

https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/roboRIO/batteryReaderNewLEDCombo

# Pinout

| Pin    | Function                                |
| ------ | --------------------------------------- |
| GPIO 4 | CAN TX (to CAN transceiver TXD)         |
| GPIO 5 | CAN RX (from CAN transceiver RXD)       |
| GPIO 16 | LED Strip Data Pin                     |
| GPIO 15 | RGB LED R (battery tracking diag light) |
| GPIO 13 | RGB LED G (battery tracking diag light) |
| GPIO 14 | RGB LED B (battery tracking diag light) |
| GPIO 32 | RC522 Reader 1 SS (ss_pin1)            |
| GPIO 33 | RC522 Reader 2 SS (ss_pin2)            |
| GPIO 22 | RC522 RST (shared between readers)     |
| GPIO 18 | SPI SCK (shared between readers)       |
| GPIO 19 | SPI MISO (shared between readers)      |
| GPIO 23 | SPI MOSI (shared between readers)      |
| GPIO 0  | Mode button (IO0, built-in; toggles mode for testing) |
| GPIO 17 | Relay toggle button (toggles canOnOff; off forces mode 0) |
| GPIO 25 | Relay                                  |
| 3.3V/5V | Power for CAN transceiver              |
| GND     | Common ground                          |
