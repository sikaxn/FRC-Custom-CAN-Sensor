# Battery Tracking + LED

This is work in progress

This firmware will combine all those function in the end. it is still in the work.

![image](../../drawing/BatteryLEDCombo_bb.png)

# Build LED only firmware

Modify DISABLE_RFID to true and build or upload.
```
const bool DISABLE_RFID = true; //Use this option to disable RFID if a LED only firmware build is needed.
```


# Please also read these documents

Battery Tracking: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/Arduino/Battery_Tracking

Addressable LED: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/Arduino/addressableLED

Battery Tracking Wiki: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/wiki/900.-IronMaple-RFID-Battery-Tracking-Solution

CD Thread 1: https://www.chiefdelphi.com/t/custom-can-sensor-esp32-development-board/505038

CD Thread 2: https://www.chiefdelphi.com/t/rfid-battery-tracking-progress-updates-video-demo-update/502847

CD Thread 3: https://www.chiefdelphi.com/t/custom-can-sensor-rfid-battery-tracking-led-controller-combo-firmware/510961

# Driver

https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/roboRIO/batteryReaderNewLEDCombo

# Pin Map

This firmware is mapped for the Waveshare ESP32-C3-Zero.

## Firmware pin assignment

| GPIO | Direction | Connected device | Notes |
| ---- | --------- | ---------------- | ----- |
| GPIO 10 | Output | Onboard WS2812 RGB LED | Status / diagnostic LED |
| GPIO 8 | Output | CAN transceiver TXD | TWAI / CAN TX |
| GPIO 9 | Input | CAN transceiver RXD | TWAI / CAN RX |
| GPIO 7 | Output | External WS2812 LED strip DIN | Main addressable LED strip |
| GPIO 1 | Output | RFID reader 1 SDA / SS | RC522 chip select for reader 1 |
| GPIO 2 | Output | RFID reader 2 SDA / SS | RC522 chip select for reader 2 |
| GPIO 3 | Output | RFID reader RST | Shared reset for both RC522 readers |
| GPIO 4 | Output | RFID SPI SCK | Shared by both RC522 readers |
| GPIO 5 | Input | RFID SPI MISO | Shared by both RC522 readers |
| GPIO 6 | Output | RFID SPI MOSI | Shared by both RC522 readers |
| GPIO 0 | Input | Onboard BOOT button | Present on board, not used by firmware logic |

## External wiring summary

### CAN transceiver

| ESP32-C3-Zero | Connect to |
| ------------- | ---------- |
| GPIO 8 | CAN transceiver TXD |
| GPIO 9 | CAN transceiver RXD |
| 3V3 or 5V | CAN transceiver power, depending on module |
| GND | CAN transceiver ground |

### External LED strip

| ESP32-C3-Zero | Connect to |
| ------------- | ---------- |
| GPIO 7 | LED strip DIN |
| 5V | LED strip power |
| GND | LED strip ground |

### RFID readers

Both RC522 readers share the same SPI bus and reset line. Each reader gets its own `SS` pin.

| ESP32-C3-Zero | Reader 1 | Reader 2 |
| ------------- | -------- | -------- |
| GPIO 1 | SS / SDA | - |
| GPIO 2 | - | SS / SDA |
| GPIO 3 | RST | RST |
| GPIO 4 | SCK | SCK |
| GPIO 5 | MISO | MISO |
| GPIO 6 | MOSI | MOSI |
| 3V3 | VCC | VCC |
| GND | GND | GND |

## Notes

- There is no relay in this hardware revision.
- There is no separate local blackout button in this hardware revision.
- The onboard RGB status LED uses `GPIO 10`; the external LED strip uses `GPIO 7`.
- The firmware runs on the ESP32-C3 single-core FreeRTOS configuration.
- Make sure the LED strip power ground, CAN transceiver ground, and ESP32 ground are common.

# LED Modes

All color-based modes use `canR/canG/canB` with `canBrig` (global brightness). Modes 19–34 use color 2 (`canR2/canG2/canB2` with `canBrig2`), clamped to `canBrig`. If color 2 enable is 0, color 2 is treated as off. Unless noted, `param0` is speed (delay ms) and `param1` is length/spacing.


| Mode | Description |
| ---- | ----------- |
| 0 | Off |
| 1 | Solid color |
| 2 | Color wipe |
| 3 | Color wipe (reset on mode change) |
| 4 | Rainbow |
| 5 | Breathe (reset on mode change) |
| 6 | Breathe (no reset) |
| 7 | Fast blinking (reset on mode change) |
| 8 | Single-pixel wipe, moving block length = `param1 + 1` (reset) |
| 9 | Single-pixel wipe, moving block length = `param1 + 1` (no reset) |
| 10 | Single-pixel bounce, block length = `param1 + 1` (reset) |
| 11 | Single-pixel bounce, block length = `param1 + 1` (no reset) |
| 12 | Center wipe, block length = `param1 + 1` (reset) |
| 13 | Center wipe, block length = `param1 + 1` (no reset) |
| 14 | Center bounce, block length = `param1 + 1` (reset) |
| 15 | Center bounce, block length = `param1 + 1` (no reset) |
| 16 | Alternating blocks (no reset); `param1` = spacing; `param0` inverted (smaller = slower) |
| 17 | Alternating block fade (reset); `param1` = spacing |
| 18 | Alternating block fade (no reset); `param1` = spacing |
| 19 | Color wipe with color 2 |
| 20 | Color wipe (reset on mode change) with color 2 |
| 21 | Breathe (reset on mode change) with color 2 |
| 22 | Breathe (no reset) with color 2 |
| 23 | Fast blinking (reset on mode change) with color 2 |
| 24 | Single-pixel wipe, moving block length = `param1 + 1` (reset) with color 2 |
| 25 | Single-pixel wipe, moving block length = `param1 + 1` (no reset) with color 2 |
| 26 | Single-pixel bounce, block length = `param1 + 1` (reset) with color 2 |
| 27 | Single-pixel bounce, block length = `param1 + 1` (no reset) with color 2 |
| 28 | Center wipe, block length = `param1 + 1` (reset) with color 2 |
| 29 | Center wipe, block length = `param1 + 1` (no reset) with color 2 |
| 30 | Center bounce, block length = `param1 + 1` (reset) with color 2 |
| 31 | Center bounce, block length = `param1 + 1` (no reset) with color 2 |
| 32 | Alternating blocks (no reset); `param1` = spacing; `param0` inverted (smaller = slower) with color 2 |
| 33 | Alternating block fade (reset); `param1` = spacing with color 2 |
| 34 | Alternating block fade (no reset); `param1` = spacing with color 2 |
| 254 | Power-on default init sequence |
| 255 | Custom pixel write mode |
