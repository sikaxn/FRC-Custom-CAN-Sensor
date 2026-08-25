# Battery Tracking + LED

This is work in progress

This firmware will combine all those function in the end. it is still in the work.

![image](../../drawing/BatteryLEDCombo_bb.png)

# Build LED only firmware

Modify DISABLE_RFID to true and build or upload.
```
const bool DISABLE_RFID = true; //Use this option to disable RFID if a LED only firmware build is needed.
```

# ESP32-S3 USB-C serial

The USB-C connector uses the ESP32-S3's direct USB CDC/JTAG interface. In Arduino
IDE, select **Tools > USB CDC On Boot > Enabled**. The standard **Hardware CDC and
JTAG** USB mode is supported and is the intended configuration.

The firmware caps the ESP32-S3 CPU clock at 80 MHz during startup.


# Please also read these documents

Battery Tracking: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/Arduino/Battery_Tracking

Historical LED-only firmware: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/Arduino/addressableLED

Battery Tracking Wiki: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/wiki/900.-IronMaple-RFID-Battery-Tracking-Solution

CD Thread 1: https://www.chiefdelphi.com/t/custom-can-sensor-esp32-development-board/505038

CD Thread 2: https://www.chiefdelphi.com/t/rfid-battery-tracking-progress-updates-video-demo-update/502847

CD Thread 3: https://www.chiefdelphi.com/t/custom-can-sensor-rfid-battery-tracking-led-controller-combo-firmware/510961

# Driver

https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/roboRIO/batteryReaderNewLEDCombo

# Pin Map

This firmware is mapped for the Waveshare ESP32-S3-Zero.

## Firmware pin assignment

| GPIO | Direction | Connected device | Notes |
| ---- | --------- | ---------------- | ----- |
| GPIO 21 | Output | Onboard WS281x RGB LED | Status / diagnostic LED (FastLED, RGB byte order, brightness 10/255) |
| GPIO 9 | Output | CAN transceiver TXD | TWAI / CAN TX |
| GPIO 10 | Input | CAN transceiver RXD | TWAI / CAN RX |
| GPIO 7 | Output | External WS2812 LED strip DIN | Main addressable LED strip |
| GPIO 11 | Output | RFID reader 1 SDA / SS | Reserved; RFID disabled by default |
| GPIO 12 | Output | RFID reader 2 SDA / SS | Reserved; RFID disabled by default |
| GPIO 13 | Output | RFID reader RST | Reserved; RFID disabled by default |
| GPIO 14 | Output | RFID SPI SCK | Reserved; RFID disabled by default |
| GPIO 15 | Input | RFID SPI MISO | Reserved; RFID disabled by default |
| GPIO 16 | Output | RFID SPI MOSI | Reserved; RFID disabled by default |
| GPIO 0 | Input | Onboard BOOT button | Cycles LED scenes on press (active low) |

## External wiring summary

### CAN transceiver

| ESP32-S3-Zero | Connect to |
| ------------- | ---------- |
| GPIO 9 | CAN transceiver TXD |
| GPIO 10 | CAN transceiver RXD |
| 3V3 or 5V | CAN transceiver power, depending on module |
| GND | CAN transceiver ground |

### External LED strip

| ESP32-S3-Zero | Connect to |
| ------------- | ---------- |
| GPIO 7 | LED strip DIN |
| 5V | LED strip power |
| GND | LED strip ground |

### RFID readers

Both RC522 readers share the same SPI bus and reset line. Each reader gets its own `SS` pin.

| ESP32-S3-Zero | Reader 1 | Reader 2 |
| ------------- | -------- | -------- |
| GPIO 11 | SS / SDA | - |
| GPIO 12 | - | SS / SDA |
| GPIO 13 | RST | RST |
| GPIO 14 | SCK | SCK |
| GPIO 15 | MISO | MISO |
| GPIO 16 | MOSI | MOSI |
| 3V3 | VCC | VCC |
| GND | GND | GND |

## Notes

- There is no relay in this hardware revision.
- There is no separate local blackout button in this hardware revision.
- Pressing GPIO 0 during normal operation cycles modes 0–34, then mode 255. Do
  not hold it while resetting the board, because it is also the ESP32-S3 boot pin.
- The LED output is thermally capped using the ESP32 internal sensor: 255 at
  55°C or below; 190 above 55°C; 128 above 65°C; 64 above 75°C; and 20 above
  80°C. Each change fades to its new cap over one second. CAN brightness
  commands still accept their normal 0–255 values.
- The 190 brightness cap above 55°C does not change the status LED. At higher
  temperatures, it alternates green/red at 4 Hz above 65°C, purple/red at 6 Hz
  above 75°C, and yellow/red at 7 Hz above 80°C. These rates do not overlap
  the RFID warning patterns.
- The onboard WS281x status LED uses `GPIO 21`; the external LED strip uses `GPIO 7`.
- The firmware uses a FreeRTOS task compatibility wrapper for ESP32-S3 builds.
- Make sure the LED strip power ground, CAN transceiver ground, and ESP32 ground are common.

# LED CAN Protocol

This section is the current CAN specification for this combined ESP32-S3
firmware. It supersedes the older LED-only firmware documentation.

All LED frames use the FRC extended 29-bit ID:

```text
CAN ID = (deviceType << 24) | (manufacturer << 16) | (apiId << 6) | deviceNumber
```

| Field | Value |
| --- | --- |
| Device type | `0x0A` |
| Manufacturer | `0x08` |
| Device number | Configurable, `0–63` |
| General command API | `0x350` |
| Custom-pixel APIs | `0x351–0x358` |
| System-core feedback API | `0x359` |
| Extended LED command API | `0x360` |

Commands must use the configured device number. The firmware publishes its
system-core feedback frame every 100 ms (10 Hz).

## General LED command (`0x350`)

| Byte | Field | Range / meaning |
| ---: | --- | --- |
| 0 | Mode | `0–255` |
| 1 | Red | `0–255` |
| 2 | Green | `0–255` |
| 3 | Blue | `0–255` |
| 4 | Brightness | `0–255`; subject to the thermal output cap |
| 5 | On/off | `1` on, `0` off |
| 6 | Param0 | Effect speed / option 1 |
| 7 | Param1 | Effect-specific option 2 |

## Custom pixel write (`0x351–0x358`)

Set mode `255` first. Each API ID may carry one pixel update.

| Bytes | Field | Meaning |
| --- | --- | --- |
| 0–1 | Pixel index | Big-endian index |
| 2 | Red | `0–255` |
| 3 | Green | `0–255` |
| 4 | Blue | `0–255` |
| 5–7 | Reserved | Set to `0` |

## Extended LED command (`0x360`)

| Bytes | Field | Meaning |
| --- | --- | --- |
| 0–1 | Active pixel count | Big-endian; clamped to the compiled maximum |
| 2–4 | Color 2 R/G/B | Secondary effect color |
| 5 | Color 2 brightness | `0–255` |
| 6 | Color 2 enable | `1` on, `0` off |
| 7 | Reboot request | Any nonzero value restarts the ESP32 |

## System-core feedback (`0x359`)

| Bytes | Field | Meaning |
| --- | --- | --- |
| 0–1 | Active LED count | Big-endian |
| 2 | Current mode | Current `canMode` |
| 3–5 | Reserved | Always `0` |
| 6 | Thermal protection stage | `0`: no cap; `1`: 190 cap (>55°C); `2`: 128 cap (>65°C); `3`: 64 cap (>75°C); `4`: 20 cap (>80°C) |
| 7 | ESP32 internal temperature | Unsigned encoding of whole °C: `encoded = clamp(round(tempC), -10, 100) + 10`; decode with `byte7 - 10` |

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
