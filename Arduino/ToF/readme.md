# ToF Sensor VL53L1X

## 🔌 **1. I2C Wiring (VL53L1X)**

* **ESP32 I2C Pins**:

  * `SDA → GP5`
  * `SCL → GP4`

| ESP32   | VL53L1X |
| ------- | ------------- |
| GP5     | SDA           |
| GP4     | SCL           |
| 3.3V    | VIN           |
| GND     | GND           |

---

## 🧭 **2. XSHUT (Sensor Power Sequencing)**

The single sensor is enabled during boot through XSHUT.

| ESP32 GPIO | Sensor   | Connects to |
| ---------- | -------- | ----------- |
| GP6        | Sensor 0 | XSHUT pin   |

---

## 🛠️ **3. CAN Bus (TWAI on ESP32)**

ESP32 TWAI uses dedicated GPIOs for CAN TX/RX.

| ESP32 GPIO | Connects to                   |
| ---------- | ----------------------------- |
| GP9        | CAN **TX** / TWAI TX (to transceiver)   |
| GP10       | CAN **RX** / TWAI RX (from transceiver) |

---

## 💡 **4. Status LED**

One WS281x LED is connected to GP21 and uses FastLED at 50/255 brightness:

| Distance | LED behavior |
| -------- | ------------ |
| `0 mm` | Solid red (invalid/timeout reading) |
| `1–50 mm` | Red blink at 50 Hz |
| `51–999 mm` | Red blink rate scales linearly from 50 Hz down to 0 Hz |
| `≥1000 mm` | Off |

## 🔌 **5. USB serial (ESP32-S3 Zero)**

The sketch requires ESP32-S3 native USB CDC. In the Arduino IDE, select **Tools → USB CDC On Boot → Enabled** before compiling and uploading. `Serial` then outputs through the board's USB-C connector at 115200 baud.

## 🔢 **6. CAN device number**

The CAN device number is persisted in EEPROM and defaults to `50` when EEPROM is uninitialized. Send one of these newline-terminated commands through USB serial:

| Command | Result |
| ------- | ------ |
| `&CANID SET <0-63>` | Changes the running device number immediately. |
| `&CANID SAVE` | Persists the running device number and reboots the board. |
| `&CANID GET` | Reports the running, saved, and default device numbers. |

## 🔎 **7. I²C scanner**

[i2cScanner.ino](i2cScanner/i2cScanner.ino) is a standalone scanner for the same GP5 (SDA) and GP4 (SCL) bus. It reports detected I²C addresses over USB serial at 115200 baud every two seconds.

---

## 📡 CAN Protocol

### 📤 **Sensor → roboRIO** (ESP32 sends sensor status)

#### API ID: `0x0301` (`SENSOR_BASE_API_ID`)

* One frame for Sensor 0.

* **Payload (8 bytes)**:

```
Byte 0–1: Distance (uint16_t, mm)
Byte 2:   Ranging mode (0 = Short, 1 = Medium, 2 = Long)
Byte 3:   ROI center (uint8_t)
Byte 4:   ROI size X (width)
Byte 5:   ROI size Y (height)
Byte 6–7: Measurement timing budget (uint16_t, µs / 1e3 resolution)
```

> Note: Timing budget is truncated to 16 bits (up to \~65ms).

---

### 📥 **roboRIO → Sensor** (ESP32 receives sensor config)

#### API ID: `0x0305` (`SENSOR_CONFIG_API_ID`)

* One configuration frame for Sensor 0.

* **Payload (4 bytes)**:

```
Byte 0: Ranging mode (0 = Short, 1 = Medium, 2 = Long)
Byte 1: ROI center (uint8_t)
Byte 2: ROI size X (4–16, must be even)
Byte 3: ROI size Y (4–16, must be even)
```

Only one config frame should be sent per 200 ms to allow a safe I2C update delay.

---
