# ToF Sensor VL53L1X

## 🔌 **1. I2C Wiring (VL53L1X)**

* **ESP32 I2C Pins**:

  * `SDA → GPIO 21`
  * `SCL → GPIO 22`
* **Shared among all sensors**:

  * All VL53L1X sensors connect to **the same SDA/SCL lines**.

| ESP32   | VL53L1X (all) |
| ------- | ------------- |
| GPIO 21 | SDA           |
| GPIO 22 | SCL           |
| 3.3V    | VIN           |
| GND     | GND           |

---

## 🧭 **2. XSHUT (Sensor Power Sequencing)**

Used to assign unique I2C addresses during boot.

| ESP32 GPIO | Sensor   | Connects to |
| ---------- | -------- | ----------- |
| GPIO 16    | Sensor 0 | XSHUT pin   |
| GPIO 17    | Sensor 1 | XSHUT pin   |
| GPIO 18    | Sensor 2 | XSHUT pin   |
| GPIO 19    | Sensor 3 | XSHUT pin   |

> All other pins of the sensors can remain connected to shared I2C.

---

## 🛠️ **3. CAN Bus (TWAI on ESP32)**

ESP32 TWAI uses dedicated GPIOs for CAN TX/RX.

| ESP32 GPIO | Connects to                   |
| ---------- | ----------------------------- |
| GPIO 4     | CAN **TX** (to transceiver)   |
| GPIO 5     | CAN **RX** (from transceiver) |

---

## 📡 CAN Protocol

### 📤 **Sensor → roboRIO** (ESP32 sends sensor status)

#### API ID Range: `0x0301` to `0x0304` (`SENSOR_BASE_API_ID + i`)

* One frame per sensor (i = 0–3)

* **API IDs**:

  * Sensor 0: `0x0301`
  * Sensor 1: `0x0302`
  * Sensor 2: `0x0303`
  * Sensor 3: `0x0304`

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

#### API ID Range: `0x0305` to `0x0308` (`SENSOR_CONFIG_API_ID + i`)

* One config frame per sensor

* **API IDs**:

  * Sensor 0: `0x0305`
  * Sensor 1: `0x0306`
  * Sensor 2: `0x0307`
  * Sensor 3: `0x0308`

* **Payload (4 bytes)**:

```
Byte 0: Ranging mode (0 = Short, 1 = Medium, 2 = Long)
Byte 1: ROI center (uint8_t)
Byte 2: ROI size X (4–16, must be even)
Byte 3: ROI size Y (4–16, must be even)
```

Only one config frame should be sent per 200 ms per sensor to allow safe I2C update delay.

---
