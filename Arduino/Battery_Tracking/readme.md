# Battery Tracking 

CD Thread: https://www.chiefdelphi.com/t/rfid-battery-tracking-some-progress/502847

Video demo: https://www.youtube.com/watch?v=rKZOKIWVOAg

![](../../img/batterytrax.JPG)

Developed with wpilib 2025 and the latest firmware avaliable at the time for REV PDH and CTRE PDP.

## Android Companion APP and roboRIO Driver


https://github.com/sikaxn/FRC-RFID-Battery-Reader/

roboRIO Driver is only needed if you want to read battery data, or if your Power Distro device can not be connected to CAN (PDP 2.0). Energy usage monitoring is only supported with PDP 1.0 or PDH as roboRIO can not provide total current data.

https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/main/roboRIO/batteryReader

## To be done

1. DONE Reading date and time from roboRIO Heartbeat 
2. DONE Auto discovery of CTRE PDP or REV PDH so no more requirement of having roboRIO Driver sending voltage and energy (unless using PDP 2.0)


## Arduino IDE Setup

This Library needs to be manually installed to handle NDEF encoding.

https://github.com/Seeed-Studio/Seeed_Arduino_NFC

these can be installed using arduino library manager

https://github.com/OSSLibraries/Arduino_MFRC522v2
https://github.com/bblanchon/ArduinoJson

## Example RFID Reader holder

https://makerworld.com/en/models/1504037-frc-battery-tracking-example-rfid-reader-holder#profileId-1573770

![](../../img/readerHolder.JPG)

## Logic

1. When ESP32 power up, it will scan on both reader (I have 2 reader attached assuming a battery can be installed in any direction), once a card is found, it will lock on with that reader.
2. once a reader is chosen, it will pull data from battery and send it to CAN.
A new usage record is created with voltage, date, etc all 0 because at this time roboRIO isn’t fully booted yet.
3. once a meaningful date message (roboRIO get its date time after attaching to DS) is received from roboRIO, it will update the date on that record
4. robot start being driven, tested, etc. ESP32 monitor voltage sent from roboRIO and record the lowest value
5. robot is disabled. disable will trigger a write. Latest date time, voltage energy is written to battery
6. robot is re enabled… and disabled. every time robot is disabled, that record is updated.
7. we are done with robot, battery removed from robot, and data could be pulled by android APP.

## Card and data format

Data is stored on Mifare Classic 1K card are NDEF formatted json. [Docs](JSON_Format.md)

## Wiring

![image](../../drawing/BatteryRFID_bb.png)


| Peripheral         | Function                   | ESP32 Pin   | Notes                           |
| ------------------ | -------------------------- | ----------- | ------------------------------- |
| **RC522 Reader 1** | SPI SS (Slave Select)      | **GPIO 5**  | Connected to `ss_pin1`          |
| **RC522 Reader 2** | SPI SS (Slave Select)      | **GPIO 4**  | Connected to `ss_pin2`          |
| **RC522 Shared**   | RST (Reset)                | **GPIO 22** | Used for both readers           |
| **SPI (Shared)**   | SCK (Clock)                | **GPIO 18** | Used for both readers           |
|                    | MISO (Master In Slave Out) | **GPIO 19** | Used for both readers           |
|                    | MOSI (Master Out Slave In) | **GPIO 23** | Used for both readers           |
| **CAN TX**         | Transmit Line              | **GPIO 16** | Connected to CAN transceiver TX |
| **CAN RX**         | Receive Line               | **GPIO 17** | Connected to CAN transceiver RX |

##  **CAN Message Types in Use**

#### 1. **Heartbeat (robot status)** — `CAN ID = 0x01011840` rio -> ESP32

* **Fixed ID** sent by roboRIO
* **8-byte payload**, where:

  * `data[4] & (1 << 4)` = robot enabled status (bit 4)

#### 2. **BATTERY\_STATUS\_API\_ID\_1** (`apiID = 0x135`) rio -> ESP32

* **ID** = calculated by `makeCANMsgID(...)`
* **7+ bytes** payload:

  ```
  Byte 0: year (offset from 2000, e.g., 25 for 2025)
  Byte 1: month
  Byte 2: day
  Byte 3: hour
  Byte 4: minute
  Byte 5: second
  Byte 6: voltage * 10 (e.g., 139 = 13.9V)
  ```

#### 3. **BATTERY\_STATUS\_API\_ID\_2** (`apiID = 0x136`) rio -> ESP32

* **2 bytes** payload:

  ```
  Byte 0–1: Energy (uint16_t)
  ```

#### 4. **RFID Meta Info**

* **API IDs:** `0x131`, `0x132`, `0x133`
* **Used to send metadata after tag read**

##### ➤ `0x131`: Battery Serial (first 8 bytes) ESP32 -> rio

```
Byte 0–7: ASCII characters of SN
```

#####  ➤ `0x132`: **Datetime + Voltage Info** ESP32 -> rio

```
Byte 0: Year (offset from 2000, e.g., 25 = 2025)
Byte 1: Month (1–12)
Byte 2: Day of Month (1–31)
Byte 3: Hour (0–23)
Byte 4: Minute (0–59)
Byte 5: Second (0–59)
Byte 6: Voltage × 10 (e.g., 13.9V → 139)
Byte 7: Reserved (set to 0)
```



##### ➤ `0x133`: Cycle count + note ESP32 -> rio

```
Byte 0–1: Cycle count (uint16_t)
Byte 2: Note type (0 = normal, 1 = practice, 3 = Scrap, 4 = Other)
```

