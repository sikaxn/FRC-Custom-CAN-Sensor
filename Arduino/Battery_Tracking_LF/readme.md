# CAN Protocol

| API ID | Direction | Purpose                                       | Payload                    |
| ------ | --------- | --------------------------------------------- | -------------------------- |
| `0x10` | Rio → ESP | Reboot request                                | `[0x01]` (one byte)        |
| `0x11` | ESP → Rio | First 8 bytes of tag serial                   | `[S0..S7]`                 |
| `0x12` | ESP → Rio | Last 6 bytes of tag serial + status + counter | `[S8..S13][status][count]` |


# GPIO

| **Symbol**    | **GPIO Pin**              | **Direction**       | **Function / Description**                                                                     |
| ------------- | ------------------------- | ------------------- | ---------------------------------------------------------------------------------------------- |
| `RFID_RX_PIN` | **GPIO 26**               | Input               | UART RX — receives data from **RDM6300** (reader TX → ESP32 RX)                                |
| `RFID_TX_PIN` | **GPIO 34**               | Output *(optional)* | UART TX — unused, safe to leave unconnected.                                                   |
| `LED_G`       | **GPIO 13**               | Output              | **Green LED** — indicates **tag present**                                                      |
| `LED_B`       | **GPIO 14**               | Output              | **Blue LED** — used for CAN-bus error blink                                                    |
| `LED_R`       | **GPIO 15**               | Output              | **Red LED** — combined with green to form **yellow** (“no tag”)                                |
| `CAN_TX_PIN`  | **GPIO 4** (`GPIO_NUM_4`) | Output              | **TWAI TX** — transmits CAN frames to bus via transceiver (e.g., TJA1051)                      |
| `CAN_RX_PIN`  | **GPIO 5** (`GPIO_NUM_5`) | Input               | **TWAI RX** — receives CAN frames from bus via transceiver                                     |
