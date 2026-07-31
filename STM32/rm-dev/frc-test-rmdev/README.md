# RoboMaster Type C FRC CAN/IMU firmware

STM32Cube/FreeRTOS firmware for the RoboMaster Development Board Type C. It
initializes both on-board CAN transceivers at 1 Mbit/s, reads the BMI088 at
500 Hz, regulates its heater, exposes 100 Hz USB CDC telemetry, and publishes
the IMU on CAN1 at 100 Hz. A white RGB LED blink (500 ms on, 500 ms off) is the
scheduler-running indicator.

The board's USB and ST-Link connections can power the MCU for development, but
the 8-28 V main input is required for the IMU heater and other power outputs.

## Build

From this directory:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The ELF is written to `build/Debug/frc-test-rmdev.elf`. The project targets an
STM32F407 and uses the STM32Cube ARM GNU toolchain plus FreeRTOS.

## CAN connection

- Bus: board CAN1, the two-pin connector (`PD0/CAN1_RX`, `PD1/CAN1_TX`)
- Link: classic CAN 2.0, 1,000,000 bit/s
- FRC frames: 29-bit extended data frames, DLC 8
- CANalyst-II: `device=0`, `channel=0` (the adapter's CAN1)
- Termination: 120 ohms at each physical end of the bus

CAN2 is initialized at the same bitrate but does not carry the periodic IMU
frames. It remains available through the generic CAN driver/API.

## FRC identifier

The firmware follows the FRC 29-bit identifier layout used by the companion
ESP32 code:

```text
bits 28..24  device type     0x0A (Miscellaneous)
bits 23..16  manufacturer    0x08 (Team Use)
bits 15..6   API ID          10 bits
bits 5..0    device number   0-63 (default 55 / 0x37)

id = (device_type << 24) | (manufacturer << 16) |
     ((api_id & 0x3ff) << 6) | (device_number & 0x3f)
```

API IDs `0x1A0` through `0x1A2` use the previously unused API class `0x1A`.
They deliberately do not use `0x180` or `0x181`, which are reserved by other
devices in this system.

Default device-number IDs are shown below. Changing the device number changes
only bits 5..0 of each identifier.

| CAN ID (device 55) | API | Rate | Payload |
| --- | --- | --- | --- |
| `0x0A086837` | `0x1A0` | 100 Hz | accelerometer X/Y/Z and temperature |
| `0x0A086877` | `0x1A1` | 100 Hz | gyroscope X/Y/Z and sample sequence |
| `0x0A0868B7` | `0x1A2` | 1 Hz | external input voltage and ADC diagnostics |

All payloads are explicitly **little-endian**. Payload byte order is an
application protocol choice; the FRC identifier format does not define it.

### API `0x1A0`: accelerometer and temperature

| Bytes | Type | Field |
| --- | --- | --- |
| 0-1 | `int16` LE | accelerometer X raw |
| 2-3 | `int16` LE | accelerometer Y raw |
| 4-5 | `int16` LE | accelerometer Z raw |
| 6-7 | `int16` LE | temperature, centi-degrees Celsius |

### API `0x1A1`: gyroscope and sequence

| Bytes | Type | Field |
| --- | --- | --- |
| 0-1 | `int16` LE | gyroscope X raw |
| 2-3 | `int16` LE | gyroscope Y raw |
| 4-5 | `int16` LE | gyroscope Z raw |
| 6-7 | `uint16` LE | low 16 bits of the IMU sample sequence |

The two frames are queued back-to-back from the same latest IMU sample. The
sequence counts 500 Hz sensor reads and normally advances by about five per
100 Hz CAN update.

```text
acceleration_g   = accel_raw * 3 / 32768
angular_rate_dps = gyro_raw * 2000 / 32768
temperature_c    = temperature_centi / 100
```

### API `0x1A2`: power diagnostics

| Bytes | Type | Field |
| --- | --- | --- |
| 0-1 | `uint16` LE | external input, millivolts |
| 2-3 | `uint16` LE | battery-divider ADC raw |
| 4-5 | `uint16` LE | VREFINT ADC raw |
| 6-7 | `uint16` LE | uptime seconds, modulo 65,536 |

The ADC is sampled by one dedicated 1 Hz task and the same result is published
to CAN and USB. This avoids concurrent access to the ADC peripherals.

### CANalyst-II receive example

Install `python-can`, connect the firmware's CAN1 to the adapter's CAN1, then:

```python
import can

bus = can.Bus(interface="canalystii", device=0, channel=0, bitrate=1_000_000)
while True:
    message = bus.recv(1.0)
    if message and message.arbitration_id in (
        0x0A086837, 0x0A086877, 0x0A0868B7
    ):
        print(message)
```

## USB debug serial

USB CDC uses VID/PID `0483:5740` and streams IMU, health, and calibrated input
voltage records. See [tools/USB_IMU_PROTOCOL.md](tools/USB_IMU_PROTOCOL.md) for
the wire format.

The firmware also implements the newline-delimited command protocol used by
the repository's [webIDTool.html](../../../webIDTool.html) at nominal 115200
baud:

```text
&CANID GET
&CANID SET <0..63>
&CANID SAVE
```

`SET` changes the running CAN device number immediately but does not write
flash. `GET` reports the running, saved, and default values. `SAVE` appends the
running value to the persistent journal and reboots after acknowledging the
command. The default is 55 when the journal is empty or invalid. Configuration
uses reserved STM32 flash sector 11 (`0x080E0000-0x080FFFFF`) and supports
8,192 SAVE operations before maintenance erase is required. Ordinary
sector-based firmware programming preserves it, while a full-chip erase does
not. Telemetry pauses for five seconds after a recognized CANID command so the
configuration response remains visible in the web tool.

Install the UI dependencies and select either transport:

```powershell
py -3.12 -m pip install -r tools\requirements.txt
py -3.12 tools\imu_visualizer.py
```

With no source arguments, the Pygame startup screen offers:

- **Auto**, with the predicted one-time USB/CAN choice
- **Serial**, with auto-detect or selection from all discovered COM ports
- **CAN**, with an editable FRC device number from 0 through 63

Explicit options bypass the selector for scripts and unattended launches:

```powershell
py -3.12 tools\imu_visualizer.py --source auto
py -3.12 tools\imu_visualizer.py --source serial --port COM31
py -3.12 tools\imu_visualizer.py --source can --frc-device 42
```

Auto prefers a connected STM32 USB CDC device and otherwise opens CANalyst-II
device 0, channel 0 at 1 Mbit/s. It selects once at startup and is not runtime
failover. CAN source options include `--can-interface`, `--can-device`,
`--can-channel`, `--can-bitrate`, and `--frc-device`. `--source usb` remains an
alias for `--source serial`.

## Relevant source files

- `Inc/board_io.h`: complete Type C pin map
- `Drivers/can.c`: bxCAN1/CAN2 initialization and standard/extended transport
- `Drivers/frc_can.c`: FRC identifier and IMU payload encoding
- `Src/app_tasks.c`: FreeRTOS CAN RX/TX, IMU, USB, and LED tasks
