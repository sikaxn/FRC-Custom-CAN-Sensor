# USB IMU telemetry protocol

The RoboMaster Type C firmware exposes a USB CDC ACM device with VID/PID
`0483:5740`. The wire format is ASCII CSV with `\r\n` line endings. Unknown
record types or protocol versions must be ignored.

## Records

```text
HELLO,1,RMDEV-C,BMI088,100,3,2000
IMU,1,<sequence>,<timestamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp_centi_c>
STAT,1,<uptime_ms>,<can1_rx>,<can2_rx>,<rx_drop>,<can_tx>,<can_tx_retry>,<can_tx_queue_drop>,<imu_ok>,<imu_err>
PWR,1,<timestamp_ms>,<input_mv>,<battery_adc_raw>,<vref_adc_raw>
```

- `HELLO` is repeated every five seconds so a newly opened host receives the
  protocol version, telemetry rate, accelerometer range in g, and gyroscope
  range in degrees/second.
- `IMU` is transmitted at 100 Hz. Acceleration and gyroscope fields are signed
  16-bit raw BMI088 counts. `sequence` counts the 500 Hz sensor reads, so it
  normally advances by five between USB frames.
- `STAT` is transmitted once per second and contains cumulative health
  counters.
- `PWR` follows `STAT` once per second. `input_mv` is the calibrated voltage at
  the board's XT30/main power input. The two raw 12-bit ADC fields expose
  PF10/ADC3_IN8 and ADC1/VREFINT for diagnostics.

## Conversion

For the version 1 `HELLO` values above:

```text
acceleration_g = accel_raw * 3 / 32768
angular_rate_dps = gyro_raw * 2000 / 32768
temperature_c = temp_centi_c / 100
external_input_v = input_mv / 1000
```

The Type C voltage input uses a 200 kOhm / 22 kOhm divider. Firmware averages
32 conversions from both the divider and VREFINT, then applies a per-board
calibration captured at 14.950 V (`battery_raw = 1783.750`,
`vref_raw = 1467.875`). The supported main input range is 8-28 V.

USB CDC is packetized and reads may split or combine records. Host software
must buffer bytes and only parse complete newline-terminated records.

## CAN device-number commands

The device accepts the protocol used by `webIDTool.html`. Commands are ASCII or
UTF-8 text terminated by LF; CRLF is also accepted. The nominal serial setting
is 115200, 8 data bits, no parity, and one stop bit.

```text
&CANID GET
&CANID SET <device_number>
&CANID SAVE
```

- `GET` reports the current runtime number, saved number, and default number.
- `SET` requires a decimal integer from 0 through 63 and takes effect on the
  next CAN publication. It does not modify flash.
- `SAVE` appends the current number to flash, acknowledges, waits approximately
  one second, and resets the MCU.

Responses follow the companion ESP32 firmware's text format so the existing
web tool can show them directly:

```text
[CANID] Current=55, EEPROM=55, Default=55
[CANID] Running DEVICE_NUMBER set to 42
[CANID] Invalid value. Must be 0-63.
[CANID] Saved to EEPROM. Rebooting...
[CANID] Save failed.
```

`EEPROM` is retained in the response for tool compatibility; the STM32 stores
records in an append-only journal in reserved flash sector 11. It has 8,192
slots and reports `Save failed` rather than erasing the sector after they are
used. Normal telemetry is paused for five seconds after each recognized command
so responses are not buried by the 100 Hz IMU stream.
