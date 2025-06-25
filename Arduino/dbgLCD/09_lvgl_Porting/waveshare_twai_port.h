#ifndef __TWAI_PORT_H
#define __TWAI_PORT_H

#include <Arduino.h>
#include "driver/twai.h"
#include <lvgl.h>

// ESP32 GPIOs wired to your CAN transceiver
#define RX_PIN    19
#define TX_PIN    20

// Expander‐controlled lines (unchanged)
#define TP_RST    1
#define LCD_BL    2
#define LCD_RST   3
#define SD_CS     4
#define USB_SEL   5

// How long to block waiting for a frame (ms)
#define POLLING_RATE_MS 1000

// Initialize the TWAI driver (1 Mb/s, normal mode)
bool waveshare_twai_init();

// Call periodically or from a task to pull in any waiting frames
void waveshare_twai_receive();

// A FreeRTOS task entry that continuously blocks on twai_receive()
void TaskCANRx(void *pvParams);

// Device lookup tables
extern const char * DEVICE_TYPE_MAP[32];
extern const char * MANUFACTURER_MAP[17];

// Heartbeat decoder
bool decodeFRCHeartbeat(uint8_t* data, char* out, size_t len);

// Last heartbeat timestamp (microseconds)
extern int64_t last_hb_us;

// Heartbeat timeout checker
void checkHeartbeatTimeout(lv_timer_t*);

#endif // __TWAI_PORT_H
