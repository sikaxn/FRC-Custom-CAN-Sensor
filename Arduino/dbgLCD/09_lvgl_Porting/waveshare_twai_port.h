#ifndef __TWAI_PORT_H
#define __TWAI_PORT_H

#pragma once

#include <Arduino.h>
#include "driver/twai.h"
#include <esp_io_expander.hpp>
#include "lvgl.h"  // Needed for lv_timer_t


// Pins used to connect to CAN bus transceiver:
#define RX_PIN 19
#define TX_PIN 20

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define EXAMPLE_I2C_ADDR    (ESP_IO_EXPANDER_I2C_CH422G_ADDRESS)
#define EXAMPLE_I2C_SDA_PIN 8         // I2C data line pins
#define EXAMPLE_I2C_SCL_PIN 9         // I2C clock line pin


// Intervall:
#define POLLING_RATE_MS 1000

bool waveshare_twai_init();
void waveshare_twai_receive();
void TaskCANRx(void *pvParams);

bool decodeFRCHeartbeat(uint8_t* data, char* out, size_t len);
void checkHeartbeatTimeout(lv_timer_t*);


// Device lookup tables
extern const char * DEVICE_TYPE_MAP[32];
extern const char * MANUFACTURER_MAP[17];
extern int64_t last_hb_us;




#endif