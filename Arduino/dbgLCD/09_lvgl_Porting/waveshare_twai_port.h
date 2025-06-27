#ifndef __TWAI_PORT_H
#define __TWAI_PORT_H


#pragma once
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <lvgl.h>

// Device key = (deviceType, manufacturerID, deviceNumber)
using DeviceKey = std::tuple<uint8_t, uint8_t, uint8_t>;


struct DeviceInfo {
    uint8_t dev_type;
    uint8_t mfr_id;
    uint8_t dev_num;
};

// Shared globals
//extern std::map<DeviceKey, std::vector<MessageEntry>> deviceMessages;
extern std::set<DeviceKey> uniqueDevices;
extern std::vector<lv_obj_t*> all_device_buttons;
extern int64_t last_hb_us;

struct CANDeviceData {
    uint32_t last_timestamp_us;
    std::map<uint16_t, std::vector<uint8_t>> api_messages;  // apiID → data[]
};
extern std::map<DeviceKey, CANDeviceData> deviceState;


// External UI elements
extern lv_obj_t* hb_label;
extern lv_obj_t* hb_box;
extern lv_obj_t* device_list_container;


// === Function Declarations ===
bool waveshare_twai_init();
void waveshare_twai_receive();
void TaskCANRx(void *pvParams);

bool decodeFRCHeartbeat(uint8_t* data, char* out, size_t len);
void checkHeartbeatTimeout(lv_timer_t*);
uint64_t get_bits(const char* bits, int start, int len);
void on_device_btn_clicked(lv_event_t* e);

bool sendCANMessage(uint32_t id, const uint8_t* data, uint8_t len, bool extended = true);


#define RX_PIN 19
#define TX_PIN 20
#define POLLING_RATE_MS 1000


// === CAN String Maps ===
extern const char * DEVICE_TYPE_MAP[32];
extern const char * MANUFACTURER_MAP[17];

#endif  // __TWAI_PORT_H
