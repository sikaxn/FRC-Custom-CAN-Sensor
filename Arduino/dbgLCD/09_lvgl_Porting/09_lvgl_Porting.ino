#include <Arduino.h>
#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "lvgl_v8_port.h"
#include <demos/lv_demos.h>

#include "waveshare_twai_port.h"
#include <set>
#include <tuple>

#include "esp_heap_caps.h"
#include "device_debug_ui.h"

//extern std::map<DeviceKey, std::vector<MessageEntry>> deviceMessages;


using namespace esp_panel::drivers;
using namespace esp_panel::board;

static bool driver_installed = false; 

extern std::set<std::tuple<uint8_t, uint8_t, uint8_t>> uniqueDevices;
static std::set<std::tuple<uint8_t, uint8_t, uint8_t>> uiDeviceCache;
//extern std::map<DeviceKey, std::vector<MessageEntry>> deviceMessages;
//DeviceKey key = std::make_tuple(dev_type, mfr_id, dev_num)

//std::vector<lv_obj_t*> all_device_buttons;




lv_obj_t* ctrl_label;
lv_obj_t* debug_btn_container = nullptr;

lv_obj_t* slider;
lv_obj_t* list1;
lv_obj_t* currentButton = nullptr;
lv_obj_t* device_list;
lv_obj_t* clear_btn;
lv_obj_t* hb_box;
lv_obj_t* hb_label;
lv_obj_t* device_list_container = nullptr;
lv_obj_t* info_panel = nullptr;




DeviceKey* selectedDeviceKey = nullptr;
static DeviceKey lastRenderedKey = std::make_tuple(0xFF, 0xFF, 0xFF);  // Initially invalid



void setup()
{
    String title = "LVGL porting example";

    Serial.begin(115200);

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();

    #if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
    #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
        auto lcd_bus = lcd->getBus();
        /**
        * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
        * "bounce buffer" functionality to enhance the RGB data bandwidth.
        * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
        */
        if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
        }
    #endif
    #endif
    assert(board->begin());

    Serial.println("Initializing TWAI");
    driver_installed = waveshare_twai_init();

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

// === DEVICE LIST PANEL (LEFT SIDE) ===
device_list = lv_list_create(lv_scr_act());
lv_obj_set_size(device_list, 200, 400);
lv_obj_align(device_list, LV_ALIGN_TOP_LEFT, 10, 10);
lv_list_add_text(device_list, "CAN Devices:");

// === INFO PANEL (RIGHT SIDE: LABEL + DEBUG BUTTONS) ===
lv_obj_t* info_panel = lv_obj_create(lv_scr_act());
lv_obj_set_size(info_panel, 580, 400);  // Full remaining width
lv_obj_set_flex_flow(info_panel, LV_FLEX_FLOW_COLUMN);
lv_obj_set_style_pad_row(info_panel, 12, 0);
lv_obj_set_style_pad_all(info_panel, 10, 0);
//lv_obj_clear_flag(info_panel, LV_OBJ_FLAG_SCROLLABLE);

// Align to the right of the device list
lv_obj_align_to(info_panel, device_list, LV_ALIGN_OUT_RIGHT_TOP, 10, 0);

// === Device Info Label ===
ctrl_label = lv_label_create(info_panel);
lv_label_set_text(ctrl_label, "Hi HPP, please select a device from the left.");
lv_label_set_long_mode(ctrl_label, LV_LABEL_LONG_WRAP);
lv_obj_set_width(ctrl_label, lv_obj_get_width(info_panel));
lv_obj_set_style_text_font(ctrl_label, &lv_font_montserrat_16, 0);

// === Debug Button Container ===
debug_btn_container = lv_obj_create(info_panel);
lv_obj_set_size(debug_btn_container, LV_PCT(100), 180);  // Fixed height, full width
lv_obj_set_flex_flow(debug_btn_container, LV_FLEX_FLOW_ROW_WRAP);
lv_obj_set_scrollbar_mode(debug_btn_container, LV_SCROLLBAR_MODE_AUTO);
lv_obj_set_style_pad_all(debug_btn_container, 6, 0);
lv_obj_set_style_pad_row(debug_btn_container, 6, 0);
lv_obj_set_style_pad_column(debug_btn_container, 6, 0);
//lv_obj_clear_flag(debug_btn_container, LV_OBJ_FLAG_SCROLLABLE);

// === CLEAR BUTTON (Below device list) ===
clear_btn = lv_btn_create(lv_scr_act());
lv_obj_set_size(clear_btn, 200, 40);
lv_obj_align_to(clear_btn, device_list, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

lv_obj_t* btn_label = lv_label_create(clear_btn);
lv_label_set_text(btn_label, "Clear and Rescan");
lv_obj_center(btn_label);

lv_obj_set_style_bg_color(clear_btn, lv_color_hex(0xD32F2F), 0);
lv_obj_set_style_text_color(clear_btn, lv_color_white(), 0);
// === HEARTBEAT BOX (Below the Info Panel) ===
hb_box = lv_obj_create(lv_scr_act());
lv_obj_set_size(hb_box, 580, 40);                         // fixed height to match Clear button
lv_obj_align_to(hb_box, info_panel, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
lv_obj_clear_flag(hb_box, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_set_style_pad_all(hb_box, 10, 0);                  // inner padding for label

// === HEARTBEAT LABEL ===
hb_label = lv_label_create(hb_box);
lv_label_set_long_mode(hb_label, LV_LABEL_LONG_WRAP);     // multiline
lv_obj_set_width(hb_label, lv_obj_get_width(hb_box) - 20); // account for 10 px padding on both sides
lv_obj_align(hb_label, LV_ALIGN_TOP_LEFT, 0, 0);
lv_label_set_text(hb_label, "Waiting for heartbeat...");





    lv_obj_add_event_cb(clear_btn, on_clear_btn_pressed, LV_EVENT_CLICKED, NULL);

    xTaskCreatePinnedToCore(
    TaskCANRx,      // function
    "CAN_RX",       // name
    4096,           // stack
    nullptr,        // param
    1,              // priority
    nullptr,        // handle
    0               // run on core 0
    );

    //xTaskCreatePinnedToCore(
    //TaskMemoryGC,
    //"MemGC",
    //4096,
    //nullptr,
    //1,
    //nullptr,
    //1  // run on core 1 (if you want separation from CAN)
    //);

    lv_timer_create(checkHeartbeatTimeout, 500, nullptr);
    lv_timer_create(refresh_device_list_cb, 1000, NULL); // refresh every 1 sec
    lv_timer_create(refresh_selected_device_cb, 200, nullptr); 


    lvgl_port_unlock();
}

void on_clear_btn_pressed(lv_event_t* e) {
    if (!lvgl_port_lock(-1)) return;

    // Delete all buttons and free associated DeviceInfo memory
    for (lv_obj_t* btn : all_device_buttons) {
        DeviceInfo* info = (DeviceInfo*)lv_obj_get_user_data(btn);
        if (info) {
            lv_mem_free(info);
            lv_obj_set_user_data(btn, nullptr);
        }
        lv_obj_del(btn);
    }
    all_device_buttons.clear();

    // Clear device list UI and add back the header text
    if (device_list) {
        lv_obj_clean(device_list);
        lv_list_add_text(device_list, "CAN Devices:");
    }

    // Clear debug button UI (important!)
    clear_debug_buttons();

    // Reset internal state
    uniqueDevices.clear();
    deviceState.clear();
    uiDeviceCache.clear();

    // 🧠 Clear selected key
    selectedDeviceKey = nullptr;

    // Reset the display label
    if (ctrl_label) {
        lv_label_set_text(ctrl_label, "Hi HPP, please select a device from the left.");
    }

    lvgl_port_unlock();

    Serial.println("[Clear] System reset complete.");
}




void refresh_device_list_cb(lv_timer_t * timer) {
    if (!device_list) return;
    if (!lvgl_port_lock(-1)) return;

    for (const auto& [dev_type, mfr_id, dev_num] : uniqueDevices) {
        auto key = std::make_tuple(dev_type, mfr_id, dev_num);
        if (uiDeviceCache.find(key) != uiDeviceCache.end()) {
            continue;  // Already displayed
        }

        // Build label
        const char * dev_type_name = dev_type < 32 ? DEVICE_TYPE_MAP[dev_type] : "Unknown";
        const char * mfr_name = mfr_id < 17 ? MANUFACTURER_MAP[mfr_id] : "Unknown";

        char label_str[64];
        snprintf(label_str, sizeof(label_str), "#%u [%s] %s", dev_num, mfr_name, dev_type_name);

        // Create a button in the list
        lv_obj_t* btn = lv_list_add_btn(device_list, LV_SYMBOL_BULLET, label_str);

        // Get the internal label created by the list button
        lv_obj_t* label = lv_obj_get_child(btn, 1);  // index 1: label (0 = icon)

        if (label && lv_obj_check_type(label, &lv_label_class)) {
            lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_obj_set_width(label, 160);  // force narrower than text
        }


        // Allocate memory for the device info
        DeviceInfo* devInfo = new DeviceInfo{dev_type, mfr_id, dev_num};
        lv_obj_set_user_data(btn, devInfo);


        // Add event callback
        lv_obj_add_event_cb(btn, on_device_btn_clicked, LV_EVENT_CLICKED, ctrl_label);


        // Mark as shown
        uiDeviceCache.insert(key);
        Serial.printf("→ Added to UI: %s\n", label_str);
    }

    lvgl_port_unlock();
}

void on_device_btn_clicked(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    DeviceInfo* info = static_cast<DeviceInfo*>(lv_obj_get_user_data(btn));
    if (!info) return;

    // Set the selected key
    static DeviceKey selectedKey;
    selectedKey = std::make_tuple(info->dev_type, info->mfr_id, info->dev_num);
    selectedDeviceKey = &selectedKey;

    // Force refresh UI immediately
    refresh_selected_device_cb(nullptr);
}




void loop()
{

    //waveshare_twai_receive();
}


void refresh_selected_device_cb(lv_timer_t*) {
    if (!selectedDeviceKey) return;
    if (!lvgl_port_lock(-1)) return;

    auto it = deviceState.find(*selectedDeviceKey);
    if (it == deviceState.end()) {
        lv_label_set_text(ctrl_label, "No messages found for selected device.");
        lvgl_port_unlock();
        return;
    }

    const auto& apimap = it->second.api_messages;
    const auto& [dev_type, mfr_id, dev_num] = *selectedDeviceKey;
    const char* dev_type_name = dev_type < 32 ? DEVICE_TYPE_MAP[dev_type] : "Unknown";
    const char* mfr_name = mfr_id < 17 ? MANUFACTURER_MAP[mfr_id] : "Unknown";

    // 🔒 Only create debug UI if the selected device changed
    if (*selectedDeviceKey != lastRenderedKey) {
        clear_debug_buttons();
        create_debug_buttons_if_known(debug_btn_container, mfr_id, dev_type, dev_num, apimap);
        lastRenderedKey = *selectedDeviceKey;
    }

    // 📟 Always update the message text
    std::string text;
    char header[128];
    snprintf(header, sizeof(header), "#%d [%s] %s\n", dev_num, mfr_name, dev_type_name);
    text += header;

    for (const auto& [api_id, data] : apimap) {
        char line[128];
        snprintf(line, sizeof(line), "API 0x%03X:", api_id);
        text += line;

        for (uint8_t b : data) {
            char byteStr[8];
            snprintf(byteStr, sizeof(byteStr), " %02X", b);
            text += byteStr;
        }
        text += "\n";
    }

    lv_label_set_text(ctrl_label, text.c_str());
    lvgl_port_unlock();
}
