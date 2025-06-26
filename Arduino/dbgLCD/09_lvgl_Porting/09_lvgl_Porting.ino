#include <Arduino.h>
#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "lvgl_v8_port.h"
#include <demos/lv_demos.h>

#include "waveshare_twai_port.h"
#include <set>
#include <tuple>


using namespace esp_panel::drivers;
using namespace esp_panel::board;

static bool driver_installed = false; 

extern std::set<std::tuple<uint8_t, uint8_t, uint8_t>> uniqueDevices;
static std::set<std::tuple<uint8_t, uint8_t, uint8_t>> uiDeviceCache;




lv_obj_t* ctrl_label;
lv_obj_t* slider;
lv_obj_t* list1;
lv_obj_t* currentButton = nullptr;
lv_obj_t* device_list;
lv_obj_t* clear_btn;
lv_obj_t* hb_box;
lv_obj_t* hb_label;




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

    // Create the device list on the left
    device_list = lv_list_create(lv_scr_act());
    lv_obj_set_size(device_list, 200, 400);
    lv_obj_align(device_list, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_list_add_text(device_list, "CAN Devices:");

    // Create the label next to the device list
    ctrl_label = lv_label_create(lv_scr_act());
    lv_label_set_text(ctrl_label, "Hi HPP, please select a device from the left.");
    lv_obj_set_style_text_font(ctrl_label, &lv_font_montserrat_16, 0);


    // Align ctrl_label to the right of device_list
    lv_obj_align_to(ctrl_label, device_list, LV_ALIGN_OUT_RIGHT_TOP, 20, 0);  // 20px margin to the right

        
    // Create CLEAR button below the list
    clear_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(clear_btn, 200, 40);
    lv_obj_align_to(clear_btn, device_list, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t* btn_label = lv_label_create(clear_btn);
    lv_label_set_text(btn_label, "Clear and Rescan");
    lv_obj_center(btn_label);

    // Set red style
    lv_obj_set_style_bg_color(clear_btn, lv_color_hex(0xD32F2F), 0);
    lv_obj_set_style_text_color(clear_btn, lv_color_white(), 0);

    // Add callback
    

    // Heartbeat status box
    hb_box = lv_obj_create(lv_scr_act());
    lv_obj_set_size(hb_box, 300, 100);
    lv_obj_align(hb_box, LV_ALIGN_BOTTOM_RIGHT, -10, -10);

    hb_label = lv_label_create(hb_box);
    lv_label_set_text(hb_label, "Waiting for heartbeat...");
    lv_label_set_long_mode(hb_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hb_label, 200);
    lv_obj_align(hb_label, LV_ALIGN_TOP_LEFT, 5, 5);


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
    lv_timer_create(checkHeartbeatTimeout, 500, nullptr);
    lv_timer_create(refresh_device_list_cb, 1000, NULL); // refresh every 1 sec
    

    lvgl_port_unlock();
}

void on_clear_btn_pressed(lv_event_t * e) {
    Serial.println("CLEAR pressed!");

    // Lock before modifying UI
    if (!lvgl_port_lock(-1)) return;

    // Clear internal sets
    uniqueDevices.clear();
    uiDeviceCache.clear();

    // Remove old device_list and recreate
    lv_obj_del(device_list);
    device_list = lv_list_create(lv_scr_act());
    lv_obj_set_size(device_list, 200, 400);
    lv_obj_align(device_list, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_list_add_text(device_list, "CAN Devices:");

    // Move clear button back under new list
    lv_obj_align_to(clear_btn, device_list, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // Reset main label
    if (ctrl_label) {
        lv_label_set_text(ctrl_label, "Hi HPP, please select a device from the left.");
    }

    lvgl_port_unlock();
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
        auto* devInfo = new std::tuple<uint8_t, uint8_t, uint8_t>(dev_type, mfr_id, dev_num);
        lv_obj_set_user_data(btn, devInfo);  // C++ wrapper

        // Add event callback
        lv_obj_add_event_cb(btn, on_device_btn_clicked, LV_EVENT_CLICKED, ctrl_label);


        // Mark as shown
        uiDeviceCache.insert(key);
        Serial.printf("→ Added to UI: %s\n", label_str);
    }

    lvgl_port_unlock();
}

static void on_device_btn_clicked(lv_event_t * e) {
    lv_obj_t* btn = lv_event_get_target(e);
    lv_obj_t* label = static_cast<lv_obj_t*>(lv_event_get_user_data(e));

    auto* devInfo = static_cast<std::tuple<uint8_t, uint8_t, uint8_t>*>(lv_obj_get_user_data(btn));
    if (!devInfo) return;

    uint8_t dev_type, mfr_id, dev_num;
    std::tie(dev_type, mfr_id, dev_num) = *devInfo;

    const char * dev_type_name = dev_type < 32 ? DEVICE_TYPE_MAP[dev_type] : "Unknown";
    const char * mfr_name = mfr_id < 17 ? MANUFACTURER_MAP[mfr_id] : "Unknown";

    char detail[128];
    snprintf(detail, sizeof(detail),
             "Device #%u\nType: %s\nMfr: %s\nDevID: %u",
             dev_num, dev_type_name, mfr_name, dev_num);

    lv_label_set_text(label, detail);
}


void loop()
{
    //Serial.println("IDLE loop");
    waveshare_twai_receive();
}
