#include "waveshare_twai_port.h"
#include <set>
#include <tuple>
#include "lvgl_v8_port.h"
#include <lvgl.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

std::set<std::tuple<uint8_t, uint8_t, uint8_t>> uniqueDevices;
extern lv_obj_t* hb_label;
extern lv_obj_t* hb_box;
int64_t last_hb_us = 0;
#define HEARTBEAT_ID 0x01011840



void extractCANFields(uint32_t id, uint8_t &dev_type, uint8_t &mfr_id, uint8_t &dev_num) {
    dev_type = (id >> 24) & 0x1F;
    mfr_id   = (id >> 16) & 0xFF;
    dev_num  = id & 0x3F;
}



// ——— FRC 29-bit lookup tables ———
const char * DEVICE_TYPE_MAP[32] = {
    "Broadcast","Robot Controller","Motor Controller","Relay Controller",
    "Gyro Sensor","Accelerometer","Ultrasonic Sensor","Gear Tooth Sensor",
    "Power Distribution Module","Pneumatics Controller","Miscellaneous",
    "IO Breakout","Servo Controller","Unknown","Unknown","Unknown",
    "Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown",
    "Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Firmware Update"
};

const char * MANUFACTURER_MAP[17] = {
    "Broadcast","NI","Luminary Micro","DEKA","CTR Electronics",
    "REV Robotics","Grapple","MindSensors","Team Use","Kauai Labs",
    "Copperforge","Playing With Fusion","Studica","The Thrifty Bot",
    "Redux Robotics","AndyMark","Vivid Hosting"
};

// ——— Internal: decode & print one message ———
static void handle_rx_message(twai_message_t &msg) {
    uint32_t id  = msg.identifier;
    uint8_t  dlc = msg.data_length_code;

    // Always print full ID + DLC + data bytes
    //Serial.printf("ID(EXT)=0x%08X DLC=%u Data:", id, dlc);
    for(int i = 0; i < dlc; i++) {
        //Serial.printf(" %02X", msg.data[i]);
    }
    //Serial.println();

    // Decode as FRC only if extended
    if(msg.extd) {
        uint8_t  dev_type = (id >> 24) & 0xFF;
        uint8_t  mfr_id   = (id >> 16) & 0xFF;
        uint16_t api_id   = (id >>  6) & 0x3FF;
        uint8_t  dev_num  =  id        & 0x3F;

        const char * dev_type_name = dev_type < 32 ? DEVICE_TYPE_MAP[dev_type] : "Unknown";
        const char * mfr_name      = mfr_id   < 17 ? MANUFACTURER_MAP[mfr_id] : "Unknown";

        //Serial.printf(
        //  "  → FRC: DevType=%u(%s) Mfr=%u(%s) API=%u Dev#=%u\n",
        //  dev_type, dev_type_name,
        //  mfr_id,   mfr_name,
        //  api_id,
        //  dev_num
        //);
    }
}

// ——— Public: initialize the TWAI driver ———
bool waveshare_twai_init() {
    // 1 Mb/s, normal (ACK) mode, accept all IDs
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)TX_PIN,
      (gpio_num_t)RX_PIN,
      TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if(twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("TWAI install failed");
        return false;
    }
    if(twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }
    Serial.println("TWAI running at 1 Mb/s NORMAL");
    return true;
}

// ——— Public: pull in any pending frames once ———
void waveshare_twai_receive() {
    twai_message_t msg;
    // Block up to POLLING_RATE_MS for the first frame
    if(twai_receive(&msg, pdMS_TO_TICKS(POLLING_RATE_MS)) == ESP_OK) {
        handle_rx_message(msg);
        // Drain remaining immediately
        while(twai_receive(&msg, 0) == ESP_OK) {
            handle_rx_message(msg);
        }
    }
}

// ——— Task: continuously receive frames asynchronously ———
void TaskCANRx(void *pvParams) {
    twai_message_t msg;
    Serial.println("[TaskCANRx] started");
    
    for (;;) {
        //Serial.println("[TaskCANRx] loop");
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK && msg.extd) {
            uint8_t dev_type, mfr_id, dev_num;
            extractCANFields(msg.identifier, dev_type, mfr_id, dev_num);
            auto key = std::make_tuple(dev_type, mfr_id, dev_num);

            if (uniqueDevices.insert(key).second) {
                const char * dev_type_name = dev_type < 32 ? DEVICE_TYPE_MAP[dev_type] : "Unknown";
                const char * mfr_name = mfr_id < 17 ? MANUFACTURER_MAP[mfr_id] : "Unknown";

                Serial.printf("[New CAN Device] DevType=%u (%s), Mfr=%u (%s), Dev#=%u\n",
                              dev_type, dev_type_name,
                              mfr_id, mfr_name,
                              dev_num);
            }
            if (msg.identifier == HEARTBEAT_ID && msg.data_length_code == 8) {
                char buf[128];
                bool enabled = decodeFRCHeartbeat(msg.data, buf, sizeof(buf));
                last_hb_us = esp_timer_get_time();

                //Serial.println(">> Heartbeat detected");

                if (lvgl_port_lock(-1)) {
                    if (hb_label) {
                        lv_label_set_text(hb_label, buf);
                    }

                    if (hb_box) {
                        lv_color_t bg = enabled ? lv_color_hex(0x43A047) : lv_color_hex(0xE53935);
                        lv_obj_set_style_bg_color(hb_box, bg, 0);
                    }

                    lvgl_port_unlock();
                }
            }



        }
    }
}


void checkHeartbeatTimeout(lv_timer_t*) {
    int64_t now = esp_timer_get_time();
    if ((now - last_hb_us) > 1'000'000) { // 1 second = 1,000,000 us
        if (lvgl_port_lock(-1)) {
            if (hb_box) {
                lv_obj_set_style_bg_color(hb_box, lv_color_hex(0x888888), 0);
            }
            if (hb_label) {
                lv_label_set_text(hb_label, "Waiting for heartbeat...");
            }
            lvgl_port_unlock();
        }
    }
    
}


uint64_t get_bits(const char* bits, int start, int len) {
    uint64_t val = 0;
    for (int i = 0; i < len; ++i) {
        if (bits[start + i] == '1') {
            val |= (1ULL << (len - 1 - i));
        }
    }
    return val;
}

bool decodeFRCHeartbeat(uint8_t* data, char* out, size_t len) {
    // Construct LSB-first bit string
    char bits[65] = {0};
    for (int i = 0; i < 8; ++i) {
        for (int b = 0; b < 8; ++b) {
            bits[i * 8 + b] = ((data[i] >> (7 - b)) & 1) ? '1' : '0';
        }
    }

    uint8_t  match_time      = get_bits(bits, 56, 8);
    uint16_t match_number    = get_bits(bits, 46, 10);
    uint8_t  replay_number   = get_bits(bits, 40, 6);
    bool     red_alliance    = get_bits(bits, 39, 1);
    bool     enabled         = get_bits(bits, 38, 1);
    bool     autonomous      = get_bits(bits, 37, 1);
    bool     test            = get_bits(bits, 36, 1);
    bool     watchdog        = get_bits(bits, 35, 1);
    uint8_t  tournament_type = get_bits(bits, 32, 3);
    int      year            = 2000 + get_bits(bits, 26, 6) - 36;
    int      month           = get_bits(bits, 22, 4) + 1;
    int      day             = get_bits(bits, 17, 5);
    int      seconds         = get_bits(bits, 11, 6);
    int      minutes         = get_bits(bits, 5, 6);
    int      hours           = get_bits(bits, 0, 5);

    snprintf(out, len,
        "%04d-%02d-%02d %02d:%02d:%02d (UTC)\n "
        "%s | %s | %s\n"
        "Match %d R%d  %ds",
        year, month, day, hours, minutes, seconds,
        red_alliance ? "RED" : "BLUE",
        enabled ? "ENABLED" : "DISABLED",
        autonomous ? "AUTO" : "TELEOP",
        match_number,
        replay_number,
        match_time
    );

    return enabled;
}

