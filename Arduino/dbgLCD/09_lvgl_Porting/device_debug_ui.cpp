#include "device_debug_ui.h"
#include "waveshare_twai_port.h"
#include <string>
#include <stdio.h>
#include <Arduino.h>

struct SliderContext {
    lv_obj_t* slider;
    lv_obj_t* value_label;
};

// Callback for slider value update
void slider_event_cb(lv_event_t* e) {
    SliderContext* ctx = (SliderContext*)lv_event_get_user_data(e);
    if (ctx && ctx->slider && ctx->value_label) {
        int val = lv_slider_get_value(ctx->slider);
        lv_label_set_text_fmt(ctx->value_label, "%02d", val);
    }
}

// Callback for Send button
void send_led_command_cb(lv_event_t* e) {
    SliderContext** contexts = (SliderContext**)lv_event_get_user_data(e);
    if (!contexts) return;

    uint8_t data[8];
    for (int i = 0; i < 8; ++i) {
        data[i] = lv_slider_get_value(contexts[i]->slider);
    }

    Serial.printf("[DEBUG UI] Sending LED command: [%d,%d,%d,%d,%d,%d,%d,%d]\n",
                  data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

    uint32_t can_id = (0x0A << 24) | (0x08 << 16) | (0x350 << 6) | (0x21 & 0x3F);
    sendCANMessage(can_id, data, 8);
}

void create_debug_buttons_if_known(lv_obj_t* container, uint8_t mfr_id, uint8_t dev_type, uint8_t dev_num, const std::map<uint16_t, std::vector<uint8_t>>& api_messages) {
    if (!container) return;

    bool matched = false;

    for (const auto& [api_id, data] : api_messages) {
        char key[64];
        snprintf(key, sizeof(key), "mfr=0x%02X dev=0x%02X num=0x%02X api=0x%03X", mfr_id, dev_type, dev_num, api_id);
        Serial.printf("Creating debug UI for: %s\n", key);

        if (strcmp(key, "mfr=0x08 dev=0x0A num=0x21 api=0x359") == 0) {
            matched = true;

            const char* labels[8] = {
                "Mode", "Red", "Green", "Blue", "Brightness", "On/Off", "Param0", "Param1"
            };

            // Ensure container is vertically scrollable
            lv_obj_set_scroll_dir(container, LV_DIR_VER);
            lv_obj_add_flag(container, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_ACTIVE);
            lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(container, 6, 0);
            lv_obj_set_style_pad_row(container, 8, 0);

            // Allocate slider context array
            SliderContext** slider_contexts = (SliderContext**)lv_mem_alloc(sizeof(SliderContext*) * 8);

            for (int i = 0; i < 8; ++i) {
                slider_contexts[i] = (SliderContext*)lv_mem_alloc(sizeof(SliderContext));

                lv_obj_t* row = lv_obj_create(container);
                lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
                lv_obj_set_style_pad_column(row, 8, 0);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t* name = lv_label_create(row);
                lv_label_set_text_fmt(name, "%s", labels[i]);
                lv_obj_set_width(name, 70);

                slider_contexts[i]->slider = lv_slider_create(row);
                lv_slider_set_range(slider_contexts[i]->slider, 0, (i == 5 ? 1 : 255));
                lv_slider_set_value(slider_contexts[i]->slider, (i == 5 ? 1 : 0), LV_ANIM_OFF);
                lv_obj_set_width(slider_contexts[i]->slider, 240);

                slider_contexts[i]->value_label = lv_label_create(row);
                lv_label_set_text_fmt(slider_contexts[i]->value_label, "%02d", lv_slider_get_value(slider_contexts[i]->slider));
                lv_obj_set_width(slider_contexts[i]->value_label, 40);

                lv_obj_add_event_cb(slider_contexts[i]->slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, slider_contexts[i]);
            }

            // Add Send button
            lv_obj_t* btn = lv_btn_create(container);
            lv_obj_set_size(btn, 160, 50);
            lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
            lv_obj_t* btn_label = lv_label_create(btn);
            lv_label_set_text(btn_label, "Send LED Cmd");
            lv_obj_center(btn_label);

            lv_obj_add_event_cb(btn, send_led_command_cb, LV_EVENT_CLICKED, slider_contexts);

            break;
        }
    }

    if (!matched) {
        Serial.println("[DEBUG UI] No match. Showing default message.");
        lv_obj_t* info = lv_label_create(container);
        lv_label_set_text(info, "No debug features available for this device.");
        lv_obj_set_style_text_color(info, lv_color_hex(0x888888), 0);
        lv_obj_set_width(info, LV_PCT(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
    }
}

void clear_debug_buttons() {
    if (debug_btn_container && lv_obj_get_child_cnt(debug_btn_container) > 0) {
        lv_obj_clean(debug_btn_container);
    }
}
