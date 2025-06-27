#pragma once
#include <lvgl.h>
#include <map>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
extern lv_obj_t* debug_btn_container;

void create_debug_buttons_if_known(lv_obj_t* container, uint8_t mfr_id, uint8_t dev_type, uint8_t dev_num, const std::map<uint16_t, std::vector<uint8_t>>& api_messages);

void clear_debug_buttons();

#ifdef __cplusplus
}
#endif
