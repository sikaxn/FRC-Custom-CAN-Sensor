#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <cstdint>

#include "FastLED.h"

void runCurrentMode();

uint64_t sim_time_ms();
void sim_reset_time();
void sim_step_loop();
void sim_step_loops(uint32_t count);
void sim_set_loop_us(uint32_t loop_us);
uint32_t sim_loop_us();

void sim_set_num_leds(uint16_t count);
CRGB* sim_leds_data();
uint16_t sim_leds_count();
void sim_get_leds_copy(std::vector<CRGB>& out);
void sim_set_led(uint16_t index, const CRGB& color);
CRGB sim_get_led(uint16_t index);

extern volatile uint8_t canMode;
extern volatile uint8_t canR, canG, canB, canBrig;
extern volatile bool canOnOff;
extern volatile uint8_t canParam0, canParam1;
extern volatile uint8_t canR2, canG2, canB2, canBrig2, canOnOff2;
extern volatile bool modeRefresh;
extern volatile bool customSeen;
extern volatile uint16_t customPix;
extern volatile uint8_t cR, cG, cB;

namespace py = pybind11;

PYBIND11_MODULE(ledsim, m) {
  py::class_<CRGB>(m, "CRGB")
    .def(py::init<uint8_t, uint8_t, uint8_t>())
    .def_readwrite("r", &CRGB::r)
    .def_readwrite("g", &CRGB::g)
    .def_readwrite("b", &CRGB::b);

  m.def("set_num_leds", &sim_set_num_leds);
  m.def("led_count", &sim_leds_count);
  m.def("get_led", &sim_get_led);
  m.def("set_led", &sim_set_led);
  m.def("get_leds", [](){
    std::vector<CRGB> out;
    sim_get_leds_copy(out);
    return out;
  });

  m.def("run_current_mode", [](){
    runCurrentMode();
  });

  m.def("sim_time_ms", &sim_time_ms);
  m.def("sim_reset_time", &sim_reset_time);
  m.def("sim_step_loop", &sim_step_loop);
  m.def("sim_step_loops", &sim_step_loops);
  m.def("sim_set_loop_us", &sim_set_loop_us);
  m.def("sim_loop_us", &sim_loop_us);

  m.def("set_can_mode", [](uint8_t v){ canMode = v; });
  m.def("set_can_rgb", [](uint8_t r, uint8_t g, uint8_t b){ canR = r; canG = g; canB = b; });
  m.def("set_can_brightness", [](uint8_t v){ canBrig = v; });
  m.def("set_can_onoff", [](bool v){ canOnOff = v; });
  m.def("set_can_param0", [](uint8_t v){ canParam0 = v; });
  m.def("set_can_param1", [](uint8_t v){ canParam1 = v; });

  m.def("set_can2_rgb", [](uint8_t r, uint8_t g, uint8_t b){ canR2 = r; canG2 = g; canB2 = b; });
  m.def("set_can2_brightness", [](uint8_t v){ canBrig2 = v; });
  m.def("set_can2_onoff", [](uint8_t v){ canOnOff2 = v; });

  m.def("set_mode_refresh", [](bool v){ modeRefresh = v; });

  m.def("set_custom_pixel", [](uint16_t pix, uint8_t r, uint8_t g, uint8_t b){
    customPix = pix;
    cR = r;
    cG = g;
    cB = b;
    customSeen = true;
  });
}
