#include <cstdint>
#include "Arduino.h"

static uint64_t g_sim_time_us = 0;
static uint32_t g_loop_us = 1000;

uint32_t millis() {
  return static_cast<uint32_t>((g_sim_time_us / 1000ULL) & 0xFFFFFFFFu);
}

void delay(uint32_t ms) {
  g_sim_time_us += static_cast<uint64_t>(ms) * 1000ULL;
}

uint64_t sim_time_ms() {
  return g_sim_time_us / 1000ULL;
}

void sim_reset_time() {
  g_sim_time_us = 0;
}

void sim_step_loop() {
  g_sim_time_us += g_loop_us;
}

void sim_step_loops(uint32_t count) {
  g_sim_time_us += static_cast<uint64_t>(g_loop_us) * count;
}

void sim_set_loop_us(uint32_t loop_us) {
  g_loop_us = loop_us == 0 ? 1 : loop_us;
}

uint32_t sim_loop_us() {
  return g_loop_us;
}
