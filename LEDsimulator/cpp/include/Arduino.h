#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t millis();
void delay(uint32_t ms);

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifdef __cplusplus
}
#endif
