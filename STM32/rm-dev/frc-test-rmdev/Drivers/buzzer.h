#ifndef RMDEV_DRIVERS_BUZZER_H
#define RMDEV_DRIVERS_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

/* Passive buzzer on TIM4_CH3 / PD14. Nominal frequency: 4 kHz. */
bool buzzer_init(uint32_t apb1_timer_clock_hz);
bool buzzer_set_tone(uint32_t frequency_hz, uint8_t duty_percent);
void buzzer_stop(void);

#endif /* RMDEV_DRIVERS_BUZZER_H */
