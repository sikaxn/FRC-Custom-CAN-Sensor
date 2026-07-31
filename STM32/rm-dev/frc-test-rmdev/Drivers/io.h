#ifndef RMDEV_DRIVERS_IO_H
#define RMDEV_DRIVERS_IO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t battery_adc_raw;
    uint16_t vref_adc_raw;
    uint32_t input_mv;
} io_battery_sample_t;

/* Basic board I/O: RGB status LED, user button, laser 5 V switch, and ADC. */
void io_init(void);
void io_led_set(uint8_t red, uint8_t green, uint8_t blue);
bool io_user_button_pressed(void);
void io_laser_5v_set(bool enabled);
bool io_battery_voltage_read(io_battery_sample_t *sample);

#endif /* RMDEV_DRIVERS_IO_H */
