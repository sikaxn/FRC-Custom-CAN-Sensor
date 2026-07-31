#ifndef RMDEV_DRIVERS_PWM_H
#define RMDEV_DRIVERS_PWM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PWM_SERVO_1 = 0, PWM_SERVO_2, PWM_SERVO_3, PWM_SERVO_4,
    PWM_SERVO_5, PWM_SERVO_6, PWM_SERVO_7
} pwm_servo_t;

/* Configure the seven connector outputs for standard 50 Hz servo pulses. */
bool pwm_servo_init(uint32_t apb2_timer_clock_hz);
bool pwm_servo_set_pulse_us(pwm_servo_t channel, uint16_t pulse_us);

#endif /* RMDEV_DRIVERS_PWM_H */
