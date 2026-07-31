#ifndef RMDEV_DRIVERS_IMU_H
#define RMDEV_DRIVERS_IMU_H

#include <stdbool.h>
#include <stdint.h>

/* BMI088 uses SPI1 at up to 10 MHz; IST8310 uses I2C3 at address 0x0E. */
#define IMU_ACCEL_RANGE_G   3
#define IMU_GYRO_RANGE_DPS  2000

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_centi_c;
} imu_sample_t;

bool imu_init(uint32_t cpu_and_timer_clock_hz);
bool imu_read_sample(imu_sample_t *sample);
bool imu_set_heater_duty(uint8_t duty_percent);
bool imu_read_temperature_c(float *temperature_c);
void imu_heater_update(float temperature_c);
bool magnetometer_init(uint32_t apb1_clock_hz);

#endif /* RMDEV_DRIVERS_IMU_H */
