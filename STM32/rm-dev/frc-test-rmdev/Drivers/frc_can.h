#ifndef RMDEV_DRIVERS_FRC_CAN_H
#define RMDEV_DRIVERS_FRC_CAN_H

#include "can.h"
#include "imu.h"
#include "io.h"
#include <stdint.h>

/* FRC 29-bit identifier fields used by this Team Use device. */
#define FRC_CAN_DEVICE_TYPE_MISCELLANEOUS 0x0AU
#define FRC_CAN_MANUFACTURER_TEAM_USE     0x08U
#define FRC_CAN_DEFAULT_DEVICE_NUMBER     55U
#define FRC_CAN_MAX_DEVICE_NUMBER         63U

/* API class 0x1A was unused in the companion ESP32 firmware when assigned. */
#define FRC_CAN_API_IMU_ACCEL_TEMP 0x1A0U
#define FRC_CAN_API_IMU_GYRO_SEQ   0x1A1U
#define FRC_CAN_API_POWER          0x1A2U
#define FRC_CAN_IMU_FRAME_COUNT    2U

/* Load/save the runtime device number from the reserved configuration sector. */
void frc_can_config_init(void);
uint8_t frc_can_get_device_number(void);
uint8_t frc_can_get_saved_device_number(void);
bool frc_can_set_device_number(uint8_t device_number);
bool frc_can_save_device_number(void);

uint32_t frc_can_make_identifier(uint16_t api_id, uint8_t device_number);
void frc_can_build_imu_frames(const imu_sample_t *sample, uint16_t sequence,
                              can_frame_t frames[FRC_CAN_IMU_FRAME_COUNT]);
void frc_can_build_power_frame(const io_battery_sample_t *sample,
                               uint16_t uptime_seconds, can_frame_t *frame);

#endif /* RMDEV_DRIVERS_FRC_CAN_H */
