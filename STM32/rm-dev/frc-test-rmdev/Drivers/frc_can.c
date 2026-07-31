#include "frc_can.h"

#include "stm32f4xx_hal.h"

#define FRC_CAN_CONFIG_COMMIT  0x46434944UL /* "FCID" */
#define FRC_CAN_CONFIG_VERSION 1UL
#define FRC_CAN_CONFIG_ERASED  0xFFFFFFFFUL

typedef struct {
    uint32_t commit;
    uint32_t sequence;
    uint32_t data;
    uint32_t crc32;
} frc_can_config_record_t;

typedef struct {
    const frc_can_config_record_t *first_erased;
    uint32_t newest_sequence;
    uint8_t newest_device_number;
    bool found_valid;
} frc_can_config_scan_t;

extern uint32_t __frc_can_config_start;
extern uint32_t __frc_can_config_end;

static volatile uint32_t active_device_number = FRC_CAN_DEFAULT_DEVICE_NUMBER;
static volatile uint32_t saved_device_number = FRC_CAN_DEFAULT_DEVICE_NUMBER;

static uint32_t config_data(uint8_t device_number)
{
    return (FRC_CAN_CONFIG_VERSION << 8U) | (uint32_t)device_number;
}

static uint32_t config_crc32(uint32_t sequence, uint32_t data)
{
    const uint32_t words[2] = {sequence, data};
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t word = 0U; word < 2U; ++word) {
        for (uint32_t byte = 0U; byte < 4U; ++byte) {
            crc ^= (words[word] >> (byte * 8U)) & 0xFFU;
            for (uint32_t bit = 0U; bit < 8U; ++bit) {
                const uint32_t mask = 0U - (crc & 1U);
                crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
            }
        }
    }

    return ~crc;
}

static bool config_record_erased(const frc_can_config_record_t *record)
{
    return record->commit == FRC_CAN_CONFIG_ERASED &&
           record->sequence == FRC_CAN_CONFIG_ERASED &&
           record->data == FRC_CAN_CONFIG_ERASED &&
           record->crc32 == FRC_CAN_CONFIG_ERASED;
}

static bool config_record_valid(const frc_can_config_record_t *record)
{
    const uint8_t device_number = (uint8_t)(record->data & 0xFFU);
    return record->commit == FRC_CAN_CONFIG_COMMIT &&
           device_number <= FRC_CAN_MAX_DEVICE_NUMBER &&
           record->data == config_data(device_number) &&
           record->crc32 == config_crc32(record->sequence, record->data);
}

static void config_scan(frc_can_config_scan_t *scan)
{
    uintptr_t address = (uintptr_t)&__frc_can_config_start;
    const uintptr_t end = (uintptr_t)&__frc_can_config_end;

    scan->first_erased = 0;
    scan->newest_sequence = 0U;
    scan->newest_device_number = FRC_CAN_DEFAULT_DEVICE_NUMBER;
    scan->found_valid = false;

    while ((address + sizeof(frc_can_config_record_t)) <= end) {
        const frc_can_config_record_t *record =
            (const frc_can_config_record_t *)address;
        if ((scan->first_erased == 0) && config_record_erased(record)) {
            scan->first_erased = record;
        }
        if (config_record_valid(record) &&
            (!scan->found_valid ||
             (int32_t)(record->sequence - scan->newest_sequence) > 0)) {
            scan->newest_sequence = record->sequence;
            scan->newest_device_number = (uint8_t)(record->data & 0xFFU);
            scan->found_valid = true;
        }
        address += sizeof(frc_can_config_record_t);
    }
}

void frc_can_config_init(void)
{
    frc_can_config_scan_t scan;
    config_scan(&scan);
    saved_device_number = scan.found_valid
        ? scan.newest_device_number
        : FRC_CAN_DEFAULT_DEVICE_NUMBER;
    active_device_number = saved_device_number;
}

uint8_t frc_can_get_device_number(void)
{
    return (uint8_t)active_device_number;
}

uint8_t frc_can_get_saved_device_number(void)
{
    return (uint8_t)saved_device_number;
}

bool frc_can_set_device_number(uint8_t device_number)
{
    if (device_number > FRC_CAN_MAX_DEVICE_NUMBER) return false;
    active_device_number = device_number;
    return true;
}

bool frc_can_save_device_number(void)
{
    frc_can_config_scan_t scan;
    const uint8_t device_number = frc_can_get_device_number();
    uint32_t sequence;
    uint32_t data;
    uint32_t address;
    bool success = false;

    if (device_number > FRC_CAN_MAX_DEVICE_NUMBER) return false;
    config_scan(&scan);
    if (scan.first_erased == 0) return false;

    sequence = scan.found_valid ? scan.newest_sequence + 1U : 1U;
    if (sequence == 0U) sequence = 1U;
    data = config_data(device_number);
    address = (uint32_t)(uintptr_t)scan.first_erased;

    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 4U, sequence) == HAL_OK &&
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 8U, data) == HAL_OK &&
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 12U,
                          config_crc32(sequence, data)) == HAL_OK &&
        /* Commit last: a brownout leaves the previous record valid. */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
                          FRC_CAN_CONFIG_COMMIT) == HAL_OK) {
        const frc_can_config_record_t *record =
            (const frc_can_config_record_t *)address;
        success = config_record_valid(record) &&
                  (uint8_t)(record->data & 0xFFU) == device_number;
    }

    (void)HAL_FLASH_Lock();
    if (success) saved_device_number = device_number;
    return success;
}

static void pack_i16_le(uint8_t *destination, int16_t value)
{
    const uint16_t bits = (uint16_t)value;
    destination[0] = (uint8_t)bits;
    destination[1] = (uint8_t)(bits >> 8U);
}

static void pack_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

uint32_t frc_can_make_identifier(uint16_t api_id, uint8_t device_number)
{
    return ((uint32_t)(FRC_CAN_DEVICE_TYPE_MISCELLANEOUS & 0x1FU) << 24U) |
           ((uint32_t)FRC_CAN_MANUFACTURER_TEAM_USE << 16U) |
           ((uint32_t)(api_id & 0x03FFU) << 6U) |
           ((uint32_t)device_number & 0x3FU);
}

void frc_can_build_imu_frames(const imu_sample_t *sample, uint16_t sequence,
                              can_frame_t frames[FRC_CAN_IMU_FRAME_COUNT])
{
    if ((sample == 0) || (frames == 0)) return;
    const uint8_t device_number = frc_can_get_device_number();

    frames[0].bus = CAN_BUS_1;
    frames[0].identifier = frc_can_make_identifier(
        FRC_CAN_API_IMU_ACCEL_TEMP, device_number);
    frames[0].extended = true;
    frames[0].length = 8U;
    pack_i16_le(&frames[0].data[0], sample->accel_raw[0]);
    pack_i16_le(&frames[0].data[2], sample->accel_raw[1]);
    pack_i16_le(&frames[0].data[4], sample->accel_raw[2]);
    pack_i16_le(&frames[0].data[6], sample->temperature_centi_c);

    frames[1].bus = CAN_BUS_1;
    frames[1].identifier = frc_can_make_identifier(
        FRC_CAN_API_IMU_GYRO_SEQ, device_number);
    frames[1].extended = true;
    frames[1].length = 8U;
    pack_i16_le(&frames[1].data[0], sample->gyro_raw[0]);
    pack_i16_le(&frames[1].data[2], sample->gyro_raw[1]);
    pack_i16_le(&frames[1].data[4], sample->gyro_raw[2]);
    pack_u16_le(&frames[1].data[6], sequence);
}

void frc_can_build_power_frame(const io_battery_sample_t *sample,
                               uint16_t uptime_seconds, can_frame_t *frame)
{
    if ((sample == 0) || (frame == 0)) return;

    const uint16_t input_mv = sample->input_mv > 0xFFFFUL
        ? 0xFFFFU
        : (uint16_t)sample->input_mv;

    frame->bus = CAN_BUS_1;
    frame->identifier = frc_can_make_identifier(
        FRC_CAN_API_POWER, frc_can_get_device_number());
    frame->extended = true;
    frame->length = 8U;
    pack_u16_le(&frame->data[0], input_mv);
    pack_u16_le(&frame->data[2], sample->battery_adc_raw);
    pack_u16_le(&frame->data[4], sample->vref_adc_raw);
    pack_u16_le(&frame->data[6], uptime_seconds);
}
