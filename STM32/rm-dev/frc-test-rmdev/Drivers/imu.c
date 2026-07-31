#include "imu.h"

#include "board_io.h"

#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840UL)
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844UL)
#define SPI1_BASE 0x40013000UL
#define SPI_CR1 (*(volatile uint32_t *)(SPI1_BASE + 0x00UL))
#define SPI_SR (*(volatile uint32_t *)(SPI1_BASE + 0x08UL))
#define SPI_DR8 (*(volatile uint8_t *)(SPI1_BASE + 0x0CUL))
#define TIM10_BASE 0x40014400UL
#define I2C3_BASE 0x40005C00UL
#define GPIO_MODER(port) (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_OTYPER(port) (*(volatile uint32_t *)((port) + 0x04UL))
#define GPIO_AFRL(port) (*(volatile uint32_t *)((port) + 0x20UL))
#define GPIO_AFRH(port) (*(volatile uint32_t *)((port) + 0x24UL))
#define GPIO_BSRR(port) (*(volatile uint32_t *)((port) + 0x18UL))

#define CORE_DEBUG_DEMCR (*(volatile uint32_t *)0xE000EDFCUL)
#define DWT_CTRL         (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT       (*(volatile uint32_t *)0xE0001004UL)

#define BMI088_READ_BIT          0x80U
#define BMI088_ACCEL_CHIP_ID     0x1EU
#define BMI088_GYRO_CHIP_ID      0x0FU
#define BMI088_ACCEL_DATA_REG    0x12U
#define BMI088_ACCEL_TEMP_REG    0x22U
#define BMI088_ACCEL_CONF_REG    0x40U
#define BMI088_ACCEL_RANGE_REG   0x41U
#define BMI088_ACCEL_PWR_REG     0x7DU
#define BMI088_GYRO_DATA_REG     0x02U
#define BMI088_GYRO_RANGE_REG    0x0FU
#define BMI088_GYRO_BW_REG       0x10U
#define BMI088_GYRO_LPM1_REG     0x11U
#define BMI088_GYRO_RESET_REG    0x14U

#define SPI_TIMEOUT 100000U

static bool imu_ready;
static uint32_t imu_clock_hz;

static void alternate(uint32_t port, uint8_t pin, uint8_t af, bool open_drain)
{
    const uint32_t af_shift = (pin % 8U) * 4U;
    volatile uint32_t *const afr = (pin < 8U) ? &GPIO_AFRL(port) : &GPIO_AFRH(port);
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    if (open_drain) {
        GPIO_OTYPER(port) |= BOARD_PIN_MASK(pin);
    } else {
        GPIO_OTYPER(port) &= ~BOARD_PIN_MASK(pin);
    }
    *afr = (*afr & ~(0xFUL << af_shift)) | ((uint32_t)af << af_shift);
}

static void output(uint32_t port, uint8_t pin, bool high)
{
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3UL << (pin * 2U))) | (1UL << (pin * 2U));
    GPIO_BSRR(port) = high ? BOARD_PIN_MASK(pin) : (BOARD_PIN_MASK(pin) << 16U);
}

static void chip_select(uint32_t port, uint8_t pin, bool selected)
{
    GPIO_BSRR(port) = selected ?
        (BOARD_PIN_MASK(pin) << 16U) :
        BOARD_PIN_MASK(pin);
}

static bool spi_transfer(uint8_t value, uint8_t *received)
{
    uint32_t timeout = SPI_TIMEOUT;
    while (((SPI_SR & (1UL << 1)) == 0U) && (--timeout != 0U)) {}
    if (timeout == 0U) return false;

    SPI_DR8 = value;
    timeout = SPI_TIMEOUT;
    while (((SPI_SR & 1UL) == 0U) && (--timeout != 0U)) {}
    if (timeout == 0U) return false;

    const uint8_t value_read = SPI_DR8;
    if (received != 0) {
        *received = value_read;
    }
    return true;
}

static bool spi_wait_idle(void)
{
    uint32_t timeout = SPI_TIMEOUT;
    while (((SPI_SR & (1UL << 7)) != 0U) && (--timeout != 0U)) {}
    return timeout != 0U;
}

static void delay_us(uint32_t microseconds)
{
    const uint32_t cycles_per_us = imu_clock_hz / 1000000UL;
    const uint32_t start = DWT_CYCCNT;
    const uint32_t cycles = cycles_per_us * microseconds;
    while ((uint32_t)(DWT_CYCCNT - start) < cycles) {}
}

static void delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_us(1000U);
    }
}

static bool accel_write(uint8_t reg, uint8_t value)
{
    uint8_t ignored;
    chip_select(BOARD_IMU_ACCEL_CS_PORT, BOARD_IMU_ACCEL_CS_PIN, true);
    const bool ok = spi_transfer((uint8_t)(reg & ~BMI088_READ_BIT), &ignored) &&
                    spi_transfer(value, &ignored) &&
                    spi_wait_idle();
    chip_select(BOARD_IMU_ACCEL_CS_PORT, BOARD_IMU_ACCEL_CS_PIN, false);
    return ok;
}

static bool gyro_write(uint8_t reg, uint8_t value)
{
    uint8_t ignored;
    chip_select(BOARD_IMU_GYRO_CS_PORT, BOARD_IMU_GYRO_CS_PIN, true);
    const bool ok = spi_transfer((uint8_t)(reg & ~BMI088_READ_BIT), &ignored) &&
                    spi_transfer(value, &ignored) &&
                    spi_wait_idle();
    chip_select(BOARD_IMU_GYRO_CS_PORT, BOARD_IMU_GYRO_CS_PIN, false);
    return ok;
}

static bool accel_read(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t ignored;
    bool ok;

    if ((data == 0) || (length == 0U)) return false;
    chip_select(BOARD_IMU_ACCEL_CS_PORT, BOARD_IMU_ACCEL_CS_PIN, true);
    ok = spi_transfer((uint8_t)(reg | BMI088_READ_BIT), &ignored) &&
         spi_transfer(0x55U, &ignored);
    for (uint8_t index = 0U; ok && (index < length); ++index) {
        ok = spi_transfer(0x55U, &data[index]);
    }
    ok = ok && spi_wait_idle();
    chip_select(BOARD_IMU_ACCEL_CS_PORT, BOARD_IMU_ACCEL_CS_PIN, false);
    return ok;
}

static bool gyro_read(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t ignored;
    bool ok;

    if ((data == 0) || (length == 0U)) return false;
    chip_select(BOARD_IMU_GYRO_CS_PORT, BOARD_IMU_GYRO_CS_PIN, true);
    ok = spi_transfer((uint8_t)(reg | BMI088_READ_BIT), &ignored);
    for (uint8_t index = 0U; ok && (index < length); ++index) {
        ok = spi_transfer(0x55U, &data[index]);
    }
    ok = ok && spi_wait_idle();
    chip_select(BOARD_IMU_GYRO_CS_PORT, BOARD_IMU_GYRO_CS_PIN, false);
    return ok;
}

static bool checked_accel_write(uint8_t reg, uint8_t value)
{
    uint8_t readback;
    if (!accel_write(reg, value)) return false;
    delay_us(150U);
    if (!accel_read(reg, &readback, 1U)) return false;
    delay_us(150U);
    return readback == value;
}

static bool checked_gyro_write(uint8_t reg, uint8_t value)
{
    uint8_t readback;
    if (!gyro_write(reg, value)) return false;
    delay_us(150U);
    if (!gyro_read(reg, &readback, 1U)) return false;
    delay_us(150U);
    return readback == value;
}

static int16_t decode_i16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

static int16_t decode_temperature_centi_c(uint8_t msb, uint8_t lsb)
{
    int16_t raw = (int16_t)(((uint16_t)msb << 3U) | ((uint16_t)lsb >> 5U));
    if ((raw & 0x0400) != 0) {
        raw |= (int16_t)0xF800;
    }

    const int32_t scaled = (int32_t)raw * 25;
    const int32_t rounded = (scaled >= 0) ? (scaled + 1) / 2 : (scaled - 1) / 2;
    return (int16_t)(2300 + rounded);
}

bool imu_init(uint32_t cpu_and_timer_clock_hz)
{
    uint8_t chip_id;

    imu_ready = false;
    if ((cpu_and_timer_clock_hz < 1000000UL) ||
        ((cpu_and_timer_clock_hz % 1000000UL) != 0U)) {
        return false;
    }
    imu_clock_hz = cpu_and_timer_clock_hz;

    CORE_DEBUG_DEMCR |= (1UL << 24);
    DWT_CYCCNT = 0U;
    DWT_CTRL |= 1UL;

    RCC_AHB1ENR |= (1UL << 0) | (1UL << 1) | (1UL << 2) | (1UL << 5); /* A/B/C/F */
    RCC_APB2ENR |= (1UL << 12) | (1UL << 17); /* SPI1, TIM10 */
    alternate(BOARD_IMU_SPI1_SCK_PORT, BOARD_IMU_SPI1_SCK_PIN, 5U, false);
    alternate(BOARD_IMU_SPI1_MOSI_PORT, BOARD_IMU_SPI1_MOSI_PIN, 5U, false);
    alternate(BOARD_IMU_SPI1_MISO_PORT, BOARD_IMU_SPI1_MISO_PIN, 5U, false);
    output(BOARD_IMU_ACCEL_CS_PORT, BOARD_IMU_ACCEL_CS_PIN, true);
    output(BOARD_IMU_GYRO_CS_PORT, BOARD_IMU_GYRO_CS_PIN, true);
    alternate(BOARD_IMU_HEATER_PORT, BOARD_IMU_HEATER_PIN, 3U, false);

    SPI_CR1 = (1UL << 2) | (3UL << 3) | (1UL << 8) | (1UL << 9) | (1UL << 6); /* master, /16, software NSS */

    /* Match the proven aruw-mcb BMI088 startup sequence for the Type C board. */
    delay_ms(100U);
    if (!accel_read(0x00U, &chip_id, 1U)) return false;
    delay_ms(1U);
    if (!accel_read(0x00U, &chip_id, 1U)) return false;
    delay_ms(1U);
    if (!accel_write(BMI088_ACCEL_PWR_REG, 0x04U)) return false;
    delay_us(500U);
    if (!accel_read(0x00U, &chip_id, 1U)) return false; /* force SPI mode */
    if (!accel_read(0x00U, &chip_id, 1U) || (chip_id != BMI088_ACCEL_CHIP_ID)) return false;
    if (!checked_accel_write(BMI088_ACCEL_CONF_REG, 0xABU)) return false;  /* 800 Hz, normal */
    if (!checked_accel_write(BMI088_ACCEL_RANGE_REG, 0x00U)) return false; /* +/-3 g */

    if (!gyro_write(BMI088_GYRO_RESET_REG, 0xB6U)) return false;
    delay_ms(80U);
    if (!gyro_read(0x00U, &chip_id, 1U)) return false;
    delay_ms(1U);
    if (!gyro_read(0x00U, &chip_id, 1U) || (chip_id != BMI088_GYRO_CHIP_ID)) return false;
    delay_ms(1U);
    if (!checked_gyro_write(BMI088_GYRO_RANGE_REG, 0x00U)) return false; /* +/-2000 dps */
    if (!checked_gyro_write(BMI088_GYRO_BW_REG, 0x82U)) return false;    /* 1000 Hz/116 Hz */
    if (!checked_gyro_write(BMI088_GYRO_LPM1_REG, 0x00U)) return false;  /* normal */

    (*(volatile uint32_t *)(TIM10_BASE + 0x28UL)) =
        cpu_and_timer_clock_hz / 1000000UL - 1U;
    (*(volatile uint32_t *)(TIM10_BASE + 0x2CUL)) = 999U;
    (*(volatile uint32_t *)(TIM10_BASE + 0x18UL)) = 0x68UL;
    (*(volatile uint32_t *)(TIM10_BASE + 0x20UL)) = 1U;
    (*(volatile uint32_t *)(TIM10_BASE + 0x14UL)) = 1U;
    (*(volatile uint32_t *)(TIM10_BASE + 0x00UL)) = (1UL << 7) | 1U;
    imu_ready = imu_set_heater_duty(0U);
    return imu_ready;
}

bool imu_set_heater_duty(uint8_t duty_percent)
{
    if (duty_percent > 100U) return false;
    (*(volatile uint32_t *)(TIM10_BASE + 0x34UL)) = (1000UL * duty_percent) / 100U;
    return true;
}

bool imu_read_temperature_c(float *temperature_c)
{
    uint8_t temperature[2];
    if ((temperature_c == 0) || !imu_ready ||
        !accel_read(BMI088_ACCEL_TEMP_REG, temperature, sizeof(temperature))) {
        return false;
    }
    *temperature_c =
        (float)decode_temperature_centi_c(temperature[0], temperature[1]) / 100.0f;
    return true;
}

bool imu_read_sample(imu_sample_t *sample)
{
    uint8_t accel_bytes[6];
    uint8_t gyro_bytes[6];
    uint8_t temperature[2];
    imu_sample_t next;

    if ((sample == 0) || !imu_ready) return false;
    if (!accel_read(BMI088_ACCEL_DATA_REG, accel_bytes, sizeof(accel_bytes)) ||
        !gyro_read(BMI088_GYRO_DATA_REG, gyro_bytes, sizeof(gyro_bytes)) ||
        !accel_read(BMI088_ACCEL_TEMP_REG, temperature, sizeof(temperature))) {
        return false;
    }

    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        next.accel_raw[axis] = decode_i16(&accel_bytes[axis * 2U]);
        next.gyro_raw[axis] = decode_i16(&gyro_bytes[axis * 2U]);
    }
    next.temperature_centi_c =
        decode_temperature_centi_c(temperature[0], temperature[1]);
    *sample = next;
    return true;
}

void imu_heater_update(float temperature_c)
{
    static float previous_error;
    const float error = 50.0f - temperature_c; /* Reference target: 50 C. */
    float output = error + 20.0f * (error - previous_error);
    previous_error = error;
    if ((temperature_c < 0.0f) || (output < 0.0f)) output = 0.0f;
    if (output > 1.0f) output = 1.0f;
    (void)imu_set_heater_duty((uint8_t)(output * 100.0f));
}

bool magnetometer_init(uint32_t apb1_clock_hz)
{
    const uint32_t apb1_mhz = apb1_clock_hz / 1000000UL;
    if ((apb1_mhz < 2U) || (apb1_mhz > 42U)) return false;
    RCC_AHB1ENR |= (1UL << 0) | (1UL << 2) | (1UL << 6); /* A/C/G */
    RCC_APB1ENR |= (1UL << 23); /* I2C3 */
    alternate(BOARD_MAG_I2C3_SCL_PORT, BOARD_MAG_I2C3_SCL_PIN, 4U, true);
    alternate(BOARD_MAG_I2C3_SDA_PORT, BOARD_MAG_I2C3_SDA_PIN, 4U, true);
    output(BOARD_MAG_RESET_PORT, BOARD_MAG_RESET_PIN, true);

    (*(volatile uint32_t *)(I2C3_BASE + 0x00UL)) = 0U;
    (*(volatile uint32_t *)(I2C3_BASE + 0x04UL)) = apb1_mhz;
    (*(volatile uint32_t *)(I2C3_BASE + 0x1CUL)) = (1UL << 15) | (apb1_clock_hz / 1200000UL);
    (*(volatile uint32_t *)(I2C3_BASE + 0x20UL)) = apb1_mhz * 3U / 10U + 1U;
    (*(volatile uint32_t *)(I2C3_BASE + 0x00UL)) = 1U;
    return true;
}
