#include "io.h"

#include "board_io.h"

#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844UL)
#define GPIO_MODER(port) (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_PUPDR(port) (*(volatile uint32_t *)((port) + 0x0CUL))
#define GPIO_IDR(port) (*(volatile uint32_t *)((port) + 0x10UL))
#define GPIO_BSRR(port) (*(volatile uint32_t *)((port) + 0x18UL))

#define ADC1_BASE 0x40012000UL
#define ADC3_BASE 0x40012200UL
#define ADC_COMMON_CCR (*(volatile uint32_t *)0x40012304UL)
#define ADC_SR(base) (*(volatile uint32_t *)((base) + 0x00UL))
#define ADC_CR1(base) (*(volatile uint32_t *)((base) + 0x04UL))
#define ADC_CR2(base) (*(volatile uint32_t *)((base) + 0x08UL))
#define ADC_SMPR1(base) (*(volatile uint32_t *)((base) + 0x0CUL))
#define ADC_SMPR2(base) (*(volatile uint32_t *)((base) + 0x10UL))
#define ADC_SQR1(base) (*(volatile uint32_t *)((base) + 0x2CUL))
#define ADC_SQR2(base) (*(volatile uint32_t *)((base) + 0x30UL))
#define ADC_SQR3(base) (*(volatile uint32_t *)((base) + 0x34UL))
#define ADC_DR(base) (*(volatile uint32_t *)((base) + 0x4CUL))

#define ADC_EOC_BIT (1UL << 1)
#define ADC_SWSTART_BIT (1UL << 30)
#define ADC_ADON_BIT 1UL
#define ADC_AVERAGE_SAMPLES 32U
#define ADC_TIMEOUT 100000U

/*
 * User calibration on this board:
 *   external reference = 14.950 V
 *   ADC3_IN8 average   = 1783.750 counts
 *   ADC1 VREFINT avg   = 1467.875 counts
 *
 * input_mv = battery_raw * 12302.582... / vref_raw.
 * VREFINT compensation removes VDDA variation; the rounded coefficient also
 * includes the actual 200k/22k divider tolerance on this board.
 */
#define BATTERY_CALIBRATED_SCALE_MV 12303UL

static void output_init(uint32_t port, uint8_t pin)
{
    const uint32_t shift = pin * 2U;
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3UL << shift)) | (1UL << shift);
}

static void write(uint32_t port, uint8_t pin, bool high)
{
    GPIO_BSRR(port) = high ? BOARD_PIN_MASK(pin) : (BOARD_PIN_MASK(pin) << 16U);
}

static bool adc_read_average(uint32_t base, uint16_t *average)
{
    uint32_t sum = 0U;

    if (average == 0) {
        return false;
    }

    for (uint32_t sample = 0U; sample < ADC_AVERAGE_SAMPLES; ++sample) {
        uint32_t timeout = ADC_TIMEOUT;

        ADC_SR(base) = 0U;
        ADC_CR2(base) |= ADC_SWSTART_BIT;
        while (((ADC_SR(base) & ADC_EOC_BIT) == 0U) && (--timeout != 0U)) {}
        if (timeout == 0U) {
            return false;
        }
        sum += ADC_DR(base) & 0x0FFFUL;
    }

    *average =
        (uint16_t)((sum + (ADC_AVERAGE_SAMPLES / 2U)) / ADC_AVERAGE_SAMPLES);
    return true;
}

static void battery_adc_init(void)
{
    const uint32_t pin_shift = BOARD_BATTERY_VOLTAGE_ADC_PIN * 2U;

    RCC_AHB1ENR |= (1UL << 5);                  /* GPIOF */
    RCC_APB2ENR |= (1UL << 8) | (1UL << 10);   /* ADC1, ADC3 */
    (void)RCC_APB2ENR;

    GPIO_MODER(BOARD_BATTERY_VOLTAGE_ADC_PORT) =
        (GPIO_MODER(BOARD_BATTERY_VOLTAGE_ADC_PORT) & ~(3UL << pin_shift)) |
        (3UL << pin_shift);
    GPIO_PUPDR(BOARD_BATTERY_VOLTAGE_ADC_PORT) &= ~(3UL << pin_shift);

    /*
     * PCLK2 is 84 MHz. ADC /4 gives 21 MHz, below the F407's 36 MHz limit.
     * TSVREFE enables the internal 1.2 V reference on ADC1 channel 17.
     */
    ADC_COMMON_CCR =
        (ADC_COMMON_CCR & ~((3UL << 16) | 0x1FUL)) |
        (1UL << 16) | (1UL << 23);

    ADC_CR1(ADC1_BASE) = 0U;
    ADC_CR2(ADC1_BASE) = 0U;
    ADC_SMPR1(ADC1_BASE) = 7UL << 21; /* Channel 17, 480 cycles. */
    ADC_SMPR2(ADC1_BASE) = 0U;
    ADC_SQR1(ADC1_BASE) = 0U;
    ADC_SQR2(ADC1_BASE) = 0U;
    ADC_SQR3(ADC1_BASE) = 17U;

    ADC_CR1(ADC3_BASE) = 0U;
    ADC_CR2(ADC3_BASE) = 0U;
    ADC_SMPR1(ADC3_BASE) = 0U;
    ADC_SMPR2(ADC3_BASE) = 7UL << 24; /* Channel 8, 480 cycles. */
    ADC_SQR1(ADC3_BASE) = 0U;
    ADC_SQR2(ADC3_BASE) = 0U;
    ADC_SQR3(ADC3_BASE) = 8U;

    ADC_CR2(ADC1_BASE) = ADC_ADON_BIT;
    ADC_CR2(ADC3_BASE) = ADC_ADON_BIT;
}

void io_init(void)
{
    RCC_AHB1ENR |=
        (1UL << 0) | (1UL << 2) | (1UL << 5) | (1UL << 7); /* GPIOA, C, F, H */
    (void)RCC_AHB1ENR;

    output_init(BOARD_LED_RED_PORT, BOARD_LED_RED_PIN);
    output_init(BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN);
    output_init(BOARD_LED_BLUE_PORT, BOARD_LED_BLUE_PIN);
    output_init(BOARD_LASER_5V_ENABLE_PORT, BOARD_LASER_5V_ENABLE_PIN);
    io_led_set(0U, 0U, 0U);
    io_laser_5v_set(false);

    /* The board has a physical pull-up; retain a pull-up in the MCU too. */
    GPIO_MODER(BOARD_USER_BUTTON_PORT) &= ~(3UL << (BOARD_USER_BUTTON_PIN * 2U));
    GPIO_PUPDR(BOARD_USER_BUTTON_PORT) =
        (GPIO_PUPDR(BOARD_USER_BUTTON_PORT) & ~(3UL << (BOARD_USER_BUTTON_PIN * 2U))) |
        (1UL << (BOARD_USER_BUTTON_PIN * 2U));

    battery_adc_init();
}

void io_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    write(BOARD_LED_RED_PORT, BOARD_LED_RED_PIN, red != 0U);
    write(BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN, green != 0U);
    write(BOARD_LED_BLUE_PORT, BOARD_LED_BLUE_PIN, blue != 0U);
}

bool io_user_button_pressed(void)
{
    return (GPIO_IDR(BOARD_USER_BUTTON_PORT) & BOARD_PIN_MASK(BOARD_USER_BUTTON_PIN)) == 0U;
}

void io_laser_5v_set(bool enabled)
{
    write(BOARD_LASER_5V_ENABLE_PORT, BOARD_LASER_5V_ENABLE_PIN, enabled);
}

bool io_battery_voltage_read(io_battery_sample_t *sample)
{
    io_battery_sample_t next;

    if ((sample == 0) ||
        !adc_read_average(ADC3_BASE, &next.battery_adc_raw) ||
        !adc_read_average(ADC1_BASE, &next.vref_adc_raw) ||
        (next.vref_adc_raw == 0U)) {
        return false;
    }

    next.input_mv =
        ((uint32_t)next.battery_adc_raw * BATTERY_CALIBRATED_SCALE_MV +
         ((uint32_t)next.vref_adc_raw / 2U)) /
        (uint32_t)next.vref_adc_raw;
    *sample = next;
    return true;
}
