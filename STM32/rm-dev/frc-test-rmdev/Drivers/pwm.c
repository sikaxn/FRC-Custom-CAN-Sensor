#include "pwm.h"

#include "board_io.h"

#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844UL)
#define TIM1_BASE 0x40010000UL
#define TIM8_BASE 0x40010400UL
#define TIM_CR1(base) (*(volatile uint32_t *)((base) + 0x00UL))
#define TIM_EGR(base) (*(volatile uint32_t *)((base) + 0x14UL))
#define TIM_CCMR1(base) (*(volatile uint32_t *)((base) + 0x18UL))
#define TIM_CCMR2(base) (*(volatile uint32_t *)((base) + 0x1CUL))
#define TIM_CCER(base) (*(volatile uint32_t *)((base) + 0x20UL))
#define TIM_PSC(base) (*(volatile uint32_t *)((base) + 0x28UL))
#define TIM_ARR(base) (*(volatile uint32_t *)((base) + 0x2CUL))
#define TIM_CCR(base, channel) (*(volatile uint32_t *)((base) + 0x30UL + ((channel) * 4UL)))
#define TIM_BDTR(base) (*(volatile uint32_t *)((base) + 0x44UL))
#define GPIO_MODER(port) (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_AFRL(port) (*(volatile uint32_t *)((port) + 0x20UL))
#define GPIO_AFRH(port) (*(volatile uint32_t *)((port) + 0x24UL))

static void alternate(uint32_t port, uint8_t pin, uint8_t af)
{
    const uint32_t shift = pin * 2U;
    const uint32_t af_shift = (pin % 8U) * 4U;
    volatile uint32_t *const af_register = (pin < 8U) ? &GPIO_AFRL(port) : &GPIO_AFRH(port);
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3UL << shift)) | (2UL << shift);
    *af_register = (*af_register & ~(0xFUL << af_shift)) | ((uint32_t)af << af_shift);
}

static void start_timer(uint32_t base, uint32_t prescaler)
{
    TIM_CR1(base) = 0U;
    TIM_PSC(base) = prescaler;
    TIM_ARR(base) = 19999U; /* 20 ms period at a 1 MHz timer clock */
    TIM_CCMR1(base) = 0x6868UL; /* PWM mode 1, preload, channels 1 and 2 */
    TIM_CCMR2(base) = 0x6868UL; /* PWM mode 1, preload, channels 3 and 4 */
    TIM_CCER(base) = 0x1111UL;
    TIM_EGR(base) = 1U;
    TIM_BDTR(base) = (1UL << 15); /* advanced timers require MOE */
    TIM_CR1(base) = (1UL << 7) | 1UL;
}

bool pwm_servo_init(uint32_t apb2_timer_clock_hz)
{
    if ((apb2_timer_clock_hz < 1000000UL) || ((apb2_timer_clock_hz % 1000000UL) != 0U)) return false;
    RCC_AHB1ENR |= (1UL << 2) | (1UL << 4) | (1UL << 8); /* GPIOC/E/I */
    RCC_APB2ENR |= (1UL << 0) | (1UL << 1); /* TIM1, TIM8 */

    alternate(BOARD_PWM_1_PORT, BOARD_PWM_1_PIN, 1U);
    alternate(BOARD_PWM_2_PORT, BOARD_PWM_2_PIN, 1U);
    alternate(BOARD_PWM_3_PORT, BOARD_PWM_3_PIN, 1U);
    alternate(BOARD_PWM_4_PORT, BOARD_PWM_4_PIN, 1U);
    alternate(BOARD_PWM_5_PORT, BOARD_PWM_5_PIN, 3U);
    alternate(BOARD_PWM_6_PORT, BOARD_PWM_6_PIN, 3U);
    alternate(BOARD_PWM_7_PORT, BOARD_PWM_7_PIN, 3U);

    const uint32_t prescaler = apb2_timer_clock_hz / 1000000UL - 1U;
    start_timer(TIM1_BASE, prescaler);
    start_timer(TIM8_BASE, prescaler);
    return true;
}

bool pwm_servo_set_pulse_us(pwm_servo_t channel, uint16_t pulse_us)
{
    if ((channel > PWM_SERVO_7) || (pulse_us > 20000U)) return false;
    const uint32_t base = channel <= PWM_SERVO_4 ? TIM1_BASE : TIM8_BASE;
    const uint32_t timer_channel = channel <= PWM_SERVO_4 ? (uint32_t)channel : (uint32_t)channel - 4U;
    TIM_CCR(base, timer_channel) = pulse_us;
    return true;
}
