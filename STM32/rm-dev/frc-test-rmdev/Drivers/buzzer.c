#include "buzzer.h"

#include "board_io.h"

#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840UL)
#define TIM4_BASE 0x40000800UL
#define TIM_CR1 (*(volatile uint32_t *)(TIM4_BASE + 0x00UL))
#define TIM_EGR (*(volatile uint32_t *)(TIM4_BASE + 0x14UL))
#define TIM_CCMR2 (*(volatile uint32_t *)(TIM4_BASE + 0x1CUL))
#define TIM_CCER (*(volatile uint32_t *)(TIM4_BASE + 0x20UL))
#define TIM_PSC (*(volatile uint32_t *)(TIM4_BASE + 0x28UL))
#define TIM_ARR (*(volatile uint32_t *)(TIM4_BASE + 0x2CUL))
#define TIM_CCR3 (*(volatile uint32_t *)(TIM4_BASE + 0x3CUL))
#define GPIO_MODER(port) (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_AFRH(port) (*(volatile uint32_t *)((port) + 0x24UL))

static uint32_t timer_clock_hz;

bool buzzer_init(uint32_t apb1_timer_clock_hz)
{
    if (apb1_timer_clock_hz == 0U) return false;
    timer_clock_hz = apb1_timer_clock_hz;
    RCC_AHB1ENR |= (1UL << 3); /* GPIOD */
    RCC_APB1ENR |= (1UL << 2); /* TIM4 */
    GPIO_MODER(BOARD_BUZZER_PORT) = (GPIO_MODER(BOARD_BUZZER_PORT) & ~(3UL << 28U)) | (2UL << 28U);
    GPIO_AFRH(BOARD_BUZZER_PORT) = (GPIO_AFRH(BOARD_BUZZER_PORT) & ~(0xFUL << 24U)) | (2UL << 24U);
    TIM_CCMR2 = 0x6800UL; /* PWM mode 1, preload, channel 3 */
    TIM_CCER = (1UL << 8);
    return buzzer_set_tone(4000U, 50U);
}

bool buzzer_set_tone(uint32_t frequency_hz, uint8_t duty_percent)
{
    if ((timer_clock_hz == 0U) || (frequency_hz == 0U) || (duty_percent > 100U)) return false;
    const uint32_t ticks = timer_clock_hz / frequency_hz;
    const uint32_t divider = (ticks + 65535U - 1U) / 65535U;
    if ((ticks == 0U) || (divider == 0U) || (divider > 65536U)) return false;
    const uint32_t period = ticks / divider;
    TIM_CR1 = 0U;
    TIM_PSC = divider - 1U;
    TIM_ARR = period - 1U;
    TIM_CCR3 = (period * duty_percent) / 100U;
    TIM_EGR = 1U;
    TIM_CR1 = (1UL << 7) | 1UL;
    return true;
}

void buzzer_stop(void)
{
    TIM_CCR3 = 0U;
}
