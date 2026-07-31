#include "dbus.h"

#include "board_io.h"

#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB1ENR (*(volatile uint32_t *)0x40023840UL)
#define USART3_BASE 0x40004800UL
#define USART_BRR (*(volatile uint32_t *)(USART3_BASE + 0x08UL))
#define USART_CR1 (*(volatile uint32_t *)(USART3_BASE + 0x0CUL))
#define USART_CR2 (*(volatile uint32_t *)(USART3_BASE + 0x10UL))
#define GPIO_MODER(port) (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_AFRH(port) (*(volatile uint32_t *)((port) + 0x24UL))

#define USART_CR1_RE  (1UL << 2)
#define USART_CR1_PCE (1UL << 10)
#define USART_CR1_M   (1UL << 12)
#define USART_CR1_UE  (1UL << 13)

bool dbus_init(uint32_t apb1_clock_hz)
{
    if (apb1_clock_hz == 0U) return false;
    RCC_AHB1ENR |= (1UL << 2);  /* GPIOC */
    RCC_APB1ENR |= (1UL << 18); /* USART3 */
    GPIO_MODER(BOARD_DBUS_USART3_RX_PORT) =
        (GPIO_MODER(BOARD_DBUS_USART3_RX_PORT) & ~(3UL << 22U)) | (2UL << 22U);
    GPIO_AFRH(BOARD_DBUS_USART3_RX_PORT) =
        (GPIO_AFRH(BOARD_DBUS_USART3_RX_PORT) & ~(0xFUL << 12U)) | (7UL << 12U);

    USART_CR1 = 0U;
    USART_CR2 = 0U; /* one stop bit */
    USART_BRR = (apb1_clock_hz + 50000U) / 100000U;
    USART_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_PCE | USART_CR1_M;
    return true;
}

bool dbus_decode(const uint8_t frame[DBUS_FRAME_SIZE], int16_t channels[4])
{
    if ((frame == 0) || (channels == 0)) return false;
    channels[0] = (int16_t)(((uint16_t)frame[0] | ((uint16_t)frame[1] << 8U)) & 0x07FFU) - 1024;
    channels[1] = (int16_t)(((uint16_t)frame[1] >> 3U | ((uint16_t)frame[2] << 5U)) & 0x07FFU) - 1024;
    channels[2] = (int16_t)(((uint16_t)frame[2] >> 6U | ((uint16_t)frame[3] << 2U) |
                             ((uint16_t)frame[4] << 10U)) & 0x07FFU) - 1024;
    channels[3] = (int16_t)(((uint16_t)frame[4] >> 1U | ((uint16_t)frame[5] << 7U)) & 0x07FFU) - 1024;
    return true;
}
