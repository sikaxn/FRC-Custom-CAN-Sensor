#include "uart.h"

#include "board_io.h"

#define RCC_AHB1ENR (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB2ENR (*(volatile uint32_t *)0x40023844UL)
#define USART1_BASE 0x40011000UL
#define USART6_BASE 0x40011400UL
#define USART_SR(base) (*(volatile uint32_t *)((base) + 0x00UL))
#define USART_DR(base) (*(volatile uint32_t *)((base) + 0x04UL))
#define USART_BRR(base) (*(volatile uint32_t *)((base) + 0x08UL))
#define USART_CR1(base) (*(volatile uint32_t *)((base) + 0x0CUL))
#define GPIO_MODER(port) (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_AFRL(port) (*(volatile uint32_t *)((port) + 0x20UL))
#define GPIO_AFRH(port) (*(volatile uint32_t *)((port) + 0x24UL))

#define USART_SR_TXE (1UL << 7)
#define USART_CR1_RE (1UL << 2)
#define USART_CR1_TE (1UL << 3)
#define USART_CR1_UE (1UL << 13)

static void pin_alternate(uint32_t port, uint8_t pin, uint8_t alternate_function)
{
    const uint32_t shift = pin * 2U;
    const uint32_t af_shift = (pin % 8U) * 4U;
    volatile uint32_t *const af = (pin < 8U) ? &GPIO_AFRL(port) : &GPIO_AFRH(port);
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3UL << shift)) | (2UL << shift);
    *af = (*af & ~(0xFUL << af_shift)) | ((uint32_t)alternate_function << af_shift);
}

static uint32_t base_for(uart_port_t port)
{
    return port == UART_PORT_REFEREE ? USART6_BASE : USART1_BASE;
}

bool uart_init(uart_port_t port, uint32_t peripheral_clock_hz, uint32_t baud_rate)
{
    if ((peripheral_clock_hz == 0U) || (baud_rate == 0U)) return false;
    if (port == UART_PORT_REFEREE) {
        RCC_AHB1ENR |= (1UL << 6); /* GPIOG */
        RCC_APB2ENR |= (1UL << 5); /* USART6 */
        pin_alternate(BOARD_UART1_USART6_TX_PORT, BOARD_UART1_USART6_TX_PIN, 8U);
        pin_alternate(BOARD_UART1_USART6_RX_PORT, BOARD_UART1_USART6_RX_PIN, 8U);
    } else {
        RCC_AHB1ENR |= (1UL << 0) | (1UL << 1); /* GPIOA, GPIOB */
        RCC_APB2ENR |= (1UL << 4); /* USART1 */
        pin_alternate(BOARD_UART2_USART1_TX_PORT, BOARD_UART2_USART1_TX_PIN, 7U);
        pin_alternate(BOARD_UART2_USART1_RX_PORT, BOARD_UART2_USART1_RX_PIN, 7U);
    }

    const uint32_t base = base_for(port);
    USART_CR1(base) = 0U;
    USART_BRR(base) = (peripheral_clock_hz + baud_rate / 2U) / baud_rate;
    USART_CR1(base) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    return true;
}

bool uart_write(uart_port_t port, const uint8_t *data, size_t length)
{
    if (data == 0) return false;
    const uint32_t base = base_for(port);
    while (length-- != 0U) {
        uint32_t timeout = 1000000UL;
        while (((USART_SR(base) & USART_SR_TXE) == 0U) && (--timeout != 0U)) {}
        if (timeout == 0U) return false;
        USART_DR(base) = *data++;
    }
    return true;
}
