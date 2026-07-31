#ifndef RMDEV_DRIVERS_UART_H
#define RMDEV_DRIVERS_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { UART_PORT_REFEREE = 0, UART_PORT_EXTERNAL = 1 } uart_port_t;

/* Referee = board's 3-pin "UART1" (USART6); External = 4-pin "UART2" (USART1). */
bool uart_init(uart_port_t port, uint32_t peripheral_clock_hz, uint32_t baud_rate);
bool uart_write(uart_port_t port, const uint8_t *data, size_t length);

#endif /* RMDEV_DRIVERS_UART_H */
