#ifndef RMDEV_DRIVERS_DBUS_H
#define RMDEV_DRIVERS_DBUS_H

#include <stdbool.h>
#include <stdint.h>

#define DBUS_FRAME_SIZE 18U

/* DBUS is USART3 RX at 100000 baud, 8 data bits, even parity, one stop bit. */
bool dbus_init(uint32_t apb1_clock_hz);
bool dbus_decode(const uint8_t frame[DBUS_FRAME_SIZE], int16_t channels[4]);

#endif /* RMDEV_DRIVERS_DBUS_H */
