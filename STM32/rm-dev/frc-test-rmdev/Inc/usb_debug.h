#ifndef USB_DEBUG_H
#define USB_DEBUG_H

#include <stddef.h>
#include <stdint.h>

/*
 * Queue a CDC packet for transmission without waiting for the USB endpoint.
 * Returns length when accepted, or zero while disconnected/not configured/busy.
 */
size_t usb_debug_write(const uint8_t *data, size_t length);

#endif /* USB_DEBUG_H */
