#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "usb_debug.h"
#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE
#define APP_TX_DATA_SIZE  256U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/*
 * Cube-compatible zero-copy transmit call. Buf must remain valid until the
 * transfer completes. Prefer usb_debug_write() for ordinary debug messages.
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* Non-blocking receive access for commands typed into the virtual COM port. */
size_t usb_cdc_read(uint8_t *data, size_t capacity);
uint8_t usb_cdc_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* USBD_CDC_IF_H */
