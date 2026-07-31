#ifndef USBD_CONF_H
#define USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#define USBD_MAX_NUM_INTERFACES      1U
#define USBD_MAX_NUM_CONFIGURATION   1U
#define USBD_MAX_STR_DESC_SIZ        128U
#define USBD_DEBUG_LEVEL             0U
#define USBD_LPM_ENABLED             0U
#define USBD_SELF_POWERED            0U

#define DEVICE_FS                    0U
#define DEVICE_HS                    1U

#define USBD_malloc                  USBD_static_malloc
#define USBD_free                    USBD_static_free
#define USBD_memset                  memset
#define USBD_memcpy                  memcpy
#define USBD_Delay                   HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)             do { printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_UsrLog(...)             do { } while (0)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...)             do { printf("ERROR: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_ErrLog(...)             do { } while (0)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...)             do { printf("DEBUG: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_DbgLog(...)             do { } while (0)
#endif

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *memory);

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

#ifdef __cplusplus
}
#endif

#endif /* USBD_CONF_H */
