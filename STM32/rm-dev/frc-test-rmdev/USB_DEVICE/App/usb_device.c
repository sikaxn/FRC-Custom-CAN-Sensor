#include "usb_device.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void)
{
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
        return;
    }

    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
        (void)USBD_DeInit(&hUsbDeviceFS);
        return;
    }

    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
        (void)USBD_DeInit(&hUsbDeviceFS);
        return;
    }

    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
        (void)USBD_DeInit(&hUsbDeviceFS);
    }
}
