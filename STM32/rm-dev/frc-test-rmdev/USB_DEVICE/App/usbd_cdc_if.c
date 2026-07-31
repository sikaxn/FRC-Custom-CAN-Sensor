#include "usbd_cdc_if.h"

#include <string.h>

#include "usb_device.h"

#define CDC_RX_RING_SIZE 512U

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
static uint8_t receive_ring[CDC_RX_RING_SIZE];
static volatile uint16_t receive_head;
static volatile uint16_t receive_tail;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length);
static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length, uint8_t endpoint);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
    receive_head = 0U;
    receive_tail = 0U;
    (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
    (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    receive_head = 0U;
    receive_tail = 0U;
    return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer, uint16_t length)
{
    static uint8_t line_coding[7] = {
        0x00U, 0xC2U, 0x01U, 0x00U, /* 115200 baud */
        0x00U,                       /* one stop bit */
        0x00U,                       /* no parity */
        0x08U                        /* eight data bits */
    };

    switch (command) {
    case CDC_SET_LINE_CODING:
        if (length >= sizeof(line_coding)) {
            memcpy(line_coding, buffer, sizeof(line_coding));
        }
        break;

    case CDC_GET_LINE_CODING:
        if (length >= sizeof(line_coding)) {
            memcpy(buffer, line_coding, sizeof(line_coding));
        }
        break;

    default:
        break;
    }

    return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length)
{
    uint32_t count = *length;

    for (uint32_t index = 0U; index < count; ++index) {
        uint16_t next = (uint16_t)((receive_head + 1U) % CDC_RX_RING_SIZE);
        if (next == receive_tail) {
            break;
        }

        receive_ring[receive_head] = buffer[index];
        receive_head = next;
    }

    (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length, uint8_t endpoint)
{
    (void)buffer;
    (void)length;
    (void)endpoint;
    return (int8_t)USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc;
    uint8_t result;
    uint32_t primask;

    if ((buffer == NULL) || (length == 0U)) {
        return USBD_FAIL;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (cdc == NULL) ||
        (cdc->TxState != 0U)) {
        result = USBD_BUSY;
    } else {
        (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buffer, length);
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return result;
}

size_t usb_debug_write(const uint8_t *data, size_t length)
{
    USBD_CDC_HandleTypeDef *cdc;
    size_t accepted;
    uint8_t result;
    uint32_t primask;

    if ((data == NULL) || (length == 0U)) {
        return 0U;
    }

    accepted = (length < sizeof(UserTxBufferFS)) ? length : sizeof(UserTxBufferFS);
    primask = __get_PRIMASK();
    __disable_irq();

    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (cdc == NULL) ||
        (cdc->TxState != 0U)) {
        accepted = 0U;
    } else {
        memcpy(UserTxBufferFS, data, accepted);
        (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, (uint32_t)accepted);
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        if (result != USBD_OK) {
            accepted = 0U;
        }
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return accepted;
}

size_t usb_cdc_read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;

    if (data == NULL) {
        return 0U;
    }

    while ((count < capacity) && (receive_tail != receive_head)) {
        data[count++] = receive_ring[receive_tail];
        receive_tail = (uint16_t)((receive_tail + 1U) % CDC_RX_RING_SIZE);
    }

    return count;
}

uint8_t usb_cdc_connected(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}
