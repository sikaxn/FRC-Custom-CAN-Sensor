#include "usbd_conf.h"

#include "usbd_cdc.h"
#include "usbd_core.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;

static USBD_StatusTypeDef usb_status_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return USBD_OK;
    case HAL_BUSY:
        return USBD_BUSY;
    default:
        return USBD_FAIL;
    }
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
    GPIO_InitTypeDef gpio = {0};

    if (pcd->Instance != USB_OTG_FS) {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * RoboMaster Type C USB:
     *   PA11 = OTG_FS_DM, PA12 = OTG_FS_DP.
     * PA9 is UART1_TX on this board, so device mode deliberately operates
     * without VBUS sensing. PA10 is wired to the Micro-USB ID pin but is not
     * needed while the peripheral is forced into device mode.
     */
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    /* Below the FreeRTOS max-syscall priority; this ISR does not call RTOS APIs. */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB_OTG_FS) {
        return;
    }

    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd)
{
    (void)USBD_LL_SetupStage((USBD_HandleTypeDef *)pcd->pData,
                            (uint8_t *)pcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t endpoint)
{
    (void)USBD_LL_DataOutStage((USBD_HandleTypeDef *)pcd->pData,
                              endpoint,
                              pcd->OUT_ep[endpoint].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t endpoint)
{
    (void)USBD_LL_DataInStage((USBD_HandleTypeDef *)pcd->pData,
                             endpoint,
                             pcd->IN_ep[endpoint].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *pcd)
{
    (void)USBD_LL_SOF((USBD_HandleTypeDef *)pcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    USBD_SpeedTypeDef speed = (pcd->Init.speed == PCD_SPEED_HIGH) ?
        USBD_SPEED_HIGH :
        USBD_SPEED_FULL;

    (void)USBD_LL_SetSpeed((USBD_HandleTypeDef *)pcd->pData, speed);
    (void)USBD_LL_Reset((USBD_HandleTypeDef *)pcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *pcd)
{
    (void)USBD_LL_Suspend((USBD_HandleTypeDef *)pcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *pcd)
{
    (void)USBD_LL_Resume((USBD_HandleTypeDef *)pcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t endpoint)
{
    (void)USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)pcd->pData, endpoint);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t endpoint)
{
    (void)USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)pcd->pData, endpoint);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *pcd)
{
    (void)USBD_LL_DevConnected((USBD_HandleTypeDef *)pcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *pcd)
{
    (void)USBD_LL_DevDisconnected((USBD_HandleTypeDef *)pcd->pData);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *device)
{
    HAL_StatusTypeDef status;

    if (device->id != DEVICE_FS) {
        return USBD_FAIL;
    }

    hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints = 4U;
    hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

    hpcd_USB_OTG_FS.pData = device;
    device->pData = &hpcd_USB_OTG_FS;

    status = HAL_PCD_Init(&hpcd_USB_OTG_FS);
    if (status != HAL_OK) {
        return usb_status_from_hal(status);
    }

    /* 320 x 32-bit words available in the STM32F407 OTG FS packet RAM. */
    (void)HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 128U);
    (void)HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0U, 64U);
    (void)HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1U, 112U);
    (void)HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2U, 16U);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *device)
{
    return usb_status_from_hal(HAL_PCD_DeInit((PCD_HandleTypeDef *)device->pData));
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *device)
{
    return usb_status_from_hal(HAL_PCD_Start((PCD_HandleTypeDef *)device->pData));
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *device)
{
    return usb_status_from_hal(HAL_PCD_Stop((PCD_HandleTypeDef *)device->pData));
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *device,
                                 uint8_t address,
                                 uint8_t type,
                                 uint16_t max_packet)
{
    return usb_status_from_hal(HAL_PCD_EP_Open((PCD_HandleTypeDef *)device->pData,
                                               address,
                                               max_packet,
                                               type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *device, uint8_t address)
{
    return usb_status_from_hal(HAL_PCD_EP_Close((PCD_HandleTypeDef *)device->pData,
                                                address));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *device, uint8_t address)
{
    return usb_status_from_hal(HAL_PCD_EP_Flush((PCD_HandleTypeDef *)device->pData,
                                                address));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *device, uint8_t address)
{
    return usb_status_from_hal(HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)device->pData,
                                                   address));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *device, uint8_t address)
{
    return usb_status_from_hal(HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)device->pData,
                                                   address));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *device, uint8_t address)
{
    PCD_HandleTypeDef *pcd = (PCD_HandleTypeDef *)device->pData;

    if ((address & 0x80U) != 0U) {
        return pcd->IN_ep[address & 0x7FU].is_stall;
    }
    return pcd->OUT_ep[address & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *device, uint8_t address)
{
    return usb_status_from_hal(HAL_PCD_SetAddress((PCD_HandleTypeDef *)device->pData,
                                                  address));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *device,
                                   uint8_t address,
                                   uint8_t *buffer,
                                   uint32_t size)
{
    return usb_status_from_hal(HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)device->pData,
                                                   address,
                                                   buffer,
                                                   size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *device,
                                         uint8_t address,
                                         uint8_t *buffer,
                                         uint32_t size)
{
    return usb_status_from_hal(HAL_PCD_EP_Receive((PCD_HandleTypeDef *)device->pData,
                                                  address,
                                                  buffer,
                                                  size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *device, uint8_t address)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)device->pData, address);
}

void *USBD_static_malloc(uint32_t size)
{
    static uint32_t class_memory[(sizeof(USBD_CDC_HandleTypeDef) + 3U) / 4U];
    (void)size;
    return class_memory;
}

void USBD_static_free(void *memory)
{
    (void)memory;
}

void USBD_LL_Delay(uint32_t delay)
{
    HAL_Delay(delay);
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
