#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* HAL modules used by the Type C board support and USB CDC firmware. */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/*
 * X1 is marked "12.0MHz" on sheet 6 of the RoboMaster Type C schematic
 * (PH0-OSC_IN and PH1-OSC_OUT).
 */
#if !defined(HSE_VALUE)
#define HSE_VALUE                 12000000U
#endif
#define HSE_STARTUP_TIMEOUT       100U

#if !defined(HSI_VALUE)
#define HSI_VALUE                 16000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE                 32000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE                 32768U
#endif
#define LSE_STARTUP_TIMEOUT       5000U
#define EXTERNAL_CLOCK_VALUE      12288000U

#define VDD_VALUE                 3300U
#define TICK_INT_PRIORITY         0x0FU
/* STM32F4 HAL requires this legacy switch to remain zero, even with FreeRTOS. */
#define USE_RTOS                  0U
#define PREFETCH_ENABLE           1U
#define INSTRUCTION_CACHE_ENABLE  1U
#define DATA_CACHE_ENABLE         1U

#define USE_HAL_ADC_REGISTER_CALLBACKS   0U
#define USE_HAL_CAN_REGISTER_CALLBACKS   0U
#define USE_HAL_I2C_REGISTER_CALLBACKS   0U
#define USE_HAL_PCD_REGISTER_CALLBACKS   0U
#define USE_HAL_SPI_REGISTER_CALLBACKS   0U
#define USE_HAL_TIM_REGISTER_CALLBACKS   0U
#define USE_HAL_UART_REGISTER_CALLBACKS  0U

#define USE_SPI_CRC 1U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32f4xx_hal_adc.h"
#endif
#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32f4xx_hal_i2c.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
#endif
#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f4xx_hal_pcd.h"
#endif

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif
