#ifndef BOARD_CLOCK_H
#define BOARD_CLOCK_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * RoboMaster Development Board Type C clock tree.
 *
 * The board schematic identifies X1 as a 12.0 MHz crystal on PH0/PH1.
 * PLL: 12 MHz / 6 * 168 / 2 = 168 MHz SYSCLK
 * USB: 12 MHz / 6 * 168 / 7 = 48 MHz OTG FS clock
 */
#define BOARD_HSE_CLOCK_HZ       12000000UL
#define BOARD_SYSCLK_HZ         168000000UL
#define BOARD_AHB_CLOCK_HZ      168000000UL
#define BOARD_APB1_CLOCK_HZ      42000000UL
#define BOARD_APB2_CLOCK_HZ      84000000UL
#define BOARD_APB1_TIMER_CLOCK_HZ 84000000UL
#define BOARD_APB2_TIMER_CLOCK_HZ 168000000UL
#define BOARD_USB_CLOCK_HZ       48000000UL

HAL_StatusTypeDef board_clock_config(void);

#ifdef __cplusplus
}
#endif

#endif
