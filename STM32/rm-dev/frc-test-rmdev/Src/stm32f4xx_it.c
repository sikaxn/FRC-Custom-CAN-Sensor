#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"

/*
 * HAL starts SysTick before the FreeRTOS scheduler exists. Keep the HAL
 * millisecond time base alive during board/USB initialization, and only enter
 * the FreeRTOS tick handler after the scheduler has initialized its lists.
 */
extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
