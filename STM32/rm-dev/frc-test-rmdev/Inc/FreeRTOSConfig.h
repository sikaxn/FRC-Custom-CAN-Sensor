#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      168000000UL
#define configTICK_RATE_HZ                      1000U
#define configMAX_PRIORITIES                     6
#define configMINIMAL_STACK_SIZE                 128U
#define configTOTAL_HEAP_SIZE                    (24U * 1024U)
#define configMAX_TASK_NAME_LEN                  12
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configUSE_QUEUE_SETS                     0
#define configUSE_TIMERS                         0
#define INCLUDE_vTaskDelay                        1
#define INCLUDE_xTaskDelayUntil                   1
#define INCLUDE_xTaskGetSchedulerState            1
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configKERNEL_INTERRUPT_PRIORITY          (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (5 << (8 - configPRIO_BITS))
#define configASSERT(x) do { if (!(x)) { __asm volatile("cpsid i"); for (;;) {} } } while (0)

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler

#endif
