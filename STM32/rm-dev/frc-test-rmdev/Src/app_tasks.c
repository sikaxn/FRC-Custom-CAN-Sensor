#include "app_tasks.h"
#include "imu.h"
#include "io.h"
#include "usb_debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdint.h>
#include <stdio.h>

static QueueHandle_t rx_queue;
static QueueHandle_t tx_queue;
static QueueHandle_t imu_telemetry_queue;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    imu_sample_t sample;
} imu_telemetry_t;

typedef struct {
    volatile uint32_t can_rx[2];
    volatile uint32_t can_rx_dropped;
    volatile uint32_t can_tx;
    volatile uint32_t can_tx_retries;
    volatile uint32_t can_tx_enqueue_dropped;
    volatile uint32_t imu_reads;
    volatile uint32_t imu_read_errors;
    volatile int32_t imu_temperature_centi_c;
} app_health_t;

static app_health_t health;

static void can_rx_task(void *argument)
{
    (void)argument;
    for (;;) {
        can_frame_t frame;
        while (can_receive(CAN_BUS_1, &frame)) {
            ++health.can_rx[0];
            if (xQueueSend(rx_queue, &frame, 0) != pdPASS) ++health.can_rx_dropped;
        }
        while (can_receive(CAN_BUS_2, &frame)) {
            ++health.can_rx[1];
            if (xQueueSend(rx_queue, &frame, 0) != pdPASS) ++health.can_rx_dropped;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void can_tx_task(void *argument)
{
    (void)argument;
    can_frame_t frame;
    for (;;) {
        if (xQueueReceive(tx_queue, &frame, portMAX_DELAY) == pdPASS) {
            while (!can_transmit(&frame)) {
                ++health.can_tx_retries;
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            ++health.can_tx;
        }
    }
}

static void imu_task(void *argument)
{
    (void)argument;
    TickType_t wake_time = xTaskGetTickCount();

    for (;;) {
        imu_sample_t sample;
        if (imu_read_sample(&sample)) {
            imu_telemetry_t telemetry;
            const float temperature_c = (float)sample.temperature_centi_c / 100.0f;

            health.imu_temperature_centi_c = sample.temperature_centi_c;
            ++health.imu_reads;
            telemetry.sequence = health.imu_reads;
            telemetry.timestamp_ms =
                (uint32_t)(xTaskGetTickCount() * (TickType_t)portTICK_PERIOD_MS);
            telemetry.sample = sample;
            (void)xQueueOverwrite(imu_telemetry_queue, &telemetry);
            imu_heater_update(temperature_c);
        } else {
            ++health.imu_read_errors;
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(2));
    }
}

static bool usb_write_line(char *line, size_t capacity, int length)
{
    if ((length <= 0) || ((size_t)length >= capacity)) {
        return false;
    }
    return usb_debug_write((const uint8_t *)line, (size_t)length) == (size_t)length;
}

static void usb_telemetry_task(void *argument)
{
    (void)argument;
    static const char hello[] =
        "HELLO,1,RMDEV-C,BMI088,100,3,2000\r\n";
    char line[192];
    bool hello_sent = false;
    TickType_t wake_time = xTaskGetTickCount();
    TickType_t last_hello_time = wake_time;
    TickType_t last_status_time = wake_time;

    for (;;) {
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(10));
        const TickType_t now = xTaskGetTickCount();

        if (!hello_sent || ((now - last_hello_time) >= pdMS_TO_TICKS(5000))) {
            if (
                usb_debug_write((const uint8_t *)hello, sizeof(hello) - 1U) ==
                sizeof(hello) - 1U) {
                hello_sent = true;
                last_hello_time = now;
            }
            continue;
        }

        if ((now - last_status_time) >= pdMS_TO_TICKS(1000)) {
            const unsigned long uptime_ms =
                (unsigned long)(now * (TickType_t)portTICK_PERIOD_MS);
            io_battery_sample_t power;
            int length;

            if (io_battery_voltage_read(&power)) {
                length = snprintf(
                    line, sizeof(line),
                    "STAT,1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n"
                    "PWR,1,%lu,%lu,%u,%u\r\n",
                    uptime_ms,
                    (unsigned long)health.can_rx[0],
                    (unsigned long)health.can_rx[1],
                    (unsigned long)health.can_rx_dropped,
                    (unsigned long)health.can_tx,
                    (unsigned long)health.can_tx_retries,
                    (unsigned long)health.can_tx_enqueue_dropped,
                    (unsigned long)health.imu_reads,
                    (unsigned long)health.imu_read_errors,
                    uptime_ms,
                    (unsigned long)power.input_mv,
                    (unsigned int)power.battery_adc_raw,
                    (unsigned int)power.vref_adc_raw);
            } else {
                length = snprintf(
                    line, sizeof(line),
                    "STAT,1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                    uptime_ms,
                    (unsigned long)health.can_rx[0],
                    (unsigned long)health.can_rx[1],
                    (unsigned long)health.can_rx_dropped,
                    (unsigned long)health.can_tx,
                    (unsigned long)health.can_tx_retries,
                    (unsigned long)health.can_tx_enqueue_dropped,
                    (unsigned long)health.imu_reads,
                    (unsigned long)health.imu_read_errors);
            }
            if (usb_write_line(line, sizeof(line), length)) {
                last_status_time = now;
            }
            continue;
        }

        imu_telemetry_t telemetry;
        if (xQueueReceive(imu_telemetry_queue, &telemetry, 0U) == pdPASS) {
            const int length = snprintf(
                line, sizeof(line),
                "IMU,1,%lu,%lu,%d,%d,%d,%d,%d,%d,%d\r\n",
                (unsigned long)telemetry.sequence,
                (unsigned long)telemetry.timestamp_ms,
                (int)telemetry.sample.accel_raw[0],
                (int)telemetry.sample.accel_raw[1],
                (int)telemetry.sample.accel_raw[2],
                (int)telemetry.sample.gyro_raw[0],
                (int)telemetry.sample.gyro_raw[1],
                (int)telemetry.sample.gyro_raw[2],
                (int)telemetry.sample.temperature_centi_c);
            (void)usb_write_line(line, sizeof(line), length);
        }
    }
}

bool app_tasks_start(void)
{
    rx_queue = xQueueCreate(32U, sizeof(can_frame_t));
    tx_queue = xQueueCreate(32U, sizeof(can_frame_t));
    imu_telemetry_queue = xQueueCreate(1U, sizeof(imu_telemetry_t));
    if ((rx_queue == 0) || (tx_queue == 0) || (imu_telemetry_queue == 0)) return false;
    return xTaskCreate(can_rx_task, "can_rx", 256U, 0, 4U, 0) == pdPASS &&
           xTaskCreate(can_tx_task, "can_tx", 256U, 0, 3U, 0) == pdPASS &&
           xTaskCreate(imu_task, "imu", 384U, 0, 5U, 0) == pdPASS &&
           xTaskCreate(usb_telemetry_task, "usb_imu", 512U, 0, 2U, 0) == pdPASS;
}

bool app_can_send(const can_frame_t *frame)
{
    if ((tx_queue != 0) && (frame != 0) &&
        (xQueueSend(tx_queue, frame, 0) == pdPASS)) {
        return true;
    }
    ++health.can_tx_enqueue_dropped;
    return false;
}

bool app_can_receive(can_frame_t *frame)
{
    return (rx_queue != 0) && (frame != 0) && (xQueueReceive(rx_queue, frame, 0) == pdPASS);
}

void vApplicationMallocFailedHook(void) { for (;;) {} }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name) { (void)task; (void)name; for (;;) {} }
