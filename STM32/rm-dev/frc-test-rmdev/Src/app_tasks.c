#include "app_tasks.h"
#include "frc_can.h"
#include "imu.h"
#include "io.h"
#include "usb_debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "task.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static QueueHandle_t rx_queue;
static QueueHandle_t tx_queue;
static QueueHandle_t imu_can_queue;
static QueueHandle_t imu_telemetry_queue;
static QueueHandle_t power_can_queue;
static QueueHandle_t power_usb_queue;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    imu_sample_t sample;
} imu_telemetry_t;

typedef struct {
    uint32_t timestamp_ms;
    io_battery_sample_t sample;
} power_telemetry_t;

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

#define CAN_TX_MAILBOX_RETRY_LIMIT 3U
#define USB_CANID_COMMAND_CAPACITY 64U
#define USB_CANID_QUIET_TIME_MS     5000U
#define USB_COMMAND_WRITE_TIMEOUT_MS 250U

static bool transmit_with_retry(const can_frame_t *frame)
{
    for (uint32_t attempt = 0U; attempt < CAN_TX_MAILBOX_RETRY_LIMIT; ++attempt) {
        if (can_transmit(frame)) {
            ++health.can_tx;
            return true;
        }
        ++health.can_tx_retries;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Also counts frames that could not enter a full hardware mailbox. */
    ++health.can_tx_enqueue_dropped;
    return false;
}

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
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t wake_time = xTaskGetTickCount();

    for (;;) {
        can_frame_t frame;
        imu_telemetry_t telemetry;

        /* Preserve the general-purpose application transmit queue. */
        for (uint32_t queued = 0U; queued < 4U; ++queued) {
            if (xQueueReceive(tx_queue, &frame, 0U) != pdPASS) break;
            transmit_with_retry(&frame);
        }

        /* Publish one coherent sample as two back-to-back FRC frames at 100 Hz. */
        if (xQueueReceive(imu_can_queue, &telemetry, 0U) == pdPASS) {
            can_frame_t frames[FRC_CAN_IMU_FRAME_COUNT];
            frc_can_build_imu_frames(&telemetry.sample,
                                     (uint16_t)telemetry.sequence, frames);
            for (uint32_t i = 0U; i < FRC_CAN_IMU_FRAME_COUNT; ++i) {
                transmit_with_retry(&frames[i]);
            }
        }

        power_telemetry_t power;
        if (xQueueReceive(power_can_queue, &power, 0U) == pdPASS) {
            frc_can_build_power_frame(
                &power.sample, (uint16_t)(power.timestamp_ms / 1000UL), &frame);
            transmit_with_retry(&frame);
        }

        /* Do not burst through missed periods after an unplugged CAN bus. */
        const TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - wake_time) >= period) wake_time = now;
        vTaskDelayUntil(&wake_time, period);
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
            (void)xQueueOverwrite(imu_can_queue, &telemetry);
            (void)xQueueOverwrite(imu_telemetry_queue, &telemetry);
            imu_heater_update(temperature_c);
        } else {
            ++health.imu_read_errors;
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(2));
    }
}

static void power_task(void *argument)
{
    (void)argument;
    TickType_t wake_time = xTaskGetTickCount();

    for (;;) {
        power_telemetry_t telemetry;
        if (io_battery_voltage_read(&telemetry.sample)) {
            telemetry.timestamp_ms =
                (uint32_t)(xTaskGetTickCount() * (TickType_t)portTICK_PERIOD_MS);
            (void)xQueueOverwrite(power_can_queue, &telemetry);
            (void)xQueueOverwrite(power_usb_queue, &telemetry);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1000));
    }
}

static void status_led_task(void *argument)
{
    (void)argument;
    TickType_t wake_time = xTaskGetTickCount();
    bool lit = true;

    for (;;) {
        io_led_set(lit ? 1U : 0U, lit ? 1U : 0U, lit ? 1U : 0U);
        lit = !lit;
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(500));
    }
}

static bool usb_write_line(const char *line, size_t capacity, int length)
{
    if ((length <= 0) || ((size_t)length >= capacity)) {
        return false;
    }
    return usb_debug_write((const uint8_t *)line, (size_t)length) == (size_t)length;
}

static bool usb_write_line_retry(const char *line, size_t capacity, int length)
{
    const TickType_t started = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(USB_COMMAND_WRITE_TIMEOUT_MS);

    do {
        if (usb_write_line(line, capacity, length)) return true;
        vTaskDelay(pdMS_TO_TICKS(2));
    } while ((xTaskGetTickCount() - started) < timeout);

    return false;
}

static bool parse_can_device_number(const char *text, uint8_t *device_number)
{
    uint32_t value = 0U;

    if ((text == 0) || (device_number == 0) || (*text == '\0')) return false;
    while (*text != '\0') {
        if ((*text < '0') || (*text > '9')) return false;
        value = value * 10U + (uint32_t)(*text - '0');
        if (value > FRC_CAN_MAX_DEVICE_NUMBER) return false;
        ++text;
    }

    *device_number = (uint8_t)value;
    return true;
}

static bool usb_handle_canid_command(const char *command)
{
    static const char set_prefix[] = "&CANID SET ";
    char response[128];
    int length;

    if (strcmp(command, "&CANID GET") == 0) {
        length = snprintf(
            response, sizeof(response),
            "[CANID] Current=%u, EEPROM=%u, Default=%u\r\n",
            (unsigned int)frc_can_get_device_number(),
            (unsigned int)frc_can_get_saved_device_number(),
            (unsigned int)FRC_CAN_DEFAULT_DEVICE_NUMBER);
    } else if (strncmp(command, set_prefix, sizeof(set_prefix) - 1U) == 0) {
        uint8_t device_number;
        if (parse_can_device_number(command + sizeof(set_prefix) - 1U,
                                    &device_number) &&
            frc_can_set_device_number(device_number)) {
            length = snprintf(
                response, sizeof(response),
                "[CANID] Running DEVICE_NUMBER set to %u\r\n",
                (unsigned int)device_number);
        } else {
            length = snprintf(
                response, sizeof(response),
                "[CANID] Invalid value. Must be 0-63.\r\n");
        }
    } else if (strcmp(command, "&CANID SAVE") == 0) {
        if (frc_can_save_device_number()) {
            length = snprintf(
                response, sizeof(response),
                "[CANID] Saved to EEPROM. Rebooting...\r\n");
            (void)usb_write_line_retry(response, sizeof(response), length);
            vTaskDelay(pdMS_TO_TICKS(1000));
            NVIC_SystemReset();
        }
        length = snprintf(
            response, sizeof(response),
            "[CANID] Save failed.\r\n");
    } else {
        return false;
    }

    (void)usb_write_line_retry(response, sizeof(response), length);
    return true;
}

static bool usb_poll_canid_commands(char *command, size_t capacity,
                                    size_t *length, bool *discard)
{
    uint8_t received[64];
    size_t received_count;
    bool handled = false;

    while ((received_count = usb_cdc_read(received, sizeof(received))) != 0U) {
        for (size_t index = 0U; index < received_count; ++index) {
            const uint8_t byte = received[index];
            if (byte == (uint8_t)'\n') {
                if (!*discard) {
                    command[*length] = '\0';
                    if (usb_handle_canid_command(command)) handled = true;
                }
                *length = 0U;
                *discard = false;
            } else if (byte != (uint8_t)'\r' && !*discard) {
                if ((*length + 1U) < capacity) {
                    command[(*length)++] = (char)byte;
                } else {
                    *length = 0U;
                    *discard = true;
                }
            }
        }
    }

    return handled;
}

static void usb_telemetry_task(void *argument)
{
    (void)argument;
    static const char hello[] =
        "HELLO,1,RMDEV-C,BMI088,100,3,2000\r\n";
    char line[192];
    char command[USB_CANID_COMMAND_CAPACITY];
    size_t command_length = 0U;
    bool command_discard = false;
    bool canid_command_seen = false;
    bool hello_sent = false;
    TickType_t wake_time = xTaskGetTickCount();
    TickType_t last_hello_time = wake_time;
    TickType_t last_status_time = wake_time;
    TickType_t last_canid_command_time = wake_time;

    for (;;) {
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(10));
        const TickType_t now = xTaskGetTickCount();

        if (usb_poll_canid_commands(command, sizeof(command), &command_length,
                                    &command_discard)) {
            canid_command_seen = true;
            last_canid_command_time = now;
        }

        /* Keep webIDTool responses visible instead of burying them in telemetry. */
        if (canid_command_seen &&
            (now - last_canid_command_time) <
                pdMS_TO_TICKS(USB_CANID_QUIET_TIME_MS)) {
            continue;
        }

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
            power_telemetry_t power;
            int length;

            if (xQueuePeek(power_usb_queue, &power, 0U) == pdPASS) {
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
                    (unsigned long)power.timestamp_ms,
                    (unsigned long)power.sample.input_mv,
                    (unsigned int)power.sample.battery_adc_raw,
                    (unsigned int)power.sample.vref_adc_raw);
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
    imu_can_queue = xQueueCreate(1U, sizeof(imu_telemetry_t));
    imu_telemetry_queue = xQueueCreate(1U, sizeof(imu_telemetry_t));
    power_can_queue = xQueueCreate(1U, sizeof(power_telemetry_t));
    power_usb_queue = xQueueCreate(1U, sizeof(power_telemetry_t));
    if ((rx_queue == 0) || (tx_queue == 0) || (imu_can_queue == 0) ||
        (imu_telemetry_queue == 0) || (power_can_queue == 0) ||
        (power_usb_queue == 0)) return false;
    return xTaskCreate(can_rx_task, "can_rx", 256U, 0, 4U, 0) == pdPASS &&
           xTaskCreate(can_tx_task, "can_tx", 256U, 0, 3U, 0) == pdPASS &&
           xTaskCreate(imu_task, "imu", 384U, 0, 5U, 0) == pdPASS &&
           xTaskCreate(power_task, "power", 256U, 0, 2U, 0) == pdPASS &&
           xTaskCreate(usb_telemetry_task, "usb_imu", 512U, 0, 2U, 0) == pdPASS &&
           xTaskCreate(status_led_task, "status_led", 160U, 0, 1U, 0) == pdPASS;
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
