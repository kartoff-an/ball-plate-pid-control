/**
 * @file telemetry.c
 * @brief UDP telemetry transmission service.
 *
 * Implements a periodic telemetry publisher that sends the latest
 * controller sample as a JSON-formatted UDP broadcast packet.
 *
 * A one-element FreeRTOS queue stores pending telemetry. New samples
 * overwrite older pending samples, ensuring that transmission favors
 * the most recent controller state rather than building up a backlog.
 *
 * The telemetry task transmits at a nominal rate of 20 Hz.
 *
 * @note UDP broadcast destination and transmission settings are configured
 *       through the module-level constants below.
 * @note UDP does not guarantee packet delivery or ordering.
 */

#include "telemetry.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TELEMETRY_PORT              9003
#define TELEMETRY_BROADCAST_IP      "192.168.4.255"
#define TELEMETRY_BUFFER_SIZE       512

#define TELEMETRY_PERIOD_MS         50
#define TELEMETRY_TASK_STACK        4096
#define TELEMETRY_TASK_PRIORITY     3

static const char *TAG = "TELEMETRY";

static int s_socket = -1;
static struct sockaddr_in s_destination;

static QueueHandle_t s_queue = NULL;
static TaskHandle_t s_task = NULL;

static portMUX_TYPE s_telemetry_mux = portMUX_INITIALIZER_UNLOCKED;
static telemetry_mode_t s_telemetry_mode = TELEMETRY_MODE_POSITION;

static esp_err_t telemetry_send(const telemetry_data_t *data)
{
    if (data == NULL || s_socket < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    telemetry_mode_t mode;

    portENTER_CRITICAL(&s_telemetry_mux);
    mode = s_telemetry_mode;
    portEXIT_CRITICAL(&s_telemetry_mux);

    char buffer[TELEMETRY_BUFFER_SIZE];
    int length = -1;

    switch(mode) {
        case TELEMETRY_MODE_POSITION:
            length = snprintf(buffer, sizeof(buffer),
                "{"
                    "\"type\":\"telemetry\","
                    "\"mode\":\"position\","
                    "\"sequence\":%" PRIu32 ","
                    "\"timestamp_us\":%" PRId64 ","
                    "\"ball_detected\":%s,"
                    "\"ball_x_mm\":%.3f,"
                    "\"ball_y_mm\":%.3f,"
                    "\"target_x_mm\":%.3f,"
                    "\"target_y_mm\":%.3f,"
                    "\"error_x_mm\":%.3f,"
                    "\"error_y_mm\":%.3f,"
                    "\"measurement_age_ms\":%" PRId64 ","
                    "\"controller_enabled\":%s"
                "}",
                data->sequence,
                data->timestamp_us,
                data->ball_detected ? "true" : "false",
                (double)data->ball_x_mm,
                (double)data->ball_y_mm,
                (double)data->target_x_mm,
                (double)data->target_y_mm,
                (double)data->error_x_mm,
                (double)data->error_y_mm,
                data->measurement_age_ms,
                data->controller_enabled ? "true" : "false"
            );
            break;

        case TELEMETRY_MODE_VELOCITY:
            length = snprintf(buffer, sizeof(buffer),
                "{"
                    "\"type\":\"telemetry\","
                    "\"mode\":\"velocity\","
                    "\"sequence\":%" PRIu32 ","
                    "\"timestamp_us\":%" PRId64 ","
                    "\"ball_detected\":%s,"
                    "\"ball_x_mm\":%.3f,"
                    "\"ball_y_mm\":%.3f,"
                    "\"velocity_x_mm_s\":%.3f,"
                    "\"velocity_y_mm_s\":%.3f,"
                    "\"target_x_mm\":%.3f,"
                    "\"target_y_mm\":%.3f,"
                    "\"measurement_age_ms\":%" PRId64 ","
                    "\"controller_enabled\":%s"
                "}",
                data->sequence,
                data->timestamp_us,
                data->ball_detected ? "true" : "false",
                (double)data->ball_x_mm,
                (double)data->ball_y_mm,
                (double)data->velocity_x_mm_s,
                (double)data->velocity_y_mm_s,
                (double)data->target_x_mm,
                (double)data->target_y_mm,
                data->measurement_age_ms,
                data->controller_enabled ? "true" : "false"
            );
            break;

        case TELEMETRY_MODE_SERVO:
            length = snprintf(buffer, sizeof(buffer),
                "{"
                    "\"type\":\"telemetry\","
                    "\"mode\":\"servo\","
                    "\"sequence\":%" PRIu32 ","
                    "\"timestamp_us\":%" PRId64 ","
                    "\"error_x_mm\":%.3f,"
                    "\"error_y_mm\":%.3f,"
                    "\"pid_output_x_us\":%.3f,"
                    "\"pid_output_y_us\":%.3f,"
                    "\"output_x_us\":%.3f,"
                    "\"output_y_us\":%.3f,"
                    "\"servo_neutral_x_us\":%u,"
                    "\"servo_neutral_y_us\":%u,"
                    "\"servo_x_us\":%u,"
                    "\"servo_y_us\":%u,"
                    "\"servo_saturated_x\":%s,"
                    "\"servo_saturated_y\":%s,"
                    "\"controller_enabled\":%s"
                "}",
                data->sequence,
                data->timestamp_us,
                (double)data->error_x_mm,
                (double)data->error_y_mm,
                (double)data->pid_output_x_us,
                (double)data->pid_output_y_us,
                (double)data->output_x_us,
                (double)data->output_y_us,
                (unsigned)data->servo_neutral_x_us,
                (unsigned)data->servo_neutral_y_us,
                (unsigned)data->servo_x_us,
                (unsigned)data->servo_y_us,
                data->servo_saturated_x ? "true" : "false",
                data->servo_saturated_y ? "true" : "false",
                data->controller_enabled ? "true" : "false"
            );
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (length < 0 || length >= sizeof(buffer)) {
        return ESP_ERR_INVALID_SIZE;
    }

    int sent = sendto(s_socket, buffer, length, 0, (struct sockaddr *)&s_destination, sizeof(s_destination));
    if (sent < 0) {
        ESP_LOGW(TAG, "UDP telemetry send failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void telemetry_task(void *arg)
{
    (void)arg;

    telemetry_data_t latest;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        /**
         * Limit transmission to 20 Hz
         * The PID task can publish more frequently
         */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        if (xQueueReceive(s_queue, &latest, 0) == pdPASS) {
            (void)telemetry_send(&latest);
        }
    }
}

esp_err_t telemetry_init(void)
{
    if (s_queue != NULL || s_socket >= 0) {
        return ESP_ERR_INVALID_STATE;
    }

    s_queue = xQueueCreate(1, sizeof(telemetry_data_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return ESP_ERR_NO_MEM;
    }

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return ESP_FAIL;
    }

    int broadcast_enabled = 1;
    if (setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_enabled, sizeof(broadcast_enabled)) < 0) {
        ESP_LOGE(TAG, "Failed to enable UDP broadcast");
        telemetry_deinit();
        return ESP_FAIL;
    }

    memset(&s_destination, 0, sizeof(s_destination));
    s_destination.sin_family = AF_INET;
    s_destination.sin_port = htons(TELEMETRY_PORT);
    s_destination.sin_addr.s_addr = inet_addr(TELEMETRY_BROADCAST_IP);

    BaseType_t result = xTaskCreate(
        telemetry_task,
        "telemetry",
        TELEMETRY_TASK_STACK,
        NULL,
        TELEMETRY_TASK_PRIORITY,
        &s_task
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task");
        telemetry_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP telemetry initialized: %s:%d", TELEMETRY_BROADCAST_IP, TELEMETRY_PORT);
    return ESP_OK;
}

esp_err_t telemetry_set_mode(telemetry_mode_t mode)
{
    if (mode != TELEMETRY_MODE_POSITION && mode != TELEMETRY_MODE_VELOCITY && mode != TELEMETRY_MODE_SERVO) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_telemetry_mux);
    s_telemetry_mode = mode;
    portEXIT_CRITICAL(&s_telemetry_mux);

    ESP_LOGI(TAG, "Telemetry mode updated: %d", (int)mode);

    return ESP_OK;
}

esp_err_t telemetry_publish(const telemetry_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return xQueueOverwrite(s_queue, data) == pdPASS ? ESP_OK : ESP_FAIL;
}

void telemetry_deinit(void)
{
    if (s_task != NULL) {
        vTaskDelete(s_task);
        s_task = NULL;
    }

    if (s_socket >= 0) {
        close(s_socket);
        s_socket = -1;
    }

    if (s_queue != NULL) {
        vQueueDelete(s_queue);
        s_queue = NULL;
    }
}