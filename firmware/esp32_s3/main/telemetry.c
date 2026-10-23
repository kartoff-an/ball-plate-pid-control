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

static esp_err_t telemetry_send(const telemetry_data_t *data)
{
    if (data == NULL || s_socket < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    char buffer[TELEMETRY_BUFFER_SIZE];

    int length = snprintf(buffer, sizeof(buffer),
        "{"
            "\"type\":\"telemetry\","
            "\"sequence\":%" PRIu32 ","
            "\"ball_detected\":%s,"
            "\"ball_x\":%u,"
            "\"ball_y\":%u,"
            "\"measurement_age_ms\":%" PRIu64 ","
            "\"error_x_mm\":%.2f,"
            "\"error_y_mm\":%.2f,"
            "\"output_x_us\":%.2f,"
            "\"output_y_us\":%.2f,"
            "\"servo_x_us\":%u,"
            "\"servo_y_us\":%u,"
            "\"controller_enabled\":%s"
        "}",
        (unsigned long)data->sequence,
        data->ball_detected ? "true" : "false",
        (unsigned)data->ball_x,
        (unsigned)data->ball_y,
        (long long)data->measurement_age_ms,
        (double)data->error_x,
        (double)data->error_y,
        (double)data->output_x_us,
        (double)data->output_y_us,
        (unsigned)data->servo_x_us,
        (unsigned)data->servo_y_us,
        data->controller_enabled ? "true" : "false");

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