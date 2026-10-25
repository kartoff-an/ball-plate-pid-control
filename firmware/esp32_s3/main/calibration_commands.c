#include "calibration_commands.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "pca9685.h"
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CALIBRATION_PORT        9004
#define MIN_TEST_PULSE_US       1450
#define MAX_TEST_PULSE_US       1550

#define CALIBRATION_TASK_STACK        3072
#define CALIBRATION_TASK_PRIORITY     4

static const char *TAG = "SERVO_CAL";

static void calibration_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server = {0};
    server.sin_family = AF_INET;
    server.sin_port = htons(CALIBRATION_PORT);
    server.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening on UDP port %d", CALIBRATION_PORT);

    char buffer[64];

    while (true) {
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);

        int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client, &client_len);
        if (len <= 0) {
            continue;
        }

        buffer[len] = '\0';

        unsigned channel;
        unsigned pulse;
        char extra;

        if (sscanf(buffer, "SERVO %u %u %c", &channel, &pulse, &extra) != 2) {
            ESP_LOGW(TAG, "Invalid command");
            continue;
        }

        if (channel > 1 || pulse < MIN_TEST_PULSE_US || pulse > MAX_TEST_PULSE_US) {
            ESP_LOGW(TAG, "Rejected channel/pulse: %u %u", channel, pulse);
            continue;
        }

        esp_err_t error = pca9685_set_pulse_us((uint8_t)channel, (uint16_t)pulse);
        if (error == ESP_OK) {
            ESP_LOGI(TAG, "Servo %u -> %u us", channel, pulse);
        }
        else {
            ESP_LOGE(TAG, "Servo command failed: %s", esp_err_to_name(error));
        }
    }
}

esp_err_t calibration_commands_init(void)
{
    BaseType_t result = xTaskCreate(
        calibration_task,
        "servo_cal",
        CALIBRATION_TASK_STACK,
        NULL,
        CALIBRATION_TASK_PRIORITY,
        NULL
    );

    return result == pdPASS ? ESP_OK : ESP_FAIL;
}