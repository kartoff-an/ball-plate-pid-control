#include "config_commands.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_err.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pid_controller.h"

#define CONFIG_COMMAND_PORT       9002
#define CONFIG_TASK_STACK         4096
#define CONFIG_TASK_PRIORITY      4
#define CONFIG_BUFFER_SIZE        512

static const char *TAG = "CONFIG_CMD";

static void send_ack(int sock, const struct sockaddr_in *client, socklen_t client_len, bool ok, const char *message)
{
    char response[192];

    snprintf(
        response,
        sizeof(response),
        "{\"type\":\"ack\",\"ok\":%s,\"message\":\"%s\"}",
        ok ? "true" : "false",
        message
    );

    sendto(sock, response, strlen(response), 0, (const struct sockaddr *)client, client_len);
}

static bool get_number(const cJSON *object, const char *key, double *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);

    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) {
        return false;
    }

    *value = item->valuedouble;
    return true;
}

static bool parse_gains(const cJSON *axis_object, pid_gains_t *gains)
{
    double kp;
    double ki;
    double kd;

    if (!cJSON_IsObject(axis_object) ||
        !get_number(axis_object, "kp", &kp) ||
        !get_number(axis_object, "ki", &ki) ||
        !get_number(axis_object, "kd", &kd)) {
        return false;
    }

    if (kp < 0.0 || ki < 0.0 || kd < 0.0 || kp > 10.0 || ki > 2.0 || kd > 10.0) {
        return false;
    }

    gains->kp = (float)kp;
    gains->ki = (float)ki;
    gains->kd = (float)kd;

    return true;
}

static void handle_target(int sock, const cJSON *root, const struct sockaddr_in *client, socklen_t client_len)
{
    double x;
    double y;

    if (!get_number(root, "x", &x) || !get_number(root, "y", &y)) {
        send_ack(sock, client, client_len, false, "target requires finite numeric x and y");
        return;
    }

    if (x < -110.0 || x > 110.0 || y < -110.0 || y > 110.0) {
        send_ack(sock, client, client_len, false, "target must be within -110 to 110 mm");
        return;
    }

    esp_err_t err = pid_controller_set_target((float)x, (float)y);

    if (err != ESP_OK) {
        send_ack(sock, client, client_len, false, "failed to update target");
        return;
    }

    ESP_LOGI(TAG, "Target updated: x=%.2f mm y=%.2f mm", x, y);
    send_ack(sock, client, client_len, true, "target updated");
}

static void handle_pid_config(int sock, const cJSON *root, const struct sockaddr_in *client, socklen_t client_len)
{
    const cJSON *axis_x = cJSON_GetObjectItemCaseSensitive(root, "axis_x");
    const cJSON *axis_y = cJSON_GetObjectItemCaseSensitive(root, "axis_y");

    pid_gains_t gains_x;
    pid_gains_t gains_y;

    /*
     * Validate both axes before modifying either one.
     * This avoids applying only half of a configuration
     * when one axis contains invalid values.
     */
    if (!parse_gains(axis_x, &gains_x) || !parse_gains(axis_y, &gains_y)) {
        send_ack(sock, client, client_len, false, "invalid PID gains or gain limits exceeded");
        return;
    }

    esp_err_t err = pid_controller_set_gains(PID_AXIS_X, &gains_x);

    if (err != ESP_OK) {
        send_ack(sock, client, client_len, false, "failed to update X gains");
        return;
    }

    err = pid_controller_set_gains(PID_AXIS_Y, &gains_y);

    if (err != ESP_OK) {
        send_ack(sock, client, client_len, false, "failed to update Y gains");
        return;
    }

    ESP_LOGI(
        TAG,
        "PID X: kp=%.3f ki=%.3f kd=%.3f; "
        "Y: kp=%.3f ki=%.3f kd=%.3f",
        (double)gains_x.kp, (double)gains_x.ki, (double)gains_x.kd,
        (double)gains_y.kp, (double)gains_y.ki, (double)gains_y.kd
    );

    send_ack(sock, client, client_len, true, "PID gains updated");
}

static void handle_control(int sock, const cJSON *root, const struct sockaddr_in *client, socklen_t client_len)
{
    const cJSON *command = cJSON_GetObjectItemCaseSensitive(root, "command");

    if (!cJSON_IsString(command) || command->valuestring == NULL) {
        send_ack(sock, client, client_len, false, "control requires a command string");
        return;
    }

    esp_err_t err;

    if (strcmp(command->valuestring, "start") == 0) {
        err = pid_controller_start();

        if (err == ESP_OK) {
            send_ack(sock, client, client_len, true, "controller started");
        } else {
            send_ack(sock, client, client_len, false, "failed to start controller");
        }

    } else if (strcmp(command->valuestring, "stop") == 0) {
        err = pid_controller_stop();

        if (err == ESP_OK) {
            send_ack(sock, client, client_len, true, "controller stopped");
        } else {
            send_ack(sock, client, client_len, false, "failed to stop controller");
        }

    } else if (strcmp(command->valuestring, "reset_pid") == 0) {
        err = pid_controller_reset();

        if (err == ESP_OK) {
            send_ack(sock, client, client_len, true, "PID state reset");
        } else {
            send_ack(sock, client, client_len, false, "failed to reset PID state");
        }

    } else {
        send_ack(sock, client, client_len, false, "unknown control command");
    }
}

static void handle_message(int sock, char *buffer, const struct sockaddr_in *client, socklen_t client_len)
{
    cJSON *root = cJSON_Parse(buffer);

    if (root == NULL || !cJSON_IsObject(root)) {
        if (root != NULL) {
            cJSON_Delete(root);
        }

        send_ack(sock, client, client_len, false, "invalid JSON object");
        return;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");

    if (!cJSON_IsString(type) || type->valuestring == NULL) {
        send_ack(sock, client, client_len, false, "missing message type");
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type->valuestring, "target") == 0) {
        handle_target(sock, root, client, client_len);

    } else if (strcmp(type->valuestring, "pid_config") == 0) {
        handle_pid_config(sock, root, client, client_len);

    } else if (strcmp(type->valuestring, "control") == 0) {
        handle_control(sock, root, client, client_len);

    } else {
        send_ack(sock, client, client_len, false, "unknown message type");
    }

    cJSON_Delete(root);
}

static void config_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(CONFIG_COMMAND_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d", CONFIG_COMMAND_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "JSON configuration service listening on UDP %d", CONFIG_COMMAND_PORT);

    while (true) {
        char buffer[CONFIG_BUFFER_SIZE];

        struct sockaddr_in client_addr = {0};
        socklen_t client_len = sizeof(client_addr);

        int received = recvfrom(
            sock,
            buffer,
            sizeof(buffer) - 1,
            0,
            (struct sockaddr *)&client_addr,
            &client_len
        );

        if (received < 0) {
            ESP_LOGW(TAG, "UDP receive failed");
            continue;
        }

        buffer[received] = '\0';

        ESP_LOGD(TAG, "Received configuration message: %s", buffer);

        handle_message(sock, buffer, &client_addr, client_len);
    }
}

esp_err_t config_commands_init(void)
{
    BaseType_t result = xTaskCreate(
        config_task,
        "config_commands",
        CONFIG_TASK_STACK,
        NULL,
        CONFIG_TASK_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create configuration task");
        return ESP_FAIL;
    }

    return ESP_OK;
}