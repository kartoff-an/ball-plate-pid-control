/**
 * @file calibration_commands.h
 * @brief UDP interface for runtime servo calibration and PID tuning.
 *
 * Provides a command listener that allows a PC or another network
 * device to adjust servo pulse widths and PID gains without reflashing
 * the ESP32-S3 firmware.
 *
 * Supported UDP commands:
 *
 * - SERVO <channel> <pulse_us>
 * - GET
 * - SET X <kp> <ki> <kd>
 * - SET Y <kp> <ki> <kd>
 * - SET BOTH <kp> <ki> <kd>
 *
 * The listener runs in its own FreeRTOS task and receives commands
 * on the configured calibration UDP port.
 */

#include "calibration_commands.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "esp_err.h"
#include "esp_log.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pca9685.h"
#include "pid_controller.h"

#define CALIBRATION_PORT             9004
#define CALIBRATION_TASK_STACK       3072
#define CALIBRATION_TASK_PRIORITY    4
#define CALIBRATION_BUFFER_SIZE      128

#define MIN_TEST_PULSE_US             1000
#define MAX_TEST_PULSE_US             2500
#define SERVO_CHANNEL_COUNT          2

static const char *TAG = "SERVO_CAL";

static void send_response(int sock, const char *response, const struct sockaddr_in *client, socklen_t client_len)
{
    sendto(sock, response, strlen(response), 0, (const struct sockaddr *)client, client_len);
}

static void send_gains(int sock, const struct sockaddr_in *client, socklen_t client_len)
{
    pid_gains_t gains_x;
    pid_gains_t gains_y;

    esp_err_t err_x = pid_controller_get_gains(PID_AXIS_X, &gains_x);

    esp_err_t err_y = pid_controller_get_gains(PID_AXIS_Y, &gains_y);

    if (err_x != ESP_OK || err_y != ESP_OK) {
        send_response(sock, "ERR unable to read PID gains\n", client, client_len);
        return;
    }

    char response[128];

    snprintf(
        response,
        sizeof(response),
        "OK X %.3f %.3f %.3f Y %.3f %.3f %.3f\n",
        (double)gains_x.kp,
        (double)gains_x.ki,
        (double)gains_x.kd,
        (double)gains_y.kp,
        (double)gains_y.ki,
        (double)gains_y.kd
    );

    send_response(sock, response, client, client_len);
}

static void send_kalman_config(int sock, const struct sockaddr_in *client, socklen_t client_len)
{
    float measurement_variance;
    float acceleration_variance;

    esp_err_t err = pid_controller_get_kalman_config(&measurement_variance, &acceleration_variance);

    if (err != ESP_OK) {
        send_response(sock, "ERR unable to read Kalman config\n", client, client_len);
        return;
    }

    char response[128];

    snprintf(
        response,
        sizeof(response),
        "OK KALMAN %.3f %.3f\n",
        (double)measurement_variance,
        (double)acceleration_variance
    );

    send_response(sock, response, client, client_len);
}

static void handle_command(int sock, char *command, const struct sockaddr_in *client, socklen_t client_len)
{
    char response[128];

    while (isspace((unsigned char)*command)) {
        command++;
    }

    size_t length = strlen(command);

    while (length > 0 && isspace((unsigned char)command[length - 1])) {
        command[--length] = '\0';
    }

    if (length == 0) {
        send_response(sock, "ERR empty command\n", client, client_len);
        return;
    }

    /*
     * 1. Direct servo calibration
     *
     * Format: SERVO <channel> <pulse_us>
     * Example: SERVO 0 1500
     */

    unsigned int channel;
    unsigned int pulse_us;
    char extra;

    int parsed = sscanf(command, "SERVO %u %u %c", &channel, &pulse_us, &extra);

    if (strncmp(command, "SERVO", 5) == 0 && (command[5] == '\0' || isspace((unsigned char)command[5]))) {
        if (parsed != 2) {
            send_response(sock, "ERR usage: SERVO <channel> <pulse_us>\n", client, client_len);
            return;
        }

        if (channel >= SERVO_CHANNEL_COUNT) {
            send_response(sock, "ERR channel must be 0 or 1\n", client, client_len);
            return;
        }

        if (pulse_us < MIN_TEST_PULSE_US || pulse_us > MAX_TEST_PULSE_US) {

            snprintf(response, sizeof(response), "ERR pulse must be %d-%d us\n", MIN_TEST_PULSE_US, MAX_TEST_PULSE_US
            );

            send_response(sock, response, client, client_len);
            return;
        }

        esp_err_t err = pca9685_set_pulse_us((uint8_t)channel,(uint16_t)pulse_us);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Servo command failed: channel=%u pulse=%u err=%s", channel, pulse_us, esp_err_to_name(err));

            send_response(sock, "ERR failed to set servo pulse\n", client, client_len);
            return;
        }

        ESP_LOGI(TAG, "Servo set: channel=%u pulse=%u us", channel, pulse_us);

        snprintf(response, sizeof(response), "OK SERVO %u %u\n", channel, pulse_us);

        send_response(sock, response, client, client_len);
        return;
    }

    /*
     * 2. Get current PID gains
     *
     * Format: GET
     */

    if (strcmp(command, "GET") == 0) {
        send_gains(sock, client, client_len);
        return;
    }

    if (strcmp(command, "GET KALMAN") == 0) {
        send_kalman_config(sock, client, client_len);
        return;
    }

    /*
     * 3. Update PID gains
     *
     * Format: SET X <kp> <ki> <kd>
     *         SET Y <kp> <ki> <kd>
     *         SET BOTH <kp> <ki> <kd>
     */

    if (strncmp(command, "SET", 3) == 0 && (command[3] == '\0' || isspace((unsigned char)command[3]))) {

        char axis_name[8];
        float kp;
        float ki;
        float kd;

        parsed = sscanf(
            command,
            "SET %7s %f %f %f %c",
            axis_name,
            &kp,
            &ki,
            &kd,
            &extra
        );

        if (parsed != 4) {
            send_response(sock, "ERR usage: SET X|Y|BOTH <kp> <ki> <kd>\n", client, client_len);
            return;
        }

        pid_axis_id_t axis;

        if (strcmp(axis_name, "X") == 0) {
            axis = PID_AXIS_X;
        } else if (strcmp(axis_name, "Y") == 0) {
            axis = PID_AXIS_Y;
        } else if (strcmp(axis_name, "BOTH") == 0) {
            axis = PID_AXIS_BOTH;
        } else {
            send_response(sock, "ERR axis must be X, Y, or BOTH\n", client, client_len);
            return;
        }

        pid_gains_t gains = {
            .kp = kp,
            .ki = ki,
            .kd = kd
        };

        esp_err_t err = pid_controller_set_gains(axis, &gains);

        if (err != ESP_OK) {
            send_response(sock, "ERR invalid PID gains or axis\n", client, client_len);
            return;
        }

        ESP_LOGI(TAG, "PID gains updated: axis=%s kp=%.3f ki=%.3f kd=%.3f", axis_name, (double)kp, (double)ki, (double)kd);

        send_response(sock, "OK gains updated\n", client, client_len);

        return;
    }

    send_response(sock, "ERR unknown command\n", client, client_len);
}

static void calibration_task(void *arg)
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
    server_addr.sin_port = htons(CALIBRATION_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d", CALIBRATION_PORT);

        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening for servo and PID commands on UDP port %d", CALIBRATION_PORT);

    while (true) {
        char buffer[CALIBRATION_BUFFER_SIZE];

        struct sockaddr_in client_addr;
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

        ESP_LOGI(TAG, "Received: %s", buffer);

        handle_command(sock, buffer, &client_addr, client_len);
    }

    /* Normally unreachable; retained for completeness. */
    close(sock);
    vTaskDelete(NULL);
}


esp_err_t calibration_commands_init(void)
{
    BaseType_t result = xTaskCreate(
        calibration_task,
        "servo_pid_cal",
        CALIBRATION_TASK_STACK,
        NULL,
        CALIBRATION_TASK_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create calibration task");
        return ESP_FAIL;
    }

    return ESP_OK;
}
