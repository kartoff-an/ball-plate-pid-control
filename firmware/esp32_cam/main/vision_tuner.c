/**
 * @file vision_tuner.c
 * @brief UDP-based runtime configuration for ball detection.
 *
 * Receives binary configuration packets from the PC over UDP, validates
 * and applies the supplied settings through the ball detection module,
 * then sends an acknowledgment to the packet sender.
 *
 * Packet format:
 * - 4 bytes: Magic identifier ("VTN1").
 * - 4 bytes: Sequence number (big-endian uint32).
 * - 1 byte: Detection threshold.
 * - 2 bytes: Minimum component area (big-endian uint16).
 * - 2 bytes: Maximum component area (big-endian uint16).
 * - 4 bytes: Minimum aspect ratio (IEEE-754 float32, big-endian).
 * - 4 bytes: Maximum aspect ratio (IEEE-754 float32, big-endian).
 * - 4 bytes: Minimum fill ratio (IEEE-754 float32, big-endian).
 * - 4 bytes: Maximum tracking displacement (IEEE-754 float32, big-endian).
 * - 4 bytes: Position smoothing factor (IEEE-754 float32, big-endian).
 * - 1 byte: Command (0 = APPLY, 1 = SAVE)
 *
 * Total packet size: 34 bytes.
 *
 * Acknowledgment format:
 * - 4 bytes: Magic identifier ("ACK1").
 * - 4 bytes: Received sequence number (big-endian uint32).
 * - 1 byte: Status (0 = applied, nonzero = rejected).
 *
 * @note Configuration validation is performed by
 *       ball_detection_set_config().
 * @note The UDP listener runs in its own FreeRTOS task.
 */

#include "vision_tuner.h"

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "ball_detection.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define VISION_TUNER_TASK_STACK_SIZE    3072
#define VISION_TUNER_TASK_PRIORITY      4

#define NVS_NAMESPACE                   "vision"
#define NVS_CONFIG_KEY                  "config"
#define VISION_PACKET_SIZE              34

#define VISION_CMD_APPLY                0
#define VISION_CMD_SAVE                 1

static const char *TAG = "VISION_TUNER";

static uint16_t read_u16_be(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static float read_f32_be(const uint8_t *p)
{
    uint32_t bits = read_u32_be(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void write_u32_be(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static void vision_tuner_task(void *arg)
{
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in local = {0};
    local.sin_family = AF_INET;
    local.sin_port = htons(VISION_TUNER_UDP_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind port %d failed", VISION_TUNER_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening for runtime tuning on UDP port %d", VISION_TUNER_UDP_PORT);

    for (;;) {
        uint8_t packet[64];
        struct sockaddr_in peer = {0};
        socklen_t peer_len = sizeof(peer);
        const int received = recvfrom(sock, packet, sizeof(packet), 0,
                                      (struct sockaddr *)&peer, &peer_len);
        if (received < 0) continue;

        uint32_t sequence = 0;
        uint8_t status = 1;

        if (received == VISION_PACKET_SIZE && memcmp(packet, "VTN1", 4) == 0) {
            sequence = read_u32_be(packet + 4);

            ball_detection_config_t cfg;
            cfg.threshold = packet[8];
            cfg.min_area = read_u16_be(packet + 9);
            cfg.max_area = read_u16_be(packet + 11);
            cfg.min_aspect_ratio = read_f32_be(packet + 13);
            cfg.max_aspect_ratio = read_f32_be(packet + 17);
            cfg.min_fill_ratio = read_f32_be(packet + 21);
            cfg.max_jump_px = read_f32_be(packet + 25);
            cfg.smoothing_alpha = read_f32_be(packet + 29);

            const uint8_t command = packet[33];

             if (command == VISION_CMD_APPLY || command == VISION_CMD_SAVE) {
                esp_err_t error = ball_detection_set_config(&cfg);

                if (error == ESP_OK) {
                    status = 0;
                    ESP_LOGI(TAG, "Applied seq=%" PRIu32 " threshold=%u area=%" PRIu32 "..%" PRIu32,
                            sequence, cfg.threshold, cfg.min_area, cfg.max_area);

                    if (command == VISION_CMD_SAVE) {
                        error = vision_tuner_save(&cfg);

                        if (error != ESP_OK) {
                            status = 2;

                            ESP_LOGE(TAG,
                                    "Failed to save vision config: %s",
                                    esp_err_to_name(error));
                        } else {
                            ESP_LOGI(TAG,
                                    "Vision configuration saved to NVS");
                        }
                    }
                }
            } else {
                ESP_LOGW(TAG, "Invalid tuning command: %u", command);
            }
        }

        uint8_t ack[9] = {'A','C','K','1',0,0,0,0,status};
        write_u32_be(ack + 4, sequence);
        (void)sendto(sock, ack, sizeof(ack), 0,
                     (struct sockaddr *)&peer, peer_len);
    }
}

static esp_err_t vision_tuner_load(void)
{
    nvs_handle_t handle;
    esp_err_t error = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (error == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved vision configuration; using defaults");
        return ESP_OK;
    }

    if (error != ESP_OK) {
        return error;
    }

    ball_detection_config_t cfg;
    size_t size = sizeof(cfg);

    error = nvs_get_blob(handle, NVS_CONFIG_KEY, &cfg, &size);

    nvs_close(handle);

    if (error == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved vision configuration; using defaults");
        return ESP_OK;
    }

    if (error != ESP_OK) {
        return error;
    }

    if (size != sizeof(cfg)) {
        ESP_LOGW(TAG, "Saved vision configuration has an incompatible size");
        return ESP_ERR_INVALID_SIZE;
    }

    error = ball_detection_set_config(&cfg);

    if (error == ESP_OK) {
        ESP_LOGI(TAG, "Loaded vision configuration from NVS");
    } else {
        ESP_LOGW(TAG, "Saved vision configuration is invalid; using defaults");
    }

    return error;
}

esp_err_t vision_tuner_start(void)
{
    esp_err_t error = vision_tuner_load();

    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load saved config: %s", esp_err_to_name(error));
    }

    BaseType_t result = xTaskCreate(
        vision_tuner_task,
        "vision_tuner",
        VISION_TUNER_TASK_STACK_SIZE,
        NULL,
        VISION_TUNER_TASK_PRIORITY,
        NULL
    );
    return result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t vision_tuner_save(const ball_detection_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t error = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (error != ESP_OK) {
        return error;
    }

    error = nvs_set_blob(handle, NVS_CONFIG_KEY, config, sizeof(*config));

    if (error == ESP_OK) {
        error = nvs_commit(handle);
    }

    nvs_close(handle);
    return error;
}