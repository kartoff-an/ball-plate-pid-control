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

#define VISION_TUNER_TASK_STACK_SIZE    3072
#define VISION_TUNER_TASK_PRIORITY      4

#define PACKET_SIZE                     33
static const char *TAG = "VISION_TUNER";

/*
 * Wire format:
 *  4 bytes magic "VTN1"
 *  4 bytes sequence
 *  1 byte threshold
 *  2 bytes min_area
 *  2 bytes max_area
 *  5 IEEE-754 float32 values: min_aspect, max_aspect, min_fill,
 *  max_jump_px, smoothing_alpha
 *
 * ACK format: "ACK1" + uint32 sequence + uint8 status (0=applied).
 */
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

        if (received == PACKET_SIZE && memcmp(packet, "VTN1", 4) == 0) {
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

            if (ball_detection_set_config(&cfg) == ESP_OK) {
                status = 0;
                ESP_LOGI(TAG, "Applied seq=%" PRIu32 " threshold=%u area=%" PRIu32 "..%" PRIu32,
                         sequence, cfg.threshold, cfg.min_area, cfg.max_area);
            }
        }

        uint8_t ack[9] = {'A','C','K','1',0,0,0,0,status};
        write_u32_be(ack + 4, sequence);
        (void)sendto(sock, ack, sizeof(ack), 0,
                     (struct sockaddr *)&peer, peer_len);
    }
}

esp_err_t vision_tuner_start(void)
{
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
