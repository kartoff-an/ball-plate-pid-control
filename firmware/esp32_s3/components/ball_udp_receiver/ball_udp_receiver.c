#include "ball_udp_receiver.h"

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define BALL_DETECTION_UDP_PORT       9002
#define BALL_DETECTION_PACKET_SIZE    17

#define BALL_DETECTION_IMAGE_WIDTH    160
#define BALL_DETECTION_IMAGE_HEIGHT   120

#define BALL_RECEIVER_TASK_STACK_SIZE 3072
#define BALL_RECEIVER_TASK_PRIORITY   6

#define BALL_MEASUREMENT_QUEUE_LENGTH 1

static const char *TAG = "BALL_UDP_RX";

static QueueHandle_t latest_measurement_queue;

/* Validate and decode a CAM detection packet. */
static bool decode_detection_packet(
    const uint8_t *packet,
    size_t packet_size,
    ball_measurement_t *measurement)
{
    if (packet == NULL || measurement == NULL) {
        return false;
    }
    if (packet_size != BALL_DETECTION_PACKET_SIZE) {
        return false;
    }
    if (memcmp(packet, "BALL", 4) != 0) {
        return false;
    }
    if (packet[8] > 1U) {
        return false;
    }

    measurement->frame_sequence = ((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) |
        ((uint32_t)packet[6] << 8) |
        (uint32_t)packet[7];
    measurement->center_x = ((uint16_t)packet[9] << 8) | (uint16_t)packet[10];
    measurement->center_y = ((uint16_t)packet[11] << 8) | (uint16_t)packet[12];
    measurement->ball_area = ((uint32_t)packet[13] << 24) | ((uint32_t)packet[14] << 16) |
        ((uint32_t)packet[15] << 8) |
        (uint32_t)packet[16];
    measurement->received_at_us = esp_timer_get_time();

    if (measurement->is_detected &&
        (measurement->center_x >= BALL_DETECTION_IMAGE_WIDTH ||
         measurement->center_y >= BALL_DETECTION_IMAGE_HEIGHT)) {
        return false;
    }

    return true;
}

/* Receive detection packets from the ESP32-CAM. */
static void ball_udp_receiver_task(void *context)
{
    (void)context;

    uint8_t packet[64];
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0) {
        ESP_LOGE(TAG, "Socket creation failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_address = {0};

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(BALL_DETECTION_UDP_PORT);
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socket_fd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno=%d", errno);
        close(socket_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening for ball detection on UDP port %d", BALL_DETECTION_UDP_PORT);

    bool have_sequence = false;
    uint32_t last_sequence = 0;

    while (true) {
        struct sockaddr_in sender_address = {0};
        socklen_t sender_address_length = sizeof(sender_address);

        ssize_t received_byte_count = recvfrom(
            socket_fd,
            packet,
            sizeof(packet),
            0,
            (struct sockaddr *)&sender_address,
            &sender_address_length
        );

        if (received_byte_count < 0) {
            ESP_LOGW(TAG, "UDP receive failed: errno=%d", errno);
            continue;
        }

        ball_measurement_t measurement = {0};

        if (!decode_detection_packet(packet, (size_t)received_byte_count, &measurement)) {
            ESP_LOGD(TAG, "Ignored invalid detection packet");
            continue;
        }

        /*
         * Reject duplicate or out-of-order frames.
         * Signed sequence difference supports normal uint32_t wraparound.
         */
        if (have_sequence && (int32_t)(measurement.frame_sequence - last_sequence) <= 0) {
            continue;
        }

        last_sequence = measurement.frame_sequence;
        have_sequence = true;

        /*
         * Keep only the newest measurement.
         * The receiver never waits for the PID task to consume a packet.
         */
        if (xQueueOverwrite(latest_measurement_queue, &measurement) != pdPASS) {
            ESP_LOGW(TAG, "Failed to publish ball measurement");
        }
    }
}

/* Initialize the measurement queue and UDP receiver task. */
esp_err_t ball_udp_receiver_init(void)
{
    if (latest_measurement_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    latest_measurement_queue = xQueueCreate(BALL_MEASUREMENT_QUEUE_LENGTH, sizeof(ball_measurement_t));

    if (latest_measurement_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create measurement queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_result = xTaskCreate(
        ball_udp_receiver_task,
        "ball_udp_receiver",
        BALL_RECEIVER_TASK_STACK_SIZE,
        NULL,
        BALL_RECEIVER_TASK_PRIORITY,
        NULL
    );

    if (task_result != pdPASS) {
        vQueueDelete(latest_measurement_queue);
        latest_measurement_queue = NULL;

        ESP_LOGE(TAG, "Failed to create receiver task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Ball detection receiver initialized");

    return ESP_OK;
}

/* Read the most recently received measurement without blocking. */
bool ball_udp_receiver_get_latest(
    ball_measurement_t *measurement)
{
    if (measurement == NULL || latest_measurement_queue == NULL) {
        return false;
    }

    return xQueuePeek(latest_measurement_queue, measurement, 0) == pdTRUE;
}