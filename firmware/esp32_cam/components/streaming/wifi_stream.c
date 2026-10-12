#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "wifi_stream.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define WIFI_SSID       "BallPlate-Camera"
#define WIFI_PASSWORD   "BallPlateTest123"
#define WIFI_CHANNEL    1
#define MAX_CONNECTIONS 2

#define UDP_PORT        9000

#define FRAME_WIDTH     160
#define FRAME_HEIGHT    120
#define FRAME_SIZE      (FRAME_WIDTH * FRAME_HEIGHT)

/*
 * UDP packet protocol:
 *
 * Header: 16 bytes
 *   0-3:   ASCII "GRAY"
 *   4-7:   Frame sequence number (uint32, big-endian)
 *   8-11:  Total frame size (uint32, big-endian)
 *   12-13: Packet index (uint16, big-endian, starts at 0)
 *   14-15: Total packets (uint16, big-endian)
 *
 * Payload: up to 1200 bytes of grayscale pixels.
 *
 * FRAME_SIZE = 19200 bytes
 * PACKET_PAYLOAD_SIZE = 1200 bytes
 * PACKETS_PER_FRAME = 16
 */
#define UDP_HEADER_SIZE         16
#define UDP_PACKET_PAYLOAD_SIZE 1200
#define UDP_PACKET_SIZE         (UDP_HEADER_SIZE + UDP_PACKET_PAYLOAD_SIZE)

#define PACKETS_PER_FRAME       ((FRAME_SIZE + UDP_PACKET_PAYLOAD_SIZE - 1) / UDP_PACKET_PAYLOAD_SIZE)

#define UDP_TASK_STACK_SIZE 6144
#define UDP_TASK_PRIORITY   5
#define FRAME_BUFFER_COUNT  2
#define FRAME_QUEUE_WAIT_MS 20

static const char *TAG = "WIFI_STREAM";
static QueueHandle_t free_frame_slots;
static QueueHandle_t ready_frame_slots;
static uint8_t *frame_buffers[FRAME_BUFFER_COUNT];

static void release_frame_slot(uint8_t slot)
{
    if (xQueueSend(free_frame_slots, &slot, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to return frame buffer slot %u", (unsigned)slot);
    }
}

static void destroy_frame_queues(void)
{
    if (free_frame_slots != NULL) {
        vQueueDelete(free_frame_slots);
        free_frame_slots = NULL;
    }

    if (ready_frame_slots != NULL) {
        vQueueDelete(ready_frame_slots);
        ready_frame_slots = NULL;
    }

    for (uint8_t i = 0; i < FRAME_BUFFER_COUNT; i++) {
        heap_caps_free(frame_buffers[i]);
        frame_buffers[i] = NULL;
    }
}

static esp_err_t initialize_frame_queues(void)
{
    free_frame_slots = xQueueCreate(FRAME_BUFFER_COUNT, sizeof(uint8_t));
    ready_frame_slots = xQueueCreate(FRAME_BUFFER_COUNT, sizeof(uint8_t));
    if (free_frame_slots == NULL || ready_frame_slots == NULL) {
        destroy_frame_queues();
        return ESP_ERR_NO_MEM;
    }

    for (uint8_t i = 0; i < FRAME_BUFFER_COUNT; i++) {
        frame_buffers[i] = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (frame_buffers[i] == NULL) {
            destroy_frame_queues();
            return ESP_ERR_NO_MEM;
        }

        if (xQueueSend(free_frame_slots, &i, 0) != pdTRUE) {
            destroy_frame_queues();
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t wifi_stream_submit_frame(const uint8_t *pixels, size_t frame_length)
{
    if (pixels == NULL || frame_length != FRAME_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (free_frame_slots == NULL || ready_frame_slots == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t slot;
    if (xQueueReceive(free_frame_slots, &slot, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(frame_buffers[slot], pixels, frame_length);

    if (xQueueSend(ready_frame_slots, &slot, 0) != pdTRUE) {
        release_frame_slot(slot);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t initialize_nvs(void)
{
    esp_err_t error = nvs_flash_init();

    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        error = nvs_flash_erase();
        if (error != ESP_OK) {
            return error;
        }

        error = nvs_flash_init();
    }

    return error;
}

static size_t build_frame_packet(
    uint8_t *packet,
    const uint8_t *pixels,
    size_t frame_length,
    uint32_t sequence,
    uint16_t packet_index)
{
    size_t offset = (size_t)packet_index * UDP_PACKET_PAYLOAD_SIZE;
    size_t payload_length = frame_length - offset;

    if (payload_length > UDP_PACKET_PAYLOAD_SIZE) {
        payload_length = UDP_PACKET_PAYLOAD_SIZE;
    }

    memcpy(packet, "GRAY", 4);

    uint32_t sequence_be = htonl(sequence);
    uint32_t length_be = htonl((uint32_t)frame_length);

    uint16_t index_be = htons(packet_index);
    uint16_t count_be = htons(PACKETS_PER_FRAME);

    memcpy(packet + 4, &sequence_be, sizeof(sequence_be));
    memcpy(packet + 8, &length_be, sizeof(length_be));
    memcpy(packet + 12, &index_be, sizeof(index_be));
    memcpy(packet + 14, &count_be, sizeof(count_be));
    memcpy(packet + UDP_HEADER_SIZE, pixels + offset, payload_length);

    return UDP_HEADER_SIZE + payload_length;
}

/*
 * UDP does not guarantee packet delivery. The receiver must
 * reconstruct frames using the sequence and packet indices.
 */
static int send_frame_udp(
    int sock,
    const struct sockaddr_in *client_addr,
    const uint8_t *pixels,
    size_t frame_length,
    uint32_t sequence)
{
    uint8_t packet[UDP_PACKET_SIZE];

    for (uint16_t i = 0; i < PACKETS_PER_FRAME; i++) {
        size_t packet_length = build_frame_packet(packet, pixels, frame_length, sequence, i);
        ssize_t sent = sendto(
            sock,
            packet,
            packet_length,
            0,
            (const struct sockaddr *)client_addr,
            sizeof(*client_addr)
        );

        if (sent < 0 || (size_t)sent != packet_length) {
            ESP_LOGW(
                TAG,
                "UDP send failed: errno=%d frame=%" PRIu32
                " packet=%u",
                errno,
                sequence,
                (unsigned)i
            );

            return -1;
        }
    }

    return 0;
}

/*
 * Accept HELLO datagrams during streaming.
 *
 * A PC can register or update its receiving port by sending
 * the four ASCII bytes "HELO" to 192.168.4.1:9000.
 */
static void update_client_address(int sock, struct sockaddr_in *client_addr)
{
    uint8_t buffer[32];
    struct sockaddr_in sender = {0};
    socklen_t sender_len = sizeof(sender);

    ssize_t received = recvfrom(
        sock,
        buffer,
        sizeof(buffer),
        MSG_DONTWAIT,
        (struct sockaddr *)&sender,
        &sender_len
    );

    if (received == 4 && memcmp(buffer, "HELO", 4) == 0) {
        *client_addr = sender;

        ESP_LOGI(
            TAG,
            "UDP client registered: %s:%u",
            inet_ntoa(sender.sin_addr),
            (unsigned)ntohs(sender.sin_port)
        );
    }
}

static bool receive_latest_frame_slot(uint8_t *slot)
{
    if (xQueueReceive(ready_frame_slots, slot,pdMS_TO_TICKS(FRAME_QUEUE_WAIT_MS)) != pdTRUE) {
        return false;
    }

    uint8_t newer_slot;
    while (xQueueReceive(ready_frame_slots, &newer_slot, 0) == pdTRUE) {
        release_frame_slot(*slot);
        *slot = newer_slot;
    }

    return true;
}

static void stream_client_udp(int sock,struct sockaddr_in client_addr)
{
    uint32_t sequence = 0;

    while (1) {
        update_client_address(sock, &client_addr);

        uint8_t slot;
        if (!receive_latest_frame_slot(&slot)) {
            continue;
        }

        int result = send_frame_udp(
            sock,
            &client_addr,
            frame_buffers[slot],
            FRAME_SIZE,
            sequence
        );

        release_frame_slot(slot);
        sequence++;

        if (result < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/*
 * UDP server:
 * - Bind port 9000.
 * - Wait for a PC registration datagram.
 * - Stream camera frames to the sender's IP and port.
 */
static void udp_server_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket creation failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr = {0};

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno=%d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", UDP_PORT);

    while (1) {
        uint8_t buffer[32];
        struct sockaddr_in client_addr = {0};
        socklen_t client_len = sizeof(client_addr);

        ssize_t received = recvfrom(
            sock,
            buffer,
            sizeof(buffer),
            0,
            (struct sockaddr *)&client_addr,
            &client_len
        );

        if (received < 0) {
            ESP_LOGW(TAG, "UDP receive failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (received != 4 || memcmp(buffer, "HELO", 4) != 0) {
            continue;
        }

        ESP_LOGI(
            TAG,
            "Client registered: %s:%u",
            inet_ntoa(client_addr.sin_addr),
            (unsigned)ntohs(client_addr.sin_port)
        );

        stream_client_udp(sock, client_addr);
    }
}

esp_err_t wifi_stream_init(void)
{
    esp_err_t err = initialize_nvs();

    if (err != ESP_OK) {
        return err;
    }

    err = esp_netif_init();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi AP interface");
        return ESP_FAIL;
    }

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();

    err = esp_wifi_init(&config);

    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t ap_config = {0};

    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", WIFI_SSID);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s", WIFI_PASSWORD);

    ap_config.ap.ssid_len = strlen(WIFI_SSID);
    ap_config.ap.channel = WIFI_CHANNEL;
    ap_config.ap.max_connection = MAX_CONNECTIONS;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_AP);

    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);

    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Disable Wi-Fi power saving for lower latency.
     */
    err = esp_wifi_set_ps(WIFI_PS_NONE);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not disable Wi-Fi power saving");
    }

    err = initialize_frame_queues();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize frame queue: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t task_result = xTaskCreate(
        udp_server_task,
        "udp_frame_server",
        UDP_TASK_STACK_SIZE,
        NULL,
        UDP_TASK_PRIORITY,
        NULL
    );

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UDP server task");
        destroy_frame_queues();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Wi-Fi access point started");
    ESP_LOGI(TAG, "SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "UDP port: %d", UDP_PORT);
    ESP_LOGI(TAG, "Server IP: 192.168.4.1");

    return ESP_OK;
}
