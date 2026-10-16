#include <inttypes.h>
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
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define UDP_PORT                9000

#define FRAME_WIDTH             160
#define FRAME_HEIGHT            120
#define FRAME_SIZE              (FRAME_WIDTH * FRAME_HEIGHT)

#define FRAME_HEADER_SIZE       16
#define FRAME_CHUNK_SIZE        1200
#define FRAME_PACKET_SIZE       (FRAME_HEADER_SIZE + FRAME_CHUNK_SIZE)
#define PACKETS_PER_FRAME       ((FRAME_SIZE + FRAME_CHUNK_SIZE - 1) / FRAME_CHUNK_SIZE)

#define DETECTION_PACKET_SIZE   17

#define UDP_TASK_STACK_SIZE     6144
#define UDP_TASK_PRIORITY       5
#define FRAME_BUFFER_COUNT      2
#define FRAME_QUEUE_WAIT_MS     20

static const char *TAG = "WIFI_STREAM";

typedef struct {
    uint32_t sequence;
    uint8_t detected;
    uint16_t center_x;
    uint16_t center_y;
    uint32_t area;
} detection_message_t;

static QueueHandle_t free_frame_slots;
static QueueHandle_t ready_frame_slots;
static QueueHandle_t detection_queue;

static uint8_t *frame_buffers[FRAME_BUFFER_COUNT];
static uint32_t frame_sequences[FRAME_BUFFER_COUNT];

/* Return a frame buffer to the available-buffer queue. */
static void release_frame_slot(uint8_t slot)
{
    if (xQueueSend(free_frame_slots, &slot, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to return frame buffer slot %u", (unsigned)slot);
    }
}

/* Release partially or fully initialized resources. */
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

    if (detection_queue != NULL) {
        vQueueDelete(detection_queue);
        detection_queue = NULL;
    }

    for (uint8_t i = 0; i < FRAME_BUFFER_COUNT; i++) {
        heap_caps_free(frame_buffers[i]);
        frame_buffers[i] = NULL;
    }
}

/* Allocate frame buffers and initialize the queues. */
static esp_err_t initialize_frame_queues(void)
{
    free_frame_slots = xQueueCreate(FRAME_BUFFER_COUNT, sizeof(uint8_t));
    ready_frame_slots = xQueueCreate(FRAME_BUFFER_COUNT, sizeof(uint8_t));
    detection_queue = xQueueCreate(1, sizeof(detection_message_t));
    if (free_frame_slots == NULL || ready_frame_slots == NULL || detection_queue == NULL) {
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

/* Copy a frame into an available buffer without blocking. */
esp_err_t wifi_stream_submit_frame(uint32_t sequence, const uint8_t *pixels, size_t frame_length)
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
    frame_sequences[slot] = sequence;

    if (xQueueSend(ready_frame_slots, &slot, 0) != pdTRUE) {
        release_frame_slot(slot);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Keep only the latest pending detection result. */
esp_err_t wifi_stream_submit_detection(uint32_t sequence, const ball_detection_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (detection_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    detection_message_t message = {
        .sequence = sequence,
        .detected = result->detected ? 1U : 0U,
        .center_x = result->center_x,
        .center_y = result->center_y,
        .area = result->area
    };

    BaseType_t queue_result = xQueueOverwrite(detection_queue, &message);
    return queue_result == pdPASS ? ESP_OK : ESP_FAIL;
}



/*
 * Send a grayscale frame as indexed UDP packets.
 *
 * Header:
 *   0-3   "GRAY"
 *   4-7   Frame sequence, uint32 big-endian
 *   8-11  Frame size, uint32 big-endian
 *   12-13 Packet index, uint16 big-endian
 *   14-15 Total packets, uint16 big-endian
 */
static int send_frame_udp(int sock, const struct sockaddr_in *client_addr, const uint8_t *frame, uint32_t sequence)
{
    uint8_t packet[FRAME_PACKET_SIZE];
    size_t offset = 0;
    uint16_t packet_index = 0;

    while (offset < FRAME_SIZE) {
        size_t chunk_size = FRAME_SIZE - offset;

        if (chunk_size > FRAME_CHUNK_SIZE) {
            chunk_size = FRAME_CHUNK_SIZE;
        }

        memcpy(packet, "GRAY", 4);

        uint32_t value32 = htonl(sequence);
        memcpy(packet + 4, &value32, sizeof(value32));

        value32 = htonl(FRAME_SIZE);
        memcpy(packet + 8, &value32, sizeof(value32));

        uint16_t value16 = htons(packet_index);
        memcpy(packet + 12, &value16, sizeof(value16));

        value16 = htons((uint16_t)PACKETS_PER_FRAME);
        memcpy(packet + 14, &value16, sizeof(value16));

        memcpy(packet + FRAME_HEADER_SIZE, frame + offset, chunk_size);

        size_t packet_length = FRAME_HEADER_SIZE + chunk_size;
        ssize_t sent = sendto(
            sock,
            packet,
            packet_length,
            0,
            (const struct sockaddr *)client_addr,
            sizeof(*client_addr)
        );

        if (sent < 0 || (size_t)sent != FRAME_HEADER_SIZE + chunk_size) {
            ESP_LOGW(TAG, "Frame UDP send failed: errno=%d", errno);
            return -1;
        }

        offset += chunk_size;
        packet_index++;
    }

    return 0;
}

/* Send a ball-detection result as a single UDP packet. */
static int send_detection_udp(int sock, const struct sockaddr_in *client_addr, const detection_message_t *message)
{
    uint8_t packet[DETECTION_PACKET_SIZE];

    memcpy(packet, "BALL", 4);
    packet[4] = (uint8_t)(message->sequence >> 24);
    packet[5] = (uint8_t)(message->sequence >> 16);
    packet[6] = (uint8_t)(message->sequence >> 8);
    packet[7] = (uint8_t)message->sequence;

    packet[8] = message->detected;

    packet[9]  = (uint8_t)(message->center_x >> 8);
    packet[10] = (uint8_t)message->center_x;

    packet[11] = (uint8_t)(message->center_y >> 8);
    packet[12] = (uint8_t)message->center_y;

    packet[13] = (uint8_t)(message->area >> 24);
    packet[14] = (uint8_t)(message->area >> 16);
    packet[15] = (uint8_t)(message->area >> 8);
    packet[16] = (uint8_t)message->area;

    const ssize_t sent = sendto(
        sock,
        packet,
        sizeof(packet),
        0,
        (const struct sockaddr *)client_addr,
        sizeof(*client_addr)
    );

    if (sent != sizeof(packet)) {
        ESP_LOGW(TAG, "Detection UDP send failed: errno=%d", errno);
        return -1;
    }

    return 0;
}


/*
 * Accept HELLO datagrams during streaming.
 *
 * A PC can register or update its receiving port by sending
 * the four ASCII bytes "HELO" to the camera's station IP on port 9000.
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
    }
}

/* Prefer the newest queued frame and release stale frames. */
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

/* Stream detections and frames to the registered PC. */
static void stream_client_udp(int sock,struct sockaddr_in client_addr)
{
    while (true) {
        update_client_address(sock, &client_addr);

        detection_message_t detection_message;

        if (xQueueReceive(detection_queue, &detection_message, 0) == pdPASS) {
            send_detection_udp(sock, &client_addr, &detection_message);
        }

        uint8_t slot;
        if (!receive_latest_frame_slot(&slot)) {
            continue;
        }

        int result = send_frame_udp(
            sock,
            &client_addr,
            frame_buffers[slot],
            frame_sequences[slot]
        );

        release_frame_slot(slot);

        if (result < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/*
 * UDP server:
 * - Bind port 9000.
 * - Wait for a PC registration datagram.
 * - Stream camera frames and ball detection data to the sender's IP and port.
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

    while (true) {
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
    esp_err_t err = initialize_frame_queues();
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

    ESP_LOGI(TAG, "UDP stream initialized on port %d", UDP_PORT);

    return ESP_OK;
}
