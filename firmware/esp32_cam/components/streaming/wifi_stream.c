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

#define FRAME_STREAM_UDP_PORT   9000

#define BALL_PLATE_AP_IP        "192.168.4.1"
#define BALL_DETECTION_UDP_PORT  9002

#define FRAME_WIDTH             160
#define FRAME_HEIGHT            120
#define FRAME_SIZE              (FRAME_WIDTH * FRAME_HEIGHT)

#define FRAME_HEADER_SIZE       16
#define FRAME_CHUNK_SIZE        1200
#define FRAME_PACKET_SIZE       (FRAME_HEADER_SIZE + FRAME_CHUNK_SIZE)
#define PACKETS_PER_FRAME       ((FRAME_SIZE + FRAME_CHUNK_SIZE - 1) / FRAME_CHUNK_SIZE)

#define BALL_DETECTION_PACKET_SIZE 17

#define FRAME_STREAM_TASK_STACK_SIZE 6144
#define BALL_DETECTION_TASK_STACK_SIZE 3072
#define BALL_DETECTION_TASK_PRIORITY 6
#define FRAME_STREAM_TASK_PRIORITY   4
#define FRAME_BUFFER_COUNT      2
#define FRAME_QUEUE_WAIT_MS     20

static const char *TAG = "WIFI_STREAM";

typedef struct {
    uint32_t frame_sequence;
    uint8_t is_detected;
    uint16_t center_x;
    uint16_t center_y;
    uint32_t ball_area;
} ball_detection_message_t;

static QueueHandle_t available_frame_slots;
static QueueHandle_t queued_frame_slots;
static QueueHandle_t pending_detection_queue;

static uint8_t *grayscale_frame_buffers[FRAME_BUFFER_COUNT];
static uint32_t frame_sequences[FRAME_BUFFER_COUNT];

/* Return a frame buffer to the available-buffer queue. */
static void release_frame_slot(uint8_t slot)
{
    if (xQueueSend(available_frame_slots, &slot, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to return frame buffer slot %u", (unsigned)slot);
    }
}

/* Release partially or fully initialized resources. */
static void destroy_frame_queues(void)
{
    if (available_frame_slots != NULL) {
        vQueueDelete(available_frame_slots);
        available_frame_slots = NULL;
    }

    if (queued_frame_slots != NULL) {
        vQueueDelete(queued_frame_slots);
        queued_frame_slots = NULL;
    }

    if (pending_detection_queue != NULL) {
        vQueueDelete(pending_detection_queue);
        pending_detection_queue = NULL;
    }

    for (uint8_t i = 0; i < FRAME_BUFFER_COUNT; i++) {
        heap_caps_free(grayscale_frame_buffers[i]);
        grayscale_frame_buffers[i] = NULL;
    }
}

/* Allocate frame buffers and initialize the queues. */
static esp_err_t initialize_frame_queues(void)
{
    available_frame_slots = xQueueCreate(FRAME_BUFFER_COUNT, sizeof(uint8_t));
    queued_frame_slots = xQueueCreate(FRAME_BUFFER_COUNT, sizeof(uint8_t));
    pending_detection_queue = xQueueCreate(1, sizeof(ball_detection_message_t));
    if (available_frame_slots == NULL ||
        queued_frame_slots == NULL ||
        pending_detection_queue == NULL) {
        destroy_frame_queues();
        return ESP_ERR_NO_MEM;
    }

    for (uint8_t i = 0; i < FRAME_BUFFER_COUNT; i++) {
        grayscale_frame_buffers[i] =
            heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (grayscale_frame_buffers[i] == NULL) {
            destroy_frame_queues();
            return ESP_ERR_NO_MEM;
        }

        if (xQueueSend(available_frame_slots, &i, 0) != pdTRUE) {
            destroy_frame_queues();
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

/* Copy a frame into an available buffer without blocking. */
esp_err_t wifi_stream_submit_frame(
    uint32_t frame_sequence,
    const uint8_t *grayscale_pixels,
    size_t frame_size_bytes)
{
    if (grayscale_pixels == NULL || frame_size_bytes != FRAME_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (available_frame_slots == NULL || queued_frame_slots == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer_slot_index;
    if (xQueueReceive(available_frame_slots, &buffer_slot_index, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(grayscale_frame_buffers[buffer_slot_index], grayscale_pixels, frame_size_bytes);
    frame_sequences[buffer_slot_index] = frame_sequence;

    if (xQueueSend(queued_frame_slots, &buffer_slot_index, 0) != pdTRUE) {
        release_frame_slot(buffer_slot_index);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Keep only the latest pending detection result. */
esp_err_t wifi_stream_submit_detection(uint32_t frame_sequence, const ball_detection_result_t *detection_result)
{
    if (detection_result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (pending_detection_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ball_detection_message_t detection_message = {
        .frame_sequence = frame_sequence,
        .is_detected = detection_result->detected ? 1U : 0U,
        .center_x = detection_result->center_x,
        .center_y = detection_result->center_y,
        .ball_area = detection_result->area
    };

    BaseType_t queue_result = xQueueOverwrite(pending_detection_queue, &detection_message);
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
static int send_frame_udp(
    int socket_fd,
    const struct sockaddr_in *client_address,
    const uint8_t *grayscale_pixels,
    uint32_t frame_sequence)
{
    uint8_t udp_packet[FRAME_PACKET_SIZE];
    size_t frame_pixel_offset = 0;
    uint16_t packet_index = 0;

    while (frame_pixel_offset < FRAME_SIZE) {
        size_t payload_size = FRAME_SIZE - frame_pixel_offset;

        if (payload_size > FRAME_CHUNK_SIZE) {
            payload_size = FRAME_CHUNK_SIZE;
        }

        memcpy(udp_packet, "GRAY", 4);

        uint32_t network_order_value32 = htonl(frame_sequence);
        memcpy(udp_packet + 4, &network_order_value32, sizeof(network_order_value32));

        network_order_value32 = htonl(FRAME_SIZE);
        memcpy(udp_packet + 8, &network_order_value32, sizeof(network_order_value32));

        uint16_t network_order_value16 = htons(packet_index);
        memcpy(udp_packet + 12, &network_order_value16, sizeof(network_order_value16));

        network_order_value16 = htons((uint16_t)PACKETS_PER_FRAME);
        memcpy(udp_packet + 14, &network_order_value16, sizeof(network_order_value16));

        memcpy(
            udp_packet + FRAME_HEADER_SIZE,
            grayscale_pixels + frame_pixel_offset,
            payload_size
        );

        size_t udp_packet_size = FRAME_HEADER_SIZE + payload_size;
        ssize_t sent_byte_count = sendto(
            socket_fd,
            udp_packet,
            udp_packet_size,
            0,
            (const struct sockaddr *)client_address,
            sizeof(*client_address)
        );

        if (sent_byte_count < 0 || (size_t)sent_byte_count != udp_packet_size) {
            ESP_LOGW(TAG, "Frame UDP send failed: errno=%d", errno);
            return -1;
        }

        frame_pixel_offset += payload_size;
        packet_index++;
    }

    return 0;
}

/* Send a ball-detection result as a single UDP packet. */
static int send_detection_udp(
    int socket_fd,
    const struct sockaddr_in *destination_address,
    const ball_detection_message_t *detection_message)
{
    uint8_t detection_packet[BALL_DETECTION_PACKET_SIZE];

    memcpy(detection_packet, "BALL", 4);
    detection_packet[4] = (uint8_t)(detection_message->frame_sequence >> 24);
    detection_packet[5] = (uint8_t)(detection_message->frame_sequence >> 16);
    detection_packet[6] = (uint8_t)(detection_message->frame_sequence >> 8);
    detection_packet[7] = (uint8_t)detection_message->frame_sequence;

    detection_packet[8] = detection_message->is_detected;

    detection_packet[9]  = (uint8_t)(detection_message->center_x >> 8);
    detection_packet[10] = (uint8_t)detection_message->center_x;

    detection_packet[11] = (uint8_t)(detection_message->center_y >> 8);
    detection_packet[12] = (uint8_t)detection_message->center_y;

    detection_packet[13] = (uint8_t)(detection_message->ball_area >> 24);
    detection_packet[14] = (uint8_t)(detection_message->ball_area >> 16);
    detection_packet[15] = (uint8_t)(detection_message->ball_area >> 8);
    detection_packet[16] = (uint8_t)detection_message->ball_area;

    const ssize_t sent_byte_count = sendto(
        socket_fd,
        detection_packet,
        sizeof(detection_packet),
        0,
        (const struct sockaddr *)destination_address,
        sizeof(*destination_address)
    );

    if (sent_byte_count != sizeof(detection_packet)) {
        ESP_LOGW(TAG, "Detection UDP send failed: errno=%d", errno);
        return -1;
    }

    return 0;
}

static void detection_udp_task(void *task_context)
{
    (void)task_context;

    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0) {
        ESP_LOGE(TAG, "Detection socket creation failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in detection_server_addr = {0};
    detection_server_addr.sin_family = AF_INET;
    detection_server_addr.sin_port = htons(BALL_DETECTION_UDP_PORT);

    if (inet_pton(AF_INET, BALL_PLATE_AP_IP, &detection_server_addr.sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid ball plate access point IP address");
        close(socket_fd);
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        ball_detection_message_t detection_message;
        if (xQueueReceive(pending_detection_queue, &detection_message, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        if (send_detection_udp(socket_fd, &detection_server_addr, &detection_message)) {
            ESP_LOGW(TAG, "Failed to send detection packet");
        }
    }
}

/*
 * Accept HELLO datagrams during streaming.
 *
 * A PC can register or update its receiving port by sending
 * the four ASCII bytes "HELO" to the camera's station IP on port 9000.
 */
static void update_client_addr(int socket_fd, struct sockaddr_in *client_addr)
{
    uint8_t datagram[32];
    struct sockaddr_in sender_addr = {0};
    socklen_t sender_addr_len = sizeof(sender_addr);

    ssize_t received = recvfrom(
        socket_fd,
        datagram,
        sizeof(datagram),
        MSG_DONTWAIT,
        (struct sockaddr *)&sender_addr,
        &sender_addr_len
    );

    if (received == 4 && memcmp(datagram, "HELO", 4) == 0) {
        *client_addr = sender_addr;
    }
}

/* Prefer the newest queued frame and release stale frames. */
static bool receive_latest_frame_slot(uint8_t *slot)
{
    if (xQueueReceive(queued_frame_slots, slot, pdMS_TO_TICKS(FRAME_QUEUE_WAIT_MS)) != pdTRUE) {
        return false;
    }

    uint8_t newer_slot;
    while (xQueueReceive(queued_frame_slots, &newer_slot, 0) == pdTRUE) {
        release_frame_slot(*slot);
        *slot = newer_slot;
    }

    return true;
}

/* Stream detections and frames to the registered PC. */
static void stream_client_udp(int socket_fd, struct sockaddr_in client_addr)
{
    while (true) {
        update_client_addr(socket_fd, &client_addr);

        // ball_detection_message_t detection_message;

        // if (xQueueReceive(pending_detection_queue, &detection_message, 0) == pdPASS) {
        //     send_detection_udp(socket_fd, &client_addr, &detection_message);
        // }

        uint8_t slot;
        if (!receive_latest_frame_slot(&slot)) {
            continue;
        }

        int send_result = send_frame_udp(
            socket_fd,
            &client_addr,
            grayscale_frame_buffers[slot],
            frame_sequences[slot]
        );

        release_frame_slot(slot);

        if (send_result < 0) {
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
static void frame_stream_server_task(void *task_context)
{
    (void)task_context;

    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_fd < 0) {
        ESP_LOGE(TAG, "UDP socket creation failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_address = {0};

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(FRAME_STREAM_UDP_PORT);
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socket_fd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno=%d", errno);
        close(socket_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", FRAME_STREAM_UDP_PORT);

    while (true) {
        uint8_t registration_datagram[32];
        struct sockaddr_in client_address = {0};
        socklen_t client_address_length = sizeof(client_address);

        ssize_t received_byte_count = recvfrom(
            socket_fd,
            registration_datagram,
            sizeof(registration_datagram),
            0,
            (struct sockaddr *)&client_address,
            &client_address_length
        );

        if (received_byte_count < 0) {
            ESP_LOGW(TAG, "UDP receive failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (received_byte_count != 4 ||
            memcmp(registration_datagram, "HELO", 4) != 0) {
            continue;
        }

        ESP_LOGI(
            TAG,
            "Client registered: %s:%u",
            inet_ntoa(client_address.sin_addr),
            (unsigned)ntohs(client_address.sin_port)
        );

        stream_client_udp(socket_fd, client_address);
    }
}

esp_err_t wifi_stream_init(void)
{
    esp_err_t err = initialize_frame_queues();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize frame queues and buffers: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t detection_task_result = xTaskCreate(
        detection_udp_task,
        "detection_udp",
        BALL_DETECTION_TASK_STACK_SIZE,
        NULL,
        BALL_DETECTION_TASK_PRIORITY,
        NULL
    );
    if (detection_task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create detection UDP task");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t frame_server_task_result = xTaskCreate(
        frame_stream_server_task,
        "frame_stream_server",
        FRAME_STREAM_TASK_STACK_SIZE,
        NULL,
        FRAME_STREAM_TASK_PRIORITY,
        NULL
    );

    if (frame_server_task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UDP server task");
        destroy_frame_queues();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "UDP stream initialized on port %d", FRAME_STREAM_UDP_PORT);

    return ESP_OK;
}
