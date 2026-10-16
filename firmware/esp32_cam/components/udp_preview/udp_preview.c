#include "udp_preview.h"

#include <string.h>
#include <unistd.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#define REGISTRATION_PORT       9000
#define REGISTRATION_MESSAGE    "HELO"
#define REGISTRATION_STACK_SIZE 3072
#define REGISTRATION_PRIORITY   4

static const char *TAG = "UDP_PREVIEW";

static void registration_listener_task(void *task_context)
{
    (void)task_context;

    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_fd < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in local_socket_address = {
        .sin_family = AF_INET,
        .sin_port = htons(REGISTRATION_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(socket_fd, (struct sockaddr *)&local_socket_address, sizeof(local_socket_address)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d", REGISTRATION_PORT);
        close(socket_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening for PC registration on UDP port %d", REGISTRATION_PORT);

    while (true) {
        char registration_datagram[64];

        struct sockaddr_in source_address = {0};
        socklen_t source_address_length = sizeof(source_address);

        int received_byte_count = recvfrom(
            socket_fd,
            registration_datagram,
            sizeof(registration_datagram) - 1,
            0,
            (struct sockaddr *)&source_address,
            &source_address_length
        );

        if (received_byte_count < 0) {
            ESP_LOGW(TAG, "UDP receive failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        registration_datagram[received_byte_count] = '\0';

        if (strcmp(registration_datagram, REGISTRATION_MESSAGE) != 0) {
            continue;
        }

        ESP_LOGI(TAG, "UDP connection test received from %s:%u",
            inet_ntoa(source_address.sin_addr),
            (unsigned)ntohs(source_address.sin_port)
        );
    }
}

esp_err_t udp_preview_init(void)
{
    BaseType_t result = xTaskCreate(
        registration_listener_task,
        "registration_listener",
        REGISTRATION_STACK_SIZE,
        NULL,
        REGISTRATION_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create registration listener task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
