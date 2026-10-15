#include "udp_preview.h"

#include <string.h>
#include <unistd.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#define PREVIEW_PORT         9000
#define PREVIEW_MESSAGE      "HELO"
#define DISCOVERY_STACK_SIZE 3072
#define DISCOVERY_PRIORITY   4

static const char *TAG = "UDP_PREVIEW";

static void discovery_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in local_address = {
        .sin_family = AF_INET,
        .sin_port = htons(PREVIEW_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&local_address, sizeof(local_address)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d", PREVIEW_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening for PC registration on UDP port %d", PREVIEW_PORT);

    while (true) {
        char message[64];

        struct sockaddr_in source_address;
        socklen_t source_length = sizeof(source_address);

        int received = recvfrom(
            sock,
            message,
            sizeof(message) - 1,
            0,
            (struct sockaddr *)&source_address,
            &source_length
        );

        if (received < 0) {
            ESP_LOGW(TAG, "UDP receive failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        message[received] = '\0';

        if (strcmp(message, PREVIEW_MESSAGE) != 0) {
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
        discovery_task,
        "udp_discovery",
        DISCOVERY_STACK_SIZE,
        NULL,
        DISCOVERY_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create discovery task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
