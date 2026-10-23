#include <string.h>

#include "ball_now.h"
#include "ball_measurement.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"

static const char *TAG = "BALL_NOW";

static uint8_t s_peer_mac[6];
static bool s_initialized = false;

esp_err_t ball_now_init(const uint8_t peer_mac[6])
{
    if (peer_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized) {
        return ESP_OK;
    }

    memcpy(s_peer_mac, peer_mac, sizeof(s_peer_mac));

    esp_err_t error = esp_now_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(error));
        return error;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_peer_mac, sizeof(s_peer_mac));
    peer.ifidx = WIFI_IF_STA;
    peer.channel = 0;
    peer.encrypt = false;

    error = esp_now_add_peer(&peer);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Adding S3 peer failed: %s", esp_err_to_name(error));
        esp_now_deinit();
        return error;
    }

    s_initialized = true;

    ESP_LOGI(TAG, "ESP-NOW initialized for S3 peer");
    return ESP_OK;
}

esp_err_t ball_now_send_detection(uint32_t sequence, const ball_detection_result_t *detection) {
    if (detection == NULL || !s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ball_measurement_t message = {
        .magic = BALL_MEASUREMENT_MAGIC,
        .version = BALL_MEASUREMENT_VERSION,
        .detected = detection->detected ? 1 : 0,
        .sequence = sequence,
        .center_x = detection->detected ? detection->center_x + BALL_DETECTION_CROP_X_OFFSET : 0,
        .center_y = detection->detected ? detection->center_y + BALL_DETECTION_CROP_Y_OFFSET : 0,
        .area = detection->detected ? detection->area : 0
    };

    return esp_now_send(s_peer_mac, (const uint8_t *)&message, sizeof(message));
}