#include <string.h>

#include "ball_receiver.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define FRAME_WIDTH             160
#define FRAME_HEIGHT            120

static const char *TAG = "BALL_RECEIVER";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static ball_measurement_t s_latest;
static int64_t s_last_received_us = 0;
static bool s_has_measurement = false;

static void ball_receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int data_length) {
    if (info == NULL || info->src_addr == NULL || data == NULL || data_length != sizeof(ball_measurement_t)) {
        return;
    }

    ball_measurement_t message;
    memcpy(&message, data, sizeof(message));

    if (message.magic != BALL_MEASUREMENT_MAGIC || message.version != BALL_MEASUREMENT_VERSION || message.detected > 1) {
        return;
    }

    if (message.detected && (message.center_x >= FRAME_WIDTH || message.center_y >= FRAME_HEIGHT|| message.area == 0)) {
        return;
    }

    int64_t received_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);
    s_latest = message;
    s_last_received_us = received_us;
    s_has_measurement = true;
    portEXIT_CRITICAL(&s_lock);
}

esp_err_t ball_receiver_init(void)
{
    esp_err_t error = esp_now_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_now_register_recv_cb(ball_receive_callback);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Register receive callback failed: %s", esp_err_to_name(error));
        esp_now_deinit();
        return error;
    }

    ESP_LOGI(TAG, "ESP-NOW receiver initialized");
    return ESP_OK;
}

bool ball_receiver_get_latest(ball_measurement_t *measurement, int64_t *age_ms)
{
    if (measurement == NULL || age_ms == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_lock);

    bool available = s_has_measurement;
    if (available) {
        *measurement = s_latest;
        *age_ms = (esp_timer_get_time() - s_last_received_us) / 1000;
    }

    portEXIT_CRITICAL(&s_lock);

    return available;
}