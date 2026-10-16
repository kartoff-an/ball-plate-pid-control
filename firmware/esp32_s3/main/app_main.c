#include "wifi_ap.h"
#include "ball_udp_receiver.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_ERROR_CHECK(wifi_ap_init());

    ESP_LOGI(TAG, "ESP32-S3 AP is ready");

    ESP_ERROR_CHECK(ball_udp_receiver_init());
}