#include "wifi_ap.h"
#include "ball_receiver.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"

#define DETECTION_DEBUG     0

static const char *TAG = "BALL_PLATE";

static void ball_monitor_task(void *arg)
{
    ball_measurement_t measurement;
    int64_t age_ms;

    while (true) {
        if (ball_receiver_get_latest(&measurement, &age_ms)) {
            if (age_ms < 200) {
                if (measurement.detected) {
                    ESP_LOGI(TAG,
                             "Ball: x=%u y=%u area=%" PRIu32
                             " seq=%" PRIu32 " age=%" PRId64 "ms",
                             measurement.center_x,
                             measurement.center_y,
                             measurement.area,
                             measurement.sequence,
                             age_ms);
                }
                else {
                    ESP_LOGI(TAG, "Ball not detected; age=%d ms", age_ms);
                }
            }
            else {
                ESP_LOGW(TAG, "Measurement stale: age=%d ms", age_ms);
            }
        }
        else {
            ESP_LOGI(TAG, "Waiting for first measurement");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(wifi_ap_init());

    ESP_LOGI(TAG, "ESP32-S3 AP is ready");

    ESP_ERROR_CHECK(ball_receiver_init());

    #if CAMERA_DEBUG
    xTaskCreate(ball_monitor_task, "ball_monitor", 3072, NULL, 3, NULL);
    #endif
}