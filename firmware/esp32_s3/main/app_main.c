#include "wifi_ap.h"
#include "ball_receiver.h"
#include "pca9685.h"
#include "pid_controller.h"
#include "telemetry.h"
#include "calibration_commands.h"
#include "config_commands.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DETECTION_DEBUG     1

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

    ESP_ERROR_CHECK(telemetry_init());

    ESP_ERROR_CHECK(pca9685_init(12, 13));
    ESP_ERROR_CHECK(pca9685_set_pulse_us(0, 2200));
    ESP_ERROR_CHECK(pca9685_set_pulse_us(1, 2200));

    ESP_ERROR_CHECK(ball_receiver_init());

    #if DETECTION_DEBUG
    xTaskCreate(ball_monitor_task, "ball_monitor", 3072, NULL, 3, NULL);
    #endif

    ESP_ERROR_CHECK(pid_controller_init());

    ESP_ERROR_CHECK(config_commands_init());

    ESP_ERROR_CHECK(calibration_commands_init());
}