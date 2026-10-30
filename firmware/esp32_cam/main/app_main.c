// This is flashed into, tested and validated with the ESP32-CAM along with
// the hardware and physical setup of the ball-and-plate balancing platform.

// Chip type:          ESP32-D0WD-V3 (revision v3.1)
// Features:           Wi-Fi, BT, Dual Core + LP Core, 240MHz, Vref calibration in eFuse, Coding Scheme None
// Crystal frequency:  40MHz

#include "camera.h"
#include "wifi_stream.h"
#include "wifi_station.h"
#include "ball_now.h"
#include "vision_tuner.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "BALL_PLATE_CAMERA";

static const uint8_t S3_AP_MAC[6] = {0xAC, 0xA7, 0x04, 0x2B, 0x9D, 0x35};

static void init_nvs(void)
{
    esp_err_t error = nvs_flash_init();

    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }

    ESP_ERROR_CHECK(error);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-CAM");

    init_nvs();

    ESP_ERROR_CHECK(wifi_station_connect());

    ESP_LOGI(TAG, "Wi-Fi connected");

    ESP_ERROR_CHECK(ball_now_init(S3_AP_MAC));

    ESP_ERROR_CHECK(wifi_stream_init());

    ESP_ERROR_CHECK(camera_init());

    ESP_ERROR_CHECK(vision_tuner_start());

    ESP_ERROR_CHECK(camera_start());

    ESP_LOGI(TAG, "Camera processing and streaming started");
}