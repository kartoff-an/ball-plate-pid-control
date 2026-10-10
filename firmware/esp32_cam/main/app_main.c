#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * AI Thinker ESP32-CAM with OV2640
 * Camera pins are fixed by the board layout
*/

#define CAM_PIN_PWDN         32
#define CAM_PIN_RESET        -1
#define CAM_PIN_XCLK          0
#define CAM_PIN_SIOD         26
#define CAM_PIN_SIOC         27

#define CAM_PIN_D7           35
#define CAM_PIN_D6           34
#define CAM_PIN_D5           39
#define CAM_PIN_D4           36
#define CAM_PIN_D3           21
#define CAM_PIN_D2           19
#define CAM_PIN_D1           18
#define CAM_PIN_D0            5

#define CAM_PIN_VSYNC        25
#define CAM_PIN_HREF         23
#define CAM_PIN_PCLK         22

#define IMAGE_WIDTH         160
#define IMAGE_HEIGHT        120
#define RGB_CHANNELS          3
#define FRAME_INTERVAL_MS     0

static const char *TAG = "BALL_PLATE_CAMERA";

typedef struct {
    uint8_t min;
    uint8_t max;
    uint8_t mean;
} image_stats_t;

static esp_err_t camera_init(void)
{
    const camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        // for camera efficiency we capture jpeg first
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = FRAMESIZE_QQVGA,
        .jpeg_quality = 0,

        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    return esp_camera_init(&config);
}

static inline image_stats_t calculate_image_stats(const uint8_t *gray_buf, size_t pixel_count)
{
    image_stats_t stats = {
        .min = 255,
        .max = 0,
        .mean = 0,
    };

    if (gray_buf == NULL || pixel_count == 0) {
        return stats;
    }

    uint64_t sum = 0;
    uint8_t min_val = 255;
    uint8_t max_val = 0;

    for (size_t i = 0; i < pixel_count; ++i) {
        const uint8_t gray = gray_buf[i];

        if (gray < min_val) {
            min_val = gray;
        }

        if (gray > max_val) {
            max_val = gray;
        }

        sum += gray;
    }

    stats.min = min_val;
    stats.max = max_val;
    stats.mean = (uint8_t)(sum / pixel_count);
    return stats;
}

static void camera_task(void *pvParameters)
{
    uint32_t frame_number = 0;
    int64_t last_report_us = esp_timer_get_time();
    uint32_t frames_since_report = 0;

    while (true) {
        const int64_t capture_start_us = esp_timer_get_time();

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "Frame capture failed");
            if (FRAME_INTERVAL_MS > 0) {
                vTaskDelay(pdMS_TO_TICKS(FRAME_INTERVAL_MS));
            }
            continue;
        }

        if (fb->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Unexpected pixel format: %d", (int)fb->format);
            esp_camera_fb_return(fb);
            continue;
        }

        const uint16_t width = fb->width;
        const uint16_t height = fb->height;
        const size_t actual_pixel_count = (size_t)width * height;

        if (width != IMAGE_WIDTH || height != IMAGE_HEIGHT) {
            ESP_LOGW(TAG, "Unexpected dimension: %ux%u", (unsigned)width, (unsigned)height);
            continue;
        }

        const image_stats_t stats = calculate_image_stats(fb->buf, actual_pixel_count);

        esp_camera_fb_return(fb);

        const int64_t capture_end_us = esp_timer_get_time();
        const uint32_t processing_time_ms = (uint32_t)((capture_end_us - capture_start_us) / 1000);

        frame_number++;
        frames_since_report++;

        // report once per second
        const int64_t now_us = esp_timer_get_time();
        const int64_t elapsed_us = now_us - last_report_us;

        if (elapsed_us >= 1000000) {
            const uint32_t fps_x10 = (uint32_t)(
                ((uint64_t)frames_since_report * 10000000ULL) /
                (uint64_t)elapsed_us
            );
            ESP_LOGI(
                TAG,
                "frames=%" PRIu32
                " size=%ux%u"
                " gray[min=%u max=%u mean=%u]"
                " processing=%" PRIu32 " ms"
                " fps=%" PRIu32 ".%" PRIu32,
                frame_number,
                (unsigned)width,
                (unsigned)height,
                (unsigned)stats.min,
                (unsigned)stats.max,
                (unsigned)stats.mean,
                processing_time_ms,
                fps_x10 / 10,
                fps_x10 % 10
            );

            frames_since_report = 0;
            last_report_us = now_us;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing AI Thinker ESP32-CAM");

    esp_err_t error = camera_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(error));
        return;
    }

    ESP_LOGI(TAG, "Camera initialized: QQVGA Grayscale");

    BaseType_t result = xTaskCreatePinnedToCore(camera_task, "camera_task", 4096, NULL, 5, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create camera task");
        return;
    }

    ESP_LOGI(TAG, "Camera capture loop successfully pinned to Core 1");
}