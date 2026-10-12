#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>

#include "camera_capture.h"
#include "communication.h"
#include "wifi_stream.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define IMAGE_WIDTH         160
#define IMAGE_HEIGHT        120
#define FRAME_INTERVAL_MS     0;

static const char *TAG = "BALL_PLATE_CAMERA";

static void camera_task(void *pvParameters)
{
    (void)pvParameters;

    uint32_t frame_number = 0;
    int64_t last_report_us = esp_timer_get_time();
    uint32_t frames_since_report = 0;
    uint32_t stream_dropped_since_report = 0;

    while (true) {
        const int64_t capture_start_us = esp_timer_get_time();

        camera_fb_t *frame = get_capture_frame();

        if (frame == NULL) {
            ESP_LOGW(TAG, "Frame capture failed");
            // if (FRAME_INTERVAL_MS > 0) {
            //     vTaskDelay(pdMS_TO_TICKS(FRAME_INTERVAL_MS));
            // }
            continue;
        }

        if (frame->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Unexpected pixel format: %d", (int)frame->format);
            release_capture_frame(frame);
            continue;
        }

        if (frame->buf == NULL ||
            frame->width != IMAGE_WIDTH ||
            frame->height != IMAGE_HEIGHT ||
            frame->len != (size_t)IMAGE_WIDTH * IMAGE_HEIGHT) {
            ESP_LOGW(TAG, "Unexpected frame buffer or dimensions");
            release_capture_frame(frame);
            continue;
        }

        const uint16_t width = frame->width;
        const uint16_t height = frame->height;
        const size_t actual_pixel_count = (size_t)width * height;

        const image_stats_t stats = calculate_image_stats(frame->buf, actual_pixel_count);

        const esp_err_t stream_result = wifi_stream_submit_frame(frame->buf, frame->len);
        if (stream_result == ESP_ERR_TIMEOUT) {
            stream_dropped_since_report++;
        } else if (stream_result != ESP_OK) {
            ESP_LOGW(TAG, "Could not queue frame for streaming: %s",
                     esp_err_to_name(stream_result));
        }

        release_capture_frame(frame);

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
                " fps=%" PRIu32 ".%" PRIu32
                " stream_dropped=%" PRIu32,
                frame_number,
                (unsigned)width,
                (unsigned)height,
                (unsigned)stats.min,
                (unsigned)stats.max,
                (unsigned)stats.mean,
                processing_time_ms,
                fps_x10 / 10,
                fps_x10 % 10,
                stream_dropped_since_report
            );

            frames_since_report = 0;
            stream_dropped_since_report = 0;
            last_report_us = now_us;
        }
    }
}

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static void communication_test_task(void *pvParameters)
{
    (void)pvParameters;

    static const uint8_t message[] = "HELLO";
    static const char *TAG = "COMM_TEST";

    while (true) {
        esp_err_t error = communication_send(BROADCAST_MAC, message, sizeof(message) - 1);
        if (error == ESP_OK) {
            ESP_LOGI(TAG, "HELLO packet submitted");
        }
        else {
            ESP_LOGW(TAG, "Send failed: %s", esp_err_to_name(error));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ball-and-plate camera application");

    // CAMERA CAPTURE

    ESP_ERROR_CHECK(camera_capture_init());

    ESP_ERROR_CHECK(wifi_stream_init());

    BaseType_t result = xTaskCreatePinnedToCore(camera_task, "camera_task", 4096, NULL, 5, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create camera task");
        return;
    }

    ESP_LOGI(TAG, "Camera task started on Core 1");

    // COMMUNICATION

    // ESP_ERROR_CHECK(communication_init(NULL));
    // BaseType_t comm_result = xTaskCreate(communication_test_task, "communication_test", 3072, NULL, 4, NULL);
    // if (comm_result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create commmunication task");
    // }
}