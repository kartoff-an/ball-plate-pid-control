#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>

#include "camera_capture.h"
#include "communication.h"
#include "wifi_stream.h"
#include "wifi_station.h"
#include "ball_detection.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define IMAGE_WIDTH         160
#define IMAGE_HEIGHT        120
#define FRAME_SIZE          (IMAGE_WIDTH * IMAGE_HEIGHT)
#define FRAME_INTERVAL_MS     0

#define BALL_THRESHOLD      100
#define BALL_MIN_AREA         8
#define BALL_MAX_AREA       1000

static const char *TAG = "BALL_PLATE_CAMERA";
static uint8_t binary_buf[FRAME_SIZE];

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

        // starting here, we perform the ball detection before we release the camera capture
        ball_detection_result_t detection = {0};
        const esp_err_t detection_result = detect_ball(
            frame->buf,
            width,
            height,
            BALL_THRESHOLD,
            BALL_MIN_AREA,
            BALL_MAX_AREA,
            binary_buf,
            &detection
        );

        if (detection_result != ESP_OK) {
            ESP_LOGE(TAG, "Ball detection failed: %s", esp_err_to_name(detection_result));
            release_capture_frame(frame);
            continue;
        }

        // transmit detection results over UDP
        const esp_err_t detection_stream_result = wifi_stream_submit_detection(frame_number, &detection);
        if (detection_stream_result != ESP_OK) {
            ESP_LOGW(TAG, "Could not queue detection: %s", esp_err_to_name(detection_stream_result));
        }

        // then transmit these frames over UDP for visual display in the GUI
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
            ESP_LOGI(
                TAG,
                "white_pixels=%" PRIu32
                " ball=%d"
                " center=(%u,%u)"
                " area=%" PRIu32,
                detection.white_pixels,
                (int)detection.detected,
                (unsigned)detection.center_x,
                (unsigned)detection.center_y,
                detection.area
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
    ESP_LOGI(TAG, "Starting ESP32-CAM");

    ESP_ERROR_CHECK(wifi_station_connect());

    ESP_LOGI(TAG, "Wi-Fi connected");

    ESP_ERROR_CHECK(wifi_stream_init());

    ESP_ERROR_CHECK(camera_capture_init());

    BaseType_t result = xTaskCreatePinnedToCore(camera_task, "camera_task", 4096, NULL, 5, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create camera task");
        return;
    }

    ESP_LOGI(TAG, "Camera frame capture and UDP streaming started");

    // COMMUNICATION

    // ESP_ERROR_CHECK(communication_init(NULL));
    // BaseType_t comm_result = xTaskCreate(communication_test_task, "communication_test", 3072, NULL, 4, NULL);
    // if (comm_result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create commmunication task");
    // }
}