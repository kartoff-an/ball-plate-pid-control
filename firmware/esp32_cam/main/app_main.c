#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>

#include "camera_capture.h"
#include "wifi_stream.h"
#include "wifi_station.h"
#include "ball_detection.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FRAME_WIDTH                   160
#define FRAME_HEIGHT                  120
#define FRAME_SIZE                    (FRAME_WIDTH * FRAME_HEIGHT)
#define CAMERA_FRAME_INTERVAL_MS      0

#define BALL_DETECTION_THRESHOLD      100
#define BALL_DETECTION_MIN_AREA          8
#define BALL_DETECTION_MAX_AREA       1000

static const char *TAG = "BALL_PLATE_CAMERA";
static uint8_t binary_frame[FRAME_SIZE];

static void camera_frame_task(void *task_context)
{
    (void)task_context;

    uint32_t frame_seq = 0;
    int64_t last_report_us = esp_timer_get_time();
    uint32_t frames_since_report = 0;
    uint32_t dropped_frames_since_report = 0;

    while (true) {
        const int64_t frame_start_us = esp_timer_get_time();

        camera_fb_t *frame = camera_capture_get_frame();

        if (frame == NULL) {
            ESP_LOGW(TAG, "Frame capture failed");
            // if (CAMERA_FRAME_INTERVAL_MS > 0) {
            //     vTaskDelay(pdMS_TO_TICKS(CAMERA_FRAME_INTERVAL_MS));
            // }
            continue;
        }

        if (frame->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Unexpected pixel format: %d", (int)frame->format);
            camera_capture_release_frame(frame);
            continue;
        }

        if (frame->buf == NULL ||
            frame->width != FRAME_WIDTH ||
            frame->height != FRAME_HEIGHT ||
            frame->len != (size_t)FRAME_WIDTH * FRAME_HEIGHT) {
            ESP_LOGW(TAG, "Unexpected frame buffer or dimensions");
            camera_capture_release_frame(frame);
            continue;
        }

        const uint16_t width = frame->width;
        const uint16_t height = frame->height;
        const size_t pixel_count = (size_t)width * height;

        const camera_image_stats_t stats = camera_capture_get_image_stats(
            frame->buf,
            pixel_count
        );

        // starting here, we perform the ball detection before we release the camera capture
        ball_detection_result_t detection = {0};
        const esp_err_t detection_err = ball_detection_detect(
            frame->buf,
            width,
            height,
            BALL_DETECTION_THRESHOLD,
            BALL_DETECTION_MIN_AREA,
            BALL_DETECTION_MAX_AREA,
            binary_frame,
            &detection
        );

        if (detection_err != ESP_OK) {
            ESP_LOGE(TAG, "Ball detection failed: %s", esp_err_to_name(detection_err));
            camera_capture_release_frame(frame);
            continue;
        }

        // then transmit these frames over UDP for visual display in the GUI
        const esp_err_t frame_stream_result = wifi_stream_submit_frame(
            frame_seq,
            frame->buf,
            frame->len
        );
        if (frame_stream_result == ESP_ERR_TIMEOUT) {
            dropped_frames_since_report++;
        } else if (frame_stream_result != ESP_OK) {
            ESP_LOGW(TAG, "Could not queue frame for streaming: %s",
                     esp_err_to_name(frame_stream_result));
        }

        camera_capture_release_frame(frame);

        const int64_t frame_end_us = esp_timer_get_time();
        const uint32_t processing_ms = (uint32_t)((frame_end_us - frame_start_us) / 1000);

        frame_seq++;
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
                frame_seq,
                (unsigned)width,
                (unsigned)height,
                (unsigned)stats.min,
                (unsigned)stats.max,
                (unsigned)stats.mean,
                processing_ms,
                fps_x10 / 10,
                fps_x10 % 10,
                dropped_frames_since_report
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
            dropped_frames_since_report = 0;
            last_report_us = now_us;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-CAM");

    ESP_ERROR_CHECK(wifi_station_connect());

    ESP_LOGI(TAG, "Wi-Fi connected");

    ESP_ERROR_CHECK(wifi_stream_init());

    ESP_ERROR_CHECK(camera_capture_init());

    BaseType_t task_result = xTaskCreatePinnedToCore(
        camera_frame_task,
        "camera_frame",
        4096,
        NULL,
        5,
        NULL,
        1
    );
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create camera task");
        return;
    }

    ESP_LOGI(TAG, "Camera frame capture and UDP streaming started");
}