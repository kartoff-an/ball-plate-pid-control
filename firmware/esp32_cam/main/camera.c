#include "camera.h"

#include <stdint.h>
#include <inttypes.h>

#include "camera_config.h"
#include "wifi_stream.h"
#include "ball_detection.h"
#include "ball_now.h"

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * AI Thinker ESP32-CAM pin configuration.
 */

#define CAM_PIN_PWDN   32
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27

#define CAM_PIN_D7     35
#define CAM_PIN_D6     34
#define CAM_PIN_D5     39
#define CAM_PIN_D4     36
#define CAM_PIN_D3     21
#define CAM_PIN_D2     19
#define CAM_PIN_D1     18
#define CAM_PIN_D0      5

#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22

#define CAMERA_XCLK_FREQ_HZ         20000000

#define CAMERA_TASK_STACK_SIZE      4096
#define CAMERA_TASK_PRIORITY        5
#define CAMERA_TASK_CORE            1

#define CAMERA_DEBUG                0

static const char *TAG = "CAMERA";

static uint8_t binary_frame[CAMERA_FRAME_SIZE];

esp_err_t camera_init(void)
{
    const camera_config_t camera_config = {
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

        .xclk_freq_hz = CAMERA_XCLK_FREQ_HZ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = FRAMESIZE_QQVGA,
        .jpeg_quality = 0,

        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    const esp_err_t init_result = esp_camera_init(&camera_config);

    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(init_result));
        return init_result;
    }

    ESP_LOGI(TAG, "Camera initialized");

    return ESP_OK;
}

static void camera_task(void *task_context)
{
    (void)task_context;

    uint32_t frame_seq = 0;

    int64_t last_fps_report_us = esp_timer_get_time();
    uint32_t processed_frames = 0;
    uint32_t total_processing_us = 0;
    uint32_t max_processing_us = 0;

#if CAMERA_DEBUG
    int64_t last_debug_report_us = esp_timer_get_time();
    uint32_t frames_since_report = 0;
    uint32_t dropped_frames_since_report = 0;
#endif

    while (true) {
#if CAMERA_DEBUG
        const int64_t frame_start_us = esp_timer_get_time();
#endif

        camera_fb_t *frame = esp_camera_fb_get();

        if (frame == NULL) {
            ESP_LOGW(TAG, "Frame capture failed");
            continue;
        }

        if (frame->len != CAMERA_FRAME_SIZE ||
            frame->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Unexpected frame format or size: len=%u, format=%d", (unsigned)frame->len, (int)frame->format);

            esp_camera_fb_return(frame);
            continue;
        }

        /* Ball detection */
        const int64_t processing_start_us = esp_timer_get_time();

        ball_detection_result_t detection = {0};

        const esp_err_t detection_result = ball_detection_detect(frame->buf, binary_frame, &detection);
        if (detection_result != ESP_OK) {
            ESP_LOGE(TAG, "Ball detection failed: %s", esp_err_to_name(detection_result));
            esp_camera_fb_return(frame);
            continue;
        }

        const uint32_t processing_us = (uint32_t)(esp_timer_get_time() - processing_start_us);

        /* Detection performance statistics */
        processed_frames++;
        total_processing_us += processing_us;

        if (processing_us > max_processing_us) {
            max_processing_us = processing_us;
        }

        const int64_t now_us = esp_timer_get_time();

        if (now_us - last_fps_report_us >= 1000000) {
            const float fps = processed_frames * 1000000.0f / (float)(now_us - last_fps_report_us);
            const uint32_t avg_processing_ms =
                    processed_frames > 0
                    ? total_processing_us / processed_frames / 1000
                    : 0;

            ESP_LOGI(
                TAG,
                "Detection FPS=%.2f, avg detection=%" PRIu32
                " ms, max detection=%" PRIu32 " ms",
                fps,
                avg_processing_ms,
                max_processing_us / 1000
            );

            last_fps_report_us = now_us;
            processed_frames = 0;
            total_processing_us = 0;
            max_processing_us = 0;
        }

        /* Send ball measurement to ESP32-S3 */
        const esp_err_t measurement_result = ball_now_send_detection(frame_seq, &detection);

        if (measurement_result != ESP_OK) {
            ESP_LOGW(TAG, "Measurement send failed: %s", esp_err_to_name(measurement_result));
        }

        /* Submit camera frame for UDP streaming */
        const esp_err_t stream_result = wifi_stream_submit_frame(frame_seq, frame->buf, frame->len);

#if CAMERA_DEBUG
        if (stream_result == ESP_ERR_TIMEOUT) {
            dropped_frames_since_report++;
        }
#endif

        if (stream_result != ESP_OK && stream_result != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Frame streaming failed: %s", esp_err_to_name(stream_result));
        }

        esp_camera_fb_return(frame);

        frame_seq++;

#if CAMERA_DEBUG
        frames_since_report++;

        const int64_t debug_now_us = esp_timer_get_time();
        const int64_t elapsed_us = debug_now_us - last_debug_report_us;

        if (elapsed_us >= 1000000) {
            const uint32_t fps_x10 = (uint32_t)(((uint64_t)frames_since_report * 10000000ULL) / (uint64_t)elapsed_us);
            const uint32_t frame_cycle_ms = (uint32_t)((debug_now_us - frame_start_us) / 1000);

            ESP_LOGI(
                TAG,
                "frames=%" PRIu32
                " size=%ux%u"
                " last_cycle=%" PRIu32 " ms"
                " fps=%" PRIu32 ".%" PRIu32
                " stream_dropped=%" PRIu32,
                frame_seq,
                (unsigned)CAMERA_FRAME_WIDTH,
                (unsigned)CAMERA_FRAME_HEIGHT,
                frame_cycle_ms,
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
            last_debug_report_us = debug_now_us;
        }
#endif
    }
}

esp_err_t camera_start(void)
{
    const BaseType_t result = xTaskCreatePinnedToCore(
        camera_task,
        "camera_task",
        CAMERA_TASK_STACK_SIZE,
        NULL,
        CAMERA_TASK_PRIORITY,
        NULL,
        CAMERA_TASK_CORE
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create camera task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera task started on Core %d", CAMERA_TASK_CORE);

    return ESP_OK;
}