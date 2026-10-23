#include "camera_capture.h"

#include "esp_log.h"

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

#define CAMERA_XCLK_FREQ_HZ 20000000

static const char *TAG = "CAMERA_CAPTURE";

esp_err_t camera_capture_init(void)
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

    const esp_err_t initialization_result = esp_camera_init(&camera_config);

    if (initialization_result != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(initialization_result));
        return initialization_result;
    }

    ESP_LOGI(TAG, "Camera initialized");

    return ESP_OK;
}

camera_fb_t *camera_capture_get_frame(void)
{
    return esp_camera_fb_get();
}

void camera_capture_release_frame(camera_fb_t *frame_buffer)
{
    if (frame_buffer != NULL) {
        esp_camera_fb_return(frame_buffer);
    }
}

camera_image_stats_t camera_capture_get_image_stats(
    const uint8_t *grayscale_pixels,
    size_t pixel_count)
{
    camera_image_stats_t image_stats = {
        .min = 255,
        .max = 0,
        .mean = 0,
    };

    if (grayscale_pixels == NULL || pixel_count == 0) {
        return image_stats;
    }

    uint64_t sum = 0;
    uint8_t min_intensity = 255;
    uint8_t max_intensity = 0;

    for (size_t i = 0; i < pixel_count; ++i) {
        const uint8_t pixel_intensity = grayscale_pixels[i];

        if (pixel_intensity < min_intensity) {
            min_intensity = pixel_intensity;
        }

        if (pixel_intensity > max_intensity) {
            max_intensity = pixel_intensity;
        }

        sum += pixel_intensity;
    }

    image_stats.min = min_intensity;
    image_stats.max = max_intensity;
    image_stats.mean = (uint8_t)(sum / pixel_count);

    return image_stats;
}