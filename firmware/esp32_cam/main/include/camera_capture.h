#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"

typedef struct {
    uint8_t min;
    uint8_t max;
    uint8_t mean;
} camera_image_stats_t;

esp_err_t camera_capture_init(void);

camera_fb_t *camera_capture_get_frame(void);

void camera_capture_release_frame(camera_fb_t *frame_buffer);

camera_image_stats_t camera_capture_get_image_stats(
    const uint8_t *grayscale_pixels,
    size_t pixel_count
);

#endif