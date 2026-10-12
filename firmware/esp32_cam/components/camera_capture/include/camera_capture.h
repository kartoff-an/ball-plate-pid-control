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
} image_stats_t;

esp_err_t camera_capture_init(void);

camera_fb_t *get_capture_frame(void);

void release_capture_frame(camera_fb_t *frame);

image_stats_t calculate_image_stats(const uint8_t *gray_buf, size_t pixel_count);

#endif