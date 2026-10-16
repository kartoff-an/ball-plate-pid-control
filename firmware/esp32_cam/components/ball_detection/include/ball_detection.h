#ifndef BALL_DETECTION_H
#define BALL_DETECTION_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool detected;

    uint16_t center_x;
    uint16_t center_y;

    uint16_t min_x;
    uint16_t min_y;
    uint16_t max_x;
    uint16_t max_y;

    uint32_t area;
    size_t white_pixels;
} ball_detection_result_t;

esp_err_t ball_detection_detect(
    const uint8_t *grayscale_pixels,
    uint16_t image_width,
    uint16_t image_height,
    uint8_t threshold,
    uint32_t min_area,
    uint32_t max_area,
    uint8_t *binary_pixels,
    ball_detection_result_t *detection_result
);

#endif