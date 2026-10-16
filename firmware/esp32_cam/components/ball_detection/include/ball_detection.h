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

esp_err_t detect_ball(
    const uint8_t *gray_buf,
    uint16_t width,
    uint16_t height,
    uint8_t threshold,
    uint32_t min_area,
    uint32_t max_area,
    uint8_t *binary_buf,
    ball_detection_result_t *result
);

#endif