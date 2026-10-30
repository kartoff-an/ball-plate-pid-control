#ifndef BALL_DETECTION_H
#define BALL_DETECTION_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "camera_config.h"

#define BALL_DETECTION_FRAME_WIDTH          CAMERA_FRAME_WIDTH
#define BALL_DETECTION_FRAME_HEIGHT         CAMERA_FRAME_HEIGHT

#define BALL_DETECTION_CROP_WIDTH           CAMERA_FRAME_HEIGHT
#define BALL_DETECTION_CROP_HEIGHT          CAMERA_FRAME_HEIGHT

#define BALL_DETECTION_CROP_X_OFFSET        20U
#define BALL_DETECTION_CROP_Y_OFFSET        0U

#define BALL_DETECTION_CROP_SIZE            (BALL_DETECTION_CROP_WIDTH * BALL_DETECTION_CROP_HEIGHT)

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

typedef struct {
    uint8_t threshold;
    uint16_t min_area;
    uint16_t max_area;
    float min_aspect_ratio;
    float max_aspect_ratio;
    float min_fill_ratio;
    float max_jump_px;
    float smoothing_alpha;
} ball_detection_config_t;

void ball_detection_get_config(ball_detection_config_t *config);

esp_err_t ball_detection_set_config(const ball_detection_config_t *config);

esp_err_t ball_detection_detect(
    const uint8_t *frame,
    uint8_t *binary_frame,
    ball_detection_result_t *result
);

void ball_detection_reset_tracking(void);

#endif