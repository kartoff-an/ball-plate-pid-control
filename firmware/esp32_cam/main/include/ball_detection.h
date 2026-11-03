/**
 * @file ball_detection.h
 * @brief Public interface for grayscale ball detection and tracking.
 *
 * Defines the image crop, detection result, configurable detection
 * parameters, and functions used to detect and track a ball.
 */

#ifndef BALL_DETECTION_H
#define BALL_DETECTION_H

#include <stdbool.h>
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

/**
 * @brief Result produced by the ball detection algorithm.
 *
 * Coordinates are expressed relative to the original camera frame,
 * including the configured crop offsets.
 *
 * When no valid ball is detected, the result fields are zeroed and
 * @c detected is false.
 */
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

/**
 * @brief Configuration parameters for ball detection and tracking.
 *
 * These parameters control thresholding, candidate filtering,
 * tracking displacement, and position smoothing.
 */
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

/**
 * @brief Get the currently active detection configuration.
 *
 * @param[out] config Destination for the configuration copy.
 *
 * If @p config is NULL, the function does nothing.
 */
void ball_detection_get_config(ball_detection_config_t *config);

/**
 * @brief Validate and apply a new detection configuration.
 *
 * Applying a configuration also requests a tracking reset. The reset
 * is performed by the next call to ball_detection_detect().
 *
 * @param[in] config Configuration to validate and apply.
 *
 * @return
 * - ESP_OK: Configuration applied successfully.
 * - ESP_ERR_INVALID_ARG: Configuration is NULL or contains invalid values.
 */
esp_err_t ball_detection_set_config(const ball_detection_config_t *config);

/**
 * @brief Detect and track a ball in a grayscale camera frame.
 *
 * Processes the configured square crop using thresholding and
 * connected-component labeling. Valid components are filtered and
 * selected using size, shape, and tracking constraints.
 *
 * The selected position is smoothed when a previous tracking position
 * is available.
 *
 * @param[in] frame Full grayscale camera frame.
 * @param[out] binary_frame Binary image buffer for the detection crop.
 *                           Each pixel is set to either 0 or 255.
 * @param[out] result Detection result, including position and component
 *                    information.
 *
 * @return
 * - ESP_OK: Processing completed, whether or not a ball was detected.
 * - ESP_ERR_INVALID_ARG: One or more arguments are NULL.
 * - ESP_ERR_INVALID_SIZE: Connected-component queue capacity was exceeded.
 *
 * @note The input frame must contain at least
 *       BALL_DETECTION_FRAME_WIDTH * BALL_DETECTION_FRAME_HEIGHT pixels.
 * @note The binary output buffer must contain at least
 *       BALL_DETECTION_CROP_SIZE pixels.
 */
esp_err_t ball_detection_detect(
    const uint8_t *frame,
    uint8_t *binary_frame,
    ball_detection_result_t *result
);

/**
 * @brief Request a reset of the ball tracking state.
 *
 * The reset is deferred until the next call to
 * ball_detection_detect().
 */
void ball_detection_reset_tracking(void);

#endif /* BALL_DETECTION_H*/