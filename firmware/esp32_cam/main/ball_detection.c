#include "ball_detection.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MAX_IMAGE_PIXELS            BALL_DETECTION_CROP_SIZE

/*
 * Component filtering.
 */
#define BALL_MAX_ASPECT_RATIO       2.5f
#define BALL_MIN_BBOX_FILL_PERCENT  30U
#define MAX_BALL_DISPLACEMENT_PX    25.0f

static uint16_t component_queue[MAX_IMAGE_PIXELS];

static bool s_have_previous_position = false;
static uint16_t s_previous_x = 0;
static uint16_t s_previous_y = 0;

static bool is_valid_pixel(int pixel_x, int pixel_y)
{
    return pixel_x >= 0 && pixel_y >= 0 && pixel_x < (int)BALL_DETECTION_CROP_WIDTH && pixel_y < (int)BALL_DETECTION_CROP_HEIGHT;
}

static float squaref(float value)
{
    return value * value;
}

static bool candidate_matches_shape(
    uint32_t component_area,
    uint16_t minimum_x,
    uint16_t minimum_y,
    uint16_t maximum_x,
    uint16_t maximum_y)
{
    const uint32_t bbox_width = (uint32_t)maximum_x - minimum_x + 1U;
    const uint32_t bbox_height = (uint32_t)maximum_y - minimum_y + 1U;
    const uint32_t shorter_side = bbox_width < bbox_height ? bbox_width : bbox_height;
    const uint32_t longer_side = bbox_width > bbox_height ? bbox_width : bbox_height;
    const uint32_t bbox_area = bbox_width * bbox_height;

    if (shorter_side == 0 || bbox_area == 0) {
        return false;
    }

    /*
     * Reject excessively elongated components.
     */
    if ((float)longer_side >
        BALL_MAX_ASPECT_RATIO * (float)shorter_side) {
        return false;
    }

    /*
     * Reject sparse or irregular bounding boxes.
     */
    if ((uint64_t)component_area * 100U < (uint64_t)bbox_area * BALL_MIN_BBOX_FILL_PERCENT) {
        return false;
    }

    return true;
}

esp_err_t ball_detection_detect(
    const uint8_t *grayscale_pixels,
    uint8_t threshold,
    uint32_t min_area,
    uint32_t max_area,
    uint8_t *binary_pixels,
    ball_detection_result_t *detection_result)
{
    if (grayscale_pixels == NULL || binary_pixels == NULL || detection_result == NULL ||
        min_area == 0 || max_area == 0 || max_area < min_area
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Reset every output field for this frame.
     */
    memset(detection_result, 0, sizeof(*detection_result));

    /*
     * Threshold the centered crop.
     *
     * Source frame: 160 x 120
     * Crop origin:  (20, 0)
     * Crop size:    120 x 120
     *
     * Coordinates returned by this function are crop coordinates.
     */
    for (uint16_t y = 0; y < BALL_DETECTION_CROP_HEIGHT; ++y) {
        const size_t source_row = (size_t)(y + BALL_DETECTION_CROP_Y_OFFSET) * BALL_DETECTION_FRAME_WIDTH;
        const size_t crop_row = (size_t)y * BALL_DETECTION_CROP_WIDTH;

        for (uint16_t x = 0; x < BALL_DETECTION_CROP_WIDTH; ++x) {
            const size_t source_index = source_row + x + BALL_DETECTION_CROP_X_OFFSET;
            const size_t crop_index = crop_row + x;

            if (grayscale_pixels[source_index] > threshold) {
                binary_pixels[crop_index] = 255;
                detection_result->white_pixels++;
            } else {
                binary_pixels[crop_index] = 0;
            }
        }
    }

    bool found_candidate = false;

    uint16_t best_x = 0;
    uint16_t best_y = 0;
    uint16_t best_min_x = 0;
    uint16_t best_min_y = 0;
    uint16_t best_max_x = 0;
    uint16_t best_max_y = 0;

    uint32_t best_area = 0;

    float best_distance_squared = 0.0f;

    /*
     * Traverse connected components using 8-connectivity.
     * Each white pixel is cleared when queued, so it is
     * visited at most once.
     */
    for (size_t start_index = 0; start_index < MAX_IMAGE_PIXELS; ++start_index) {
        if (binary_pixels[start_index] != 255) {
            continue;
        }

        size_t queue_head = 0;
        size_t queue_tail = 0;

        uint32_t component_area = 0;
        uint64_t sum_pixel_x = 0;
        uint64_t sum_pixel_y = 0;

        uint16_t minimum_x = BALL_DETECTION_CROP_WIDTH - 1U;
        uint16_t minimum_y = BALL_DETECTION_CROP_HEIGHT - 1U;
        uint16_t maximum_x = 0;
        uint16_t maximum_y = 0;

        component_queue[queue_tail++] = (uint16_t)start_index;
        binary_pixels[start_index] = 0;

        while (queue_head < queue_tail) {
            const uint16_t pixel_index = component_queue[queue_head++];
            const uint16_t pixel_x = pixel_index % BALL_DETECTION_CROP_WIDTH;
            const uint16_t pixel_y = pixel_index / BALL_DETECTION_CROP_WIDTH;

            component_area++;
            sum_pixel_x += pixel_x;
            sum_pixel_y += pixel_y;

            if (pixel_x < minimum_x) minimum_x = pixel_x;
            if (pixel_y < minimum_y) minimum_y = pixel_y;
            if (pixel_x > maximum_x) maximum_x = pixel_x;
            if (pixel_y > maximum_y) maximum_y = pixel_y;

            for (int offset_y = -1; offset_y <= 1; ++offset_y) {
                for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                    if (offset_x == 0 && offset_y == 0) {
                        continue;
                    }

                    const int neighbor_x = (int)pixel_x + offset_x;
                    const int neighbor_y = (int)pixel_y + offset_y;

                    if (!is_valid_pixel(neighbor_x, neighbor_y)) {
                        continue;
                    }

                    const size_t neighbor_index = (size_t)neighbor_y * BALL_DETECTION_CROP_WIDTH + (size_t)neighbor_x;

                    if (binary_pixels[neighbor_index] != 255) {
                        continue;
                    }

                    /*
                     * Mark the pixel before queueing it to
                     * prevent duplicate queue entries.
                     */
                    binary_pixels[neighbor_index] = 0;

                    if (queue_tail >= MAX_IMAGE_PIXELS) {
                        return ESP_ERR_INVALID_SIZE;
                    }

                    component_queue[queue_tail++] =
                        (uint16_t)neighbor_index;
                }
            }
        }

        if (component_area < min_area || component_area > max_area) {
            continue;
        }

        if (!candidate_matches_shape(component_area, minimum_x, minimum_y, maximum_x, maximum_y)) {
            continue;
        }

        const uint16_t center_x = (uint16_t)(sum_pixel_x / component_area);
        const uint16_t center_y = (uint16_t)(sum_pixel_y / component_area);
        float distance_squared = 0.0f;

        if (s_have_previous_position) {
            distance_squared = squaref((float)center_x - (float)s_previous_x) + squaref((float)center_y - (float)s_previous_y);

            if (distance_squared > squaref(MAX_BALL_DISPLACEMENT_PX)) {
                continue;
            }
        }

        /*
        * Choose the nearest candidate when tracking.
        * Without a previous position, choose the largest candidate.
        */
        bool select_candidate = false;

        if (!found_candidate) {
            select_candidate = true;
        } else if (s_have_previous_position) {
            select_candidate = distance_squared < best_distance_squared;
        } else {
            select_candidate = component_area > best_area;
        }

        if (!select_candidate) {
            continue;
        }

        found_candidate = true;
        best_distance_squared = distance_squared;

        best_area = component_area;
        best_x = center_x;
        best_y = center_y;
        best_min_x = minimum_x;
        best_min_y = minimum_y;
        best_max_x = maximum_x;
        best_max_y = maximum_y;
    }

    if (!found_candidate) {
        return ESP_OK;
    }

    detection_result->detected = true;
    detection_result->area = best_area;
    detection_result->center_x = best_x;
    detection_result->center_y = best_y;
    detection_result->min_x = best_min_x;
    detection_result->min_y = best_min_y;
    detection_result->max_x = best_max_x;
    detection_result->max_y = best_max_y;

    s_previous_x = best_x;
    s_previous_y = best_y;
    s_have_previous_position = true;

    return ESP_OK;
}