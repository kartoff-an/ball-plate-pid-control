/**
 * @file ball_detection.c
 * @brief Ball detection, candidate filtering, and position tracking.
 *
 * Implements the following processing pipeline:
 * 1. Image thresholding.
 * 2. Connected-component labeling using eight-connectivity.
 * 3. Candidate filtering based on area and shape.
 * 4. Candidate selection using component size or tracking distance.
 * 5. Tracking loss handling.
 * 6. Position smoothing.
 * 7. Tracking state update.
 */

#include "ball_detection.h"

#include <stdbool.h>
#include <stdint.h>
<<<<<<< HEAD
=======
#include <stddef.h>
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f
#include <string.h>
#include <math.h>

#define MAX_IMAGE_PIXELS            BALL_DETECTION_CROP_SIZE

/*
 * Component filtering.
 */
#define BALL_MAX_ASPECT_RATIO       2.5f
#define BALL_MIN_BBOX_FILL_PERCENT  30U
#define MAX_BALL_DISPLACEMENT_PX    25.0f

<<<<<<< HEAD
static uint16_t component_queue[MAX_IMAGE_PIXELS];

static bool s_have_previous_position = false;
static uint16_t s_previous_x = 0;
static uint16_t s_previous_y = 0;

static bool is_valid_pixel(int pixel_x, int pixel_y)
=======
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define MAX_IMAGE_PIXELS BALL_DETECTION_CROP_SIZE

static uint16_t component_queue[MAX_IMAGE_PIXELS];
static uint8_t visited[MAX_IMAGE_PIXELS];

static portMUX_TYPE config_lock = portMUX_INITIALIZER_UNLOCKED;
static ball_detection_config_t active_config = {
    .threshold = 100,
    .min_area = 400,
    .max_area = 500,
    .min_aspect_ratio = 0.75f,
    .max_aspect_ratio = 1.33f,
    .min_fill_ratio = 0.45f,
    .max_jump_px = 15.0f,
    .smoothing_alpha = 0.40f,
};

static bool tracking_valid;
static float tracked_x;
static float tracked_y;
static uint8_t missed_frames;
static bool tracking_reset_requested;

/**
 * @brief Represents a connected foreground component.
 *
 * Bounding-box coordinates are relative to the detection crop.
 * Center coordinates include the crop offset and use camera-frame
 * coordinates.
 */
typedef struct {
    bool valid;
    uint32_t area;
    uint16_t min_x, min_y, max_x, max_y;
    float center_x, center_y;
} ball_candidate_t;

static bool is_valid_pixel(int x, int y)
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f
{
    return x >= 0 && y >= 0 &&
           x < (int)BALL_DETECTION_CROP_WIDTH &&
           y < (int)BALL_DETECTION_CROP_HEIGHT;
}

static float squared_distance(float x1, float y1, float x2, float y2)
{
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    return dx * dx + dy * dy;
}

void ball_detection_get_config(ball_detection_config_t *config)
{
    if (config == NULL) return;
    portENTER_CRITICAL(&config_lock);
    *config = active_config;
    portEXIT_CRITICAL(&config_lock);
}

static bool config_is_valid(const ball_detection_config_t *c)
{
    return c != NULL &&
        c->min_area >= 1 && c->max_area >= c->min_area &&
        c->max_area <= BALL_DETECTION_CROP_SIZE &&
        c->min_aspect_ratio >= 0.10f && c->min_aspect_ratio <= 3.0f &&
        c->max_aspect_ratio >= c->min_aspect_ratio && c->max_aspect_ratio <= 5.0f &&
        c->min_fill_ratio >= 0.05f && c->min_fill_ratio <= 1.0f &&
        c->max_jump_px >= 1.0f && c->max_jump_px <= 120.0f &&
        c->smoothing_alpha >= 0.01f && c->smoothing_alpha <= 1.0f;
}

esp_err_t ball_detection_set_config(const ball_detection_config_t *config)
{
    if (!config_is_valid(config)) return ESP_ERR_INVALID_ARG;

    portENTER_CRITICAL(&config_lock);
    active_config = *config;
    portEXIT_CRITICAL(&config_lock);

    portENTER_CRITICAL(&config_lock);
    tracking_reset_requested = true;
    portEXIT_CRITICAL(&config_lock);
    return ESP_OK;
}

void ball_detection_reset_tracking(void)
{
    portENTER_CRITICAL(&config_lock);
    tracking_reset_requested = true;
    portEXIT_CRITICAL(&config_lock);
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
    uint8_t *binary_pixels,
    ball_detection_result_t *detection_result)
{
    if (grayscale_pixels == NULL || binary_pixels == NULL || detection_result == NULL)
        return ESP_ERR_INVALID_ARG;

    ball_detection_config_t config;
    bool reset_tracking;
    portENTER_CRITICAL(&config_lock);
    config = active_config;
    reset_tracking = tracking_reset_requested;
    tracking_reset_requested = false;
    portEXIT_CRITICAL(&config_lock);

    if (reset_tracking) {
        tracking_valid = false;
        tracked_x = 0.0f;
        tracked_y = 0.0f;
        missed_frames = 0;
    }

    memset(detection_result, 0, sizeof(*detection_result));
    memset(visited, 0, sizeof(visited));

    /*
<<<<<<< HEAD
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
=======
     * STAGE 1: IMAGE THRESHOLDING
     *
     * Extract the configured square crop from the grayscale frame.
     * COnvert each pixel into either: 255 = foreground, 0 = background
     *
     * The number of foreground pixels is also recorded.
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f
     */

    for (uint16_t y = 0; y < BALL_DETECTION_CROP_HEIGHT; ++y) {
        const size_t source_row = (size_t)(y + BALL_DETECTION_CROP_Y_OFFSET) * BALL_DETECTION_FRAME_WIDTH;
        const size_t crop_row = (size_t)y * BALL_DETECTION_CROP_WIDTH;
        for (uint16_t x = 0; x < BALL_DETECTION_CROP_WIDTH; ++x) {
            const size_t source_index = source_row + x + BALL_DETECTION_CROP_X_OFFSET;
            const size_t crop_index = crop_row + x;
            if (grayscale_pixels[source_index] > config.threshold) {
                binary_pixels[crop_index] = 255;
                detection_result->white_pixels++;
            } else {
                binary_pixels[crop_index] = 0;
            }
        }
    }

<<<<<<< HEAD
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
=======
    ball_candidate_t best = {0};
    float best_distance_sq = config.max_jump_px * config.max_jump_px;

    /*
     * STAGE 2: CONNECTED-COMPONENT LABELING
     *
     * Scan every pixel in the binary crop.
     * For each unvisited foreground pixel, perform BFS to find all connected foreground pixels
     * belonging to the same component.
     *
     * The BFS uses eight-connectivity: horizontal, vertical, and diagonal neighbors
     */
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f

    for (size_t start = 0; start < MAX_IMAGE_PIXELS; ++start) {
        if (binary_pixels[start] != 255 || visited[start]) continue;

<<<<<<< HEAD
        uint32_t component_area = 0;
        uint64_t sum_pixel_x = 0;
        uint64_t sum_pixel_y = 0;

        uint16_t minimum_x = BALL_DETECTION_CROP_WIDTH - 1U;
        uint16_t minimum_y = BALL_DETECTION_CROP_HEIGHT - 1U;
        uint16_t maximum_x = 0;
        uint16_t maximum_y = 0;
=======
        size_t head = 0, tail = 0;
        uint32_t area = 0, sum_x = 0, sum_y = 0;
        uint16_t min_x = BALL_DETECTION_CROP_WIDTH - 1;
        uint16_t min_y = BALL_DETECTION_CROP_HEIGHT - 1;
        uint16_t max_x = 0, max_y = 0;

        component_queue[tail++] = (uint16_t)start;
        visited[start] = 1;
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f

        while (head < tail) {
            const uint16_t index = component_queue[head++];
            const uint16_t x = index % BALL_DETECTION_CROP_WIDTH;
            const uint16_t y = index / BALL_DETECTION_CROP_WIDTH;
            area++;
            sum_x += x;
            sum_y += y;
            if (x < min_x) min_x = x;
            if (y < min_y) min_y = y;
            if (x > max_x) max_x = x;
            if (y > max_y) max_y = y;

<<<<<<< HEAD
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
=======
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = (int)x + dx, ny = (int)y + dy;
                    if (!is_valid_pixel(nx, ny)) continue;
                    const size_t ni = (size_t)ny * BALL_DETECTION_CROP_WIDTH + (size_t)nx;
                    if (binary_pixels[ni] != 255 || visited[ni]) continue;
                    visited[ni] = 1;
                    if (tail >= MAX_IMAGE_PIXELS) return ESP_ERR_INVALID_SIZE;
                    component_queue[tail++] = (uint16_t)ni;
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f
                }
            }
        }

<<<<<<< HEAD
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

=======
        /*
         * STAGE 3: CANDIDATE FILTERING
         *
         * Reject components are unlikely to represent the ball.
         */

        if (area < config.min_area || area > config.max_area) continue;
        if (min_x == 0 || min_y == 0 ||
            max_x == BALL_DETECTION_CROP_WIDTH - 1 ||
            max_y == BALL_DETECTION_CROP_HEIGHT - 1) continue;

        const uint16_t width = max_x - min_x + 1;
        const uint16_t height = max_y - min_y + 1;
        const float aspect = (float)width / (float)height;
        const float fill = (float)area / ((float)width * (float)height);
        if (aspect < config.min_aspect_ratio ||
            aspect > config.max_aspect_ratio ||
            fill < config.min_fill_ratio) continue;

        const float cx = (float)(sum_x / area) + BALL_DETECTION_CROP_X_OFFSET;
        const float cy = (float)(sum_y / area) + BALL_DETECTION_CROP_Y_OFFSET;

        /**
         * STAGE 4: CANDIDATE SELECTION
         *
         * Without an existing tracking position, select the largest valid compoent.
         *
         * With an existing tracking position, select the candidate closest to the previously tracked position,
         * subject to the maximum allowed displacement.
         */

        if (!tracking_valid) {
            if (!best.valid || area > best.area) {
                best = (ball_candidate_t){true, area, min_x, min_y, max_x, max_y, cx, cy};
            }
        } else {
            const float d2 = squared_distance(cx, cy, tracked_x, tracked_y);
            if (d2 > best_distance_sq) continue;
            if (!best.valid || d2 < best_distance_sq ||
                (d2 == best_distance_sq && area > best.area)) {
                best = (ball_candidate_t){true, area, min_x, min_y, max_x, max_y, cx, cy};
                best_distance_sq = d2;
            }
        }
    }

    /**
     * STAGE 6: TRACKING LOSS HANDLING
     *
     * If no valid candidate is found, increment the missed-frame counter.
     * After more than three consecutive missed frames, invalidate tracking.
     * The next valid detection will then use the initial largest-component selection strategy.
     */
    if (!best.valid) {
        if (missed_frames < 3) missed_frames++;
        else tracking_valid = false;
        return ESP_OK;
    }

    /**
     * STAGE 5: POSITION SMOOTHING
     *
     * Apply exponential smoothing when a previous tracking exists.
     * A smaller alpha produces stronger smoothing but increases response delay.
     * An alpha of 1.0 uses the current measurement without smoothing.
     */
    float out_x = best.center_x;
    float out_y = best.center_y;
    if (tracking_valid) {
        out_x = config.smoothing_alpha * best.center_x + (1.0f - config.smoothing_alpha) * tracked_x;
        out_y = config.smoothing_alpha * best.center_y + (1.0f - config.smoothing_alpha) * tracked_y;
    }

    /**
     * STAGE 6: TRACKING UPDATE
     *
     * Store the smoothed position for the next frame and reset the missed-frame
     * counter after a successful detection.
     */
    tracked_x = out_x;
    tracked_y = out_y;
    tracking_valid = true;
    missed_frames = 0;

    detection_result->detected = true;
    detection_result->area = best.area;
    detection_result->center_x = (uint16_t)(out_x + 0.5f);
    detection_result->center_y = (uint16_t)(out_y + 0.5f);
    detection_result->min_x = best.min_x + BALL_DETECTION_CROP_X_OFFSET;
    detection_result->min_y = best.min_y + BALL_DETECTION_CROP_Y_OFFSET;
    detection_result->max_x = best.max_x + BALL_DETECTION_CROP_X_OFFSET;
    detection_result->max_y = best.max_y + BALL_DETECTION_CROP_Y_OFFSET;
>>>>>>> 1e69f2f22f0608f1c368b96a9ddc339d9af8553f
    return ESP_OK;
}
