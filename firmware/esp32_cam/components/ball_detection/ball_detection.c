#include "ball_detection.h"

#include <stdbool.h>
#include <limits.h>
#include <string.h>

#define MAX_IMAGE_PIXELS    (160U * 120U)

static uint16_t component_queue[MAX_IMAGE_PIXELS];

static bool is_valid_pixel(int pixel_x, int pixel_y, uint16_t image_width, uint16_t image_height)
{
    return pixel_x >= 0 && pixel_y >= 0 && pixel_x < (int)image_width && pixel_y < (int)image_height;
}

esp_err_t ball_detection_detect(
    const uint8_t *grayscale_pixels,
    uint16_t image_width,
    uint16_t image_height,
    uint8_t threshold,
    uint32_t min_area,
    uint32_t max_area,
    uint8_t *binary_pixels,
    ball_detection_result_t *detection_result)
{
    if (grayscale_pixels == NULL || binary_pixels == NULL || detection_result == NULL ||
        image_width == 0 || image_height == 0 ||
        min_area == 0 || max_area == 0 || max_area < min_area
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t pixel_count = (size_t)image_width * image_height;
    if (pixel_count > MAX_IMAGE_PIXELS || pixel_count > UINT16_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(detection_result, 0, sizeof(*detection_result));

    // Threshold the grayscale image
    for (size_t i = 0; i < pixel_count; ++i) {
        if (grayscale_pixels[i] > threshold) {
            binary_pixels[i] = 255;
            detection_result->white_pixels++;
        }
        else {
            binary_pixels[i] = 0;
        }
    }

    uint32_t largest_area = 0;

    // Find and traverse each connected component
    for (size_t component_start_index = 0; component_start_index < pixel_count; ++component_start_index) {
        if (binary_pixels[component_start_index] != 255) {
            continue;
        }

        size_t queue_head = 0;
        size_t queue_tail = 0;

        uint32_t component_area = 0;
        uint32_t sum_pixel_x = 0;
        uint32_t sum_pixel_y = 0;

        uint16_t minimum_x = image_width - 1;
        uint16_t minimum_y = image_height - 1;
        uint16_t maximum_x = 0;
        uint16_t maximum_y = 0;

        component_queue[queue_tail++] = (uint16_t)component_start_index;
        binary_pixels[component_start_index] = 0;

        while (queue_head < queue_tail) {
            const uint16_t pixel_index = component_queue[queue_head++];
            const uint16_t pixel_x = pixel_index % image_width;
            const uint16_t pixel_y = pixel_index / image_width;

            component_area++;
            sum_pixel_x += pixel_x;
            sum_pixel_y += pixel_y;

            if (pixel_x < minimum_x) minimum_x = pixel_x;
            if (pixel_y < minimum_y) minimum_y = pixel_y;
            if (pixel_x > maximum_x) maximum_x = pixel_x;
            if (pixel_y > maximum_y) maximum_y = pixel_y;

            // examine the 8 neighboring pixels
            for (int neighbor_offset_y = -1; neighbor_offset_y <= 1; ++neighbor_offset_y) {
                for (int neighbor_offset_x = -1; neighbor_offset_x <= 1; ++neighbor_offset_x) {
                    if (neighbor_offset_x == 0 && neighbor_offset_y == 0) {
                        continue;
                    }

                    const int neighbor_x = (int)pixel_x + neighbor_offset_x;
                    const int neighbor_y = (int)pixel_y + neighbor_offset_y;

                    if (!is_valid_pixel(neighbor_x, neighbor_y, image_width, image_height)) {
                        continue;
                    }

                    const size_t neighbor_pixel_index =
                        (size_t)neighbor_y * image_width + (size_t)neighbor_x;
                    if (binary_pixels[neighbor_pixel_index] != 255) {
                        continue;
                    }

                    binary_pixels[neighbor_pixel_index] = 0;

                    if (queue_tail >= pixel_count) {
                        return ESP_ERR_INVALID_SIZE;
                    }

                    component_queue[queue_tail++] = (uint16_t)neighbor_pixel_index;
                }
            }
        }

        // Reject components outside the area limits.
        if (component_area < min_area || component_area > max_area || component_area <= largest_area) {
            continue;
        }

        largest_area = component_area;

        detection_result->detected = true;
        detection_result->area = component_area;
        detection_result->center_x = (uint16_t)(sum_pixel_x / component_area);
        detection_result->center_y = (uint16_t)(sum_pixel_y / component_area);
        detection_result->min_x = minimum_x;
        detection_result->min_y = minimum_y;
        detection_result->max_x = maximum_x;
        detection_result->max_y = maximum_y;
    }

    return ESP_OK;
}