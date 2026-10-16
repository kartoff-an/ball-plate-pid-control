#include "ball_detection.h"

#include <stdbool.h>
#include <limits.h>
#include <string.h>

#define MAX_IMAGE_PIXELS    (160U * 120U)

static uint16_t component_queue[MAX_IMAGE_PIXELS];

static bool is_valid_pixel(int x, int y, uint16_t width, uint16_t height)
{
    return x >= 0 && y >= 0 && x < (int)width && y < (int)height;
}

esp_err_t detect_ball(
    const uint8_t *gray_buf,
    uint16_t width,
    uint16_t height,
    uint8_t threshold,
    uint32_t min_area,
    uint32_t max_area,
    uint8_t *binary_buf,
    ball_detection_result_t *result)
{
    if (gray_buf == NULL || binary_buf == NULL || result == NULL ||
        width == 0 || height == 0 || min_area == 0 || max_area == 0 ||
        max_area < min_area
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t pixel_count = (size_t)width * height;
    if (pixel_count > MAX_IMAGE_PIXELS || pixel_count > UINT16_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(result, 0, sizeof(*result));

    // Threshold the grayscale image
    for (size_t i = 0; i < pixel_count; ++i) {
        if (gray_buf[i] > threshold) {
            binary_buf[i] = 255;
            result->white_pixels++;
        }
        else {
            binary_buf[i] = 0;
        }
    }

    uint32_t largest_area = 0;

    // Find and traverse each connected component
    for (size_t start = 0; start < pixel_count; ++start) {
        if (binary_buf[start] != 255) {
            continue;
        }

        size_t head = 0;
        size_t tail = 0;

        uint32_t area = 0;
        uint32_t sum_x = 0;
        uint32_t sum_y = 0;

        uint16_t min_x = width - 1;
        uint16_t min_y = height - 1;
        uint16_t max_x = 0;
        uint16_t max_y = 0;

        component_queue[tail++] = (uint16_t)start;
        binary_buf[start] = 0;

        while (head < tail) {
            const uint16_t index = component_queue[head++];
            const uint16_t x = index % width;
            const uint16_t y = index / width;

            area++;
            sum_x += x;
            sum_y += y;

            if (x < min_x) min_x = x;
            if (y < min_y) min_y = y;
            if (x > max_x) max_x = x;
            if (y > max_y) max_y = y;

            // examine the 8 neighboring pixels
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }

                    const int nx = (int)x + dx;
                    const int ny = (int)y + dy;

                    if (!is_valid_pixel(nx, ny, width, height)) {
                        continue;
                    }

                    const size_t neighbor_index = (size_t)ny * width + (size_t)nx;
                    if (binary_buf[neighbor_index] != 255) {
                        continue;
                    }

                    binary_buf[neighbor_index] = 0;

                    if (tail >= pixel_count) {
                        return ESP_ERR_INVALID_SIZE;
                    }

                    component_queue[tail++] = (uint16_t)neighbor_index;
                }
            }
        }

        // Reject components outside the area limits.
        if (area < min_area || area > max_area || area <= largest_area) {
            continue;
        }

        largest_area = area;

        result->detected = true;
        result->area = area;
        result->center_x = (uint16_t)(sum_x / area);
        result->center_y = (uint16_t)(sum_y / area);
        result->min_x = min_x;
        result->min_y = min_y;
        result->max_x = max_x;
        result->max_y = max_y;
    }

    return ESP_OK;
}