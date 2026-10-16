#ifndef WIFI_STREAM_H
#define WIFI_STREAM_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "ball_detection.h"

esp_err_t wifi_stream_init(void);
esp_err_t wifi_stream_submit_frame(uint32_t sequence, const uint8_t *pixels, size_t frame_length);
esp_err_t wifi_stream_submit_detection(uint32_t sequence, const ball_detection_result_t *result);

#endif