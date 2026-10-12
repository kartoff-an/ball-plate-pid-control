#ifndef WIFI_STREAM_H
#define WIFI_STREAM_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t wifi_stream_init(void);
esp_err_t wifi_stream_submit_frame(const uint8_t *pixels, size_t frame_length);

#endif