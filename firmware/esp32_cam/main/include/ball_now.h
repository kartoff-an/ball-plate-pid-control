#ifndef BALL_NOW_H
#define BALL_NOW_H

#include <stdint.h>
#include "esp_err.h"
#include "ball_detection.h"

esp_err_t ball_now_init(const uint8_t peer_mac[6]);

esp_err_t ball_now_send_detection(
    uint32_t sequence,
    const ball_detection_result_t *detection
);

#endif
