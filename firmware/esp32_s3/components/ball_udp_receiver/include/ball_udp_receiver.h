#ifndef BALL_UDP_RECEIVER_H
#define BALL_UDP_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t frame_sequence;
    bool is_detected;
    uint16_t center_x;
    uint16_t center_y;
    uint32_t ball_area;
    int64_t received_at_us;
} ball_measurement_t;

esp_err_t ball_udp_receiver_init(void);

bool ball_udp_receiver_get_latest(ball_measurement_t *measurement);

#endif
