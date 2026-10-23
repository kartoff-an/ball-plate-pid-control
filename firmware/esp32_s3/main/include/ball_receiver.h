#ifndef BALL_RECEIVER_H
#define BALL_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ball_measurement.h"

esp_err_t ball_receiver_init(void);

bool ball_receiver_get_latest(ball_measurement_t *measurement, int64_t *age_ms);

#endif
