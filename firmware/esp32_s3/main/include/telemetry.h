#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t sequence;

    bool ball_detected;
    uint16_t ball_x;
    uint16_t ball_y;
    int64_t measurement_age_ms;

    float error_x;
    float error_y;

    float output_x_us;
    float output_y_us;

    uint16_t servo_x_us;
    uint16_t servo_y_us;

    bool controller_enabled;
} telemetry_data_t;

esp_err_t telemetry_init(void);
esp_err_t telemetry_send(const telemetry_data_t *data);
void telemetry_deinit(void);

#endif