
/**
 * @file kalman_filter.h
 * @brief 2D constant-velocity Kalman filter for ball position estimation.
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdbool.h>

typedef struct {
    float position;
    float velocity;

    // Symmetric 2x2 covariance matrix:
    // [ p_position          p_position_velocity ]
    // [ p_position_velocity p_velocity          ]
    float p_position;
    float p_position_velocity;
    float p_velocity;
} kalman_axis_t;

typedef struct {
    kalman_axis_t x;
    kalman_axis_t y;

    float measurement_variance;
    float acceleration_variance;

    bool initialized;
} kalman_filter_t;

/**
 * @brief Initialize the filter with default noise parameters.
 */
void kalman_filter_init(kalman_filter_t *filter);

/**
 * @brief Reset filter state.
 */
void kalman_filter_reset(kalman_filter_t *filter);

/**
 * @brief Process one new position measurement.
 *
 * @param filter Filter instance.
 * @param measured_x_mm Measured X position in mm.
 * @param measured_y_mm Measured Y position in mm.
 * @param dt_seconds Time since previous measurement in seconds.
 *
 * @return true if the filter successfully processed the measurement.
 */
bool kalman_filter_update(
    kalman_filter_t *filter,
    float measured_x_mm,
    float measured_y_mm,
    float dt_seconds
);

/**
 * @brief Retrieve filtered position and velocity.
 */
void kalman_filter_get_state(
    const kalman_filter_t *filter,
    float *x_mm,
    float *y_mm,
    float *vx_mm_s,
    float *vy_mm_s
);

#endif
