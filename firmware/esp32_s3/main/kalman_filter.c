
/**
 * @file kalman_filter.c
 * @brief 2D constant-velocity Kalman filter implementation.
 */

#include <math.h>
#include <string.h>

#include "kalman_filter.h"

#define KALMAN_MEASUREMENT_VARIANCE       4.0f
#define KALMAN_ACCELERATION_VARIANCE      500.0f

#define KALMAN_INITIAL_POSITION_VARIANCE  4.0f
#define KALMAN_INITIAL_VELOCITY_VARIANCE  2500.0f

#define KALMAN_MAX_DT_SECONDS             0.25f

static void axis_initialize(kalman_axis_t *axis, float position)
{
    axis->position = position;
    axis->velocity = 0.0f;

    axis->p_position = KALMAN_INITIAL_POSITION_VARIANCE;
    axis->p_position_velocity = 0.0f;
    axis->p_velocity = KALMAN_INITIAL_VELOCITY_VARIANCE;
}

static void axis_predict(
    kalman_axis_t *axis,
    float dt,
    float acceleration_variance)
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    const float old_p_position = axis->p_position;
    const float old_p_position_velocity = axis->p_position_velocity;
    const float old_p_velocity = axis->p_velocity;

    // Constant-velocity state prediction.
    axis->position += axis->velocity * dt;

    // Covariance prediction: F * P * F' + Q.
    axis->p_position =
        old_p_position +
        2.0f * dt * old_p_position_velocity +
        dt2 * old_p_velocity +
        acceleration_variance * dt4 * 0.25f;

    axis->p_position_velocity =
        old_p_position_velocity +
        dt * old_p_velocity +
        acceleration_variance * dt3 * 0.5f;

    axis->p_velocity =
        old_p_velocity +
        acceleration_variance * dt2;
}

static void axis_update(
    kalman_axis_t *axis,
    float measurement,
    float measurement_variance)
{
    const float innovation = measurement - axis->position;
    const float innovation_variance =
        axis->p_position + measurement_variance;

    if (innovation_variance <= 0.0f) {
        return;
    }

    const float gain_position =
        axis->p_position / innovation_variance;

    const float gain_velocity =
        axis->p_position_velocity / innovation_variance;

    const float old_p_position = axis->p_position;
    const float old_p_position_velocity = axis->p_position_velocity;
    const float old_p_velocity = axis->p_velocity;

    // State correction.
    axis->position += gain_position * innovation;
    axis->velocity += gain_velocity * innovation;

    // Covariance correction.
    axis->p_position =
        (1.0f - gain_position) * old_p_position;

    axis->p_position_velocity =
        (1.0f - gain_position) * old_p_position_velocity;

    axis->p_velocity =
        old_p_velocity -
        gain_velocity * old_p_position_velocity;

    // Maintain covariance symmetry and avoid small negative roundoff.
    if (axis->p_position < 0.0f) {
        axis->p_position = 0.0f;
    }

    if (axis->p_velocity < 0.0f) {
        axis->p_velocity = 0.0f;
    }
}

void kalman_filter_init(kalman_filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    memset(filter, 0, sizeof(*filter));

    filter->measurement_variance =
        KALMAN_MEASUREMENT_VARIANCE;

    filter->acceleration_variance =
        KALMAN_ACCELERATION_VARIANCE;
}

void kalman_filter_reset(kalman_filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->initialized = false;

    memset(&filter->x, 0, sizeof(filter->x));
    memset(&filter->y, 0, sizeof(filter->y));
}

bool kalman_filter_update(
    kalman_filter_t *filter,
    float measured_x_mm,
    float measured_y_mm,
    float dt_seconds)
{
    if (filter == NULL ||
        !isfinite(measured_x_mm) ||
        !isfinite(measured_y_mm)) {
        return false;
    }

    if (!filter->initialized ||
        !isfinite(dt_seconds) ||
        dt_seconds <= 0.0f ||
        dt_seconds > KALMAN_MAX_DT_SECONDS) {

        axis_initialize(&filter->x, measured_x_mm);
        axis_initialize(&filter->y, measured_y_mm);

        filter->initialized = true;
        return true;
    }

    axis_predict(
        &filter->x,
        dt_seconds,
        filter->acceleration_variance
    );

    axis_predict(
        &filter->y,
        dt_seconds,
        filter->acceleration_variance
    );

    axis_update(
        &filter->x,
        measured_x_mm,
        filter->measurement_variance
    );

    axis_update(
        &filter->y,
        measured_y_mm,
        filter->measurement_variance
    );

    return true;
}

void kalman_filter_get_state(
    const kalman_filter_t *filter,
    float *x_mm,
    float *y_mm,
    float *vx_mm_s,
    float *vy_mm_s)
{
    if (filter == NULL || !filter->initialized) {
        return;
    }

    if (x_mm != NULL) {
        *x_mm = filter->x.position;
    }

    if (y_mm != NULL) {
        *y_mm = filter->y.position;
    }

    if (vx_mm_s != NULL) {
        *vx_mm_s = filter->x.velocity;
    }

    if (vy_mm_s != NULL) {
        *vy_mm_s = filter->y.velocity;
    }
}
