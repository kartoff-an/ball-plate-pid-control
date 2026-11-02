/**
 * @file pid_controller.h
 * @brief Public interface for the ball-and-plate PID controller.
 *
 * Provides functions for configuring PID gains, setting the target ball
 * position, controlling the controller state, and querying its status.
 *
 * The controller operates independently on the X and Y axes. PID outputs
 * are expressed as servo pulse-width corrections relative to the neutral
 * servo position.
 *
 * @note Position coordinates and target values are expressed in millimeters.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Identifies the PID axis to configure or query.
 */
typedef enum {
    PID_AXIS_X,
    PID_AXIS_Y,
    PID_AXIS_BOTH
} pid_axis_id_t;

/**
 * @brief PID gain configuration.
 *
 * The gains determine the proportional, integral, and derivative
 * contributions to the controller output.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

/**
 * @brief Initializes and starts the PID control task.
 *
 * The task periodically reads the latest ball measurement, calculates
 * the position error, updates the PID controllers, and publishes telemetry.
 *
 * @return
 * - ESP_OK: Control task was created successfully.
 * - ESP_FAIL: FreeRTOS could not create the task.
 *
 * @note This function does not initialize the PCA9685. The servo driver
 *       must be initialized separately before enabling physical outputs.
 * @note The current implementation does not prevent multiple calls from
 *       creating multiple control tasks.
 */
esp_err_t pid_controller_init(void);

/**
 * @brief Updates the PID gains for the specified axis.
 *
 * @param[in] axis  Axis whose gains will be updated.
 * @param[in] gains Pointer to the new gain configuration.
 *
 * @return
 * - ESP_OK: Gains updated successfully.
 * - ESP_ERR_INVALID_ARG: Invalid axis, NULL pointer, non-finite gain,
 *   or gain outside the permitted range.
 *
 * @note PID gains are updated inside a critical section so the control
 *       task does not read partially updated gain values.
 * @note PID integral and derivative state are not reset by this operation.
 */
esp_err_t pid_controller_set_gains(pid_axis_id_t axis, const pid_gains_t *gains);

/**
 * @brief Retrieves the PID gains for one axis.
 *
 * @param[in]  axis  Axis whose gains will be retrieved. PID_AXIS_BOTH
 *                   is not supported.
 * @param[out] gains Destination for the retrieved gains.
 *
 * @return
 * - ESP_OK: Gains retrieved successfully.
 * - ESP_ERR_INVALID_ARG: Invalid axis or NULL destination pointer.
 */
esp_err_t pid_controller_get_gains(pid_axis_id_t axis, pid_gains_t * gains);

/**
 * @brief Sets the desired ball position on the plate.
 *
 * The target coordinates are expressed in millimeters relative to the
 * plate center. The accepted range is -110 mm to +110 mm on each axis.
 *
 * @param[in] x_mm Target X coordinate in millimeters.
 * @param[in] y_mm Target Y coordinate in millimeters.
 *
 * @return
 * - ESP_OK: Target updated successfully.
 * - ESP_ERR_INVALID_ARG: Coordinate is non-finite or outside the
 *   permitted range.
 */
esp_err_t pid_controller_set_target(float x_mm, float y_mm);

/**
 * @brief Enables PID control.
 *
 * Requests a controller-state reset before control calculations resume.
 * The control task processes this request on its next iteration.
 *
 * @return ESP_OK.
 *
 * @note Physical servo output is still governed by CONTROL_OUTPUT_ENABLED.
 */
esp_err_t pid_controller_start(void);

/**
 * @brief Disables PID control and requests neutral servo positions.
 *
 * The control task processes the disable and neutral-position requests
 * on its next iteration.
 *
 * @return ESP_OK.
 *
 * @note Physical servo output is still governed by CONTROL_OUTPUT_ENABLED.
 */
esp_err_t pid_controller_stop(void);

/**
 * @brief Requests a reset of the PID controller state.
 *
 * The control task clears the integral and previous-error values,
 * resets measurement sequence tracking, and clears the current outputs.
 *
 * @return ESP_OK.
 *
 * @note The reset is asynchronous. It is performed by the control task
 *       on its next iteration.
 */
esp_err_t pid_controller_reset(void);

/**
 * @brief Checks whether PID control is enabled.
 *
 * @return true if the controller is enabled; otherwise false.
 *
 * @note This reports the controller enable state, not whether physical
 *       servo output is enabled.
 */
bool pid_controller_is_enabled(void);

esp_err_t pid_controller_set_servo_neutral(uint16_t x_us, uint16_t y_us);

esp_err_t pid_controller_set_kalman_config(float measurement_variance, float acceleration_variance);

esp_err_t pid_controller_get_kalman_config(float *measurement_variance, float *acceleration_variance);

#endif /* PID_CONTROLLER_H*/
