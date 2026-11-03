/**
 * @file ball_receiver.h
 * @brief ESP-NOW receiver interface for ball position measurements.
 *
 * Provides functions to initialize the ESP-NOW receiver and retrieve
 * the latest valid ball measurement received from the ESP32-CAM.
 *
 * The receiver stores only the most recently received valid measurement.
 * Measurement freshness is reported to the caller so that the control
 * task can reject stale data.
 */

#ifndef BALL_RECEIVER_H
#define BALL_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ball_measurement.h"

/**
 * @brief Initialize the ESP-NOW measurement receiver.
 *
 * Initializes the ESP-NOW driver and registers the receive callback
 * responsible for validating and storing incoming ball measurements.
 *
 * @return
 *      - ESP_OK: Receiver initialized successfully.
 *      - Other ESP-IDF error code: ESP-NOW initialization or callback
 *        registration failed.
 *
 * @note This function assumes ESP-NOW has not already been initialized
 *       elsewhere in the application.
 * @note Successful initialization does not guarantee that the receiver
 *       has already received a measurement.
 */
esp_err_t ball_receiver_init(void);

/**
 * @brief Retrieve the latest valid ball measurement.
 *
 * Copies the most recently received valid measurement and calculates
 * its age based on the local ESP timer.
 *
 * @param[out] measurement Pointer to the destination measurement structure.
 * @param[out] age_ms Pointer to the destination measurement age in milliseconds.
 *
 * @return
 *      - true: A valid measurement has been received and copied.
 *      - false: Either an argument is NULL or no measurement has been received.
 *
 * @note This function does not reject stale measurements. The caller
 *       must compare @p age_ms against its maximum permitted measurement age.
 * @note A measurement with detected == 0 is still considered a valid
 *       received measurement. Check the detected field before using
 *       its coordinates.
 */
bool ball_receiver_get_latest(ball_measurement_t *measurement, int64_t *age_ms);

#endif /* BALL_RECEIVER_H*/
