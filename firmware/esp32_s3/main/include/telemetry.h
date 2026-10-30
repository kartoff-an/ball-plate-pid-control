/**
 * @file telemetry.h
 * @brief Public interface and data structures for UDP telemetry.
 *
 * Defines the telemetry sample exchanged between the PID controller
 * and the telemetry transmission task.
 *
 * Telemetry includes ball detection data, measurement age, position
 * errors, PID outputs, servo pulse widths, and controller state.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Telemetry snapshot published by the control system.
 *
 * Each instance represents one processed ball measurement and its
 * corresponding controller state.
 *
 * Position errors are expressed in millimeters. PID outputs and servo
 * commands are expressed in microseconds.
 */
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

/**
 * @brief Initializes the UDP telemetry service.
 *
 * Creates a one-element FreeRTOS queue, opens a UDP socket, enables
 * broadcast transmission, and starts the telemetry task.
 *
 * The telemetry task periodically retrieves the latest queued sample
 * and broadcasts it to the configured destination.
 *
 * @return
 * - ESP_OK: Telemetry service initialized successfully.
 * - ESP_ERR_INVALID_STATE: Service is already initialized.
 * - ESP_ERR_NO_MEM: Queue allocation failed.
 * - ESP_FAIL: Socket creation, socket configuration, or task creation failed.
 *
 * @note The current destination is configured in telemetry.c.
 * @note Successful initialization does not guarantee that a PC is
 *       listening for telemetry packets.
 */
esp_err_t telemetry_init(void);

/**
 * @brief Publishes a telemetry sample for transmission.
 *
 * Copies the supplied sample into the telemetry queue. The queue has
 * a length of one, so publishing a new sample overwrites any sample
 * that has not yet been transmitted.
 *
 * @param[in] data Pointer to the telemetry sample.
 *
 * @return
 * - ESP_OK: Sample queued successfully.
 * - ESP_ERR_INVALID_ARG: NULL sample pointer.
 * - ESP_ERR_INVALID_STATE: Telemetry queue is not initialized.
 * - ESP_FAIL: Queue operation failed.
 *
 * @note This function does not transmit the packet immediately.
 *       Transmission is handled asynchronously by the telemetry task.
 * @note Intermediate samples may be discarded intentionally so that
 *       the receiver gets the most recent available telemetry.
 */
esp_err_t telemetry_publish(const telemetry_data_t *data);

/**
 * @brief Stops the telemetry task and releases its resources.
 *
 * Deletes the telemetry task, closes the UDP socket, and deletes
 * the telemetry queue.
 *
 * @note This function is safe to call when the service is partially
 *       initialized or already deinitialized.
 */
void telemetry_deinit(void);

#endif