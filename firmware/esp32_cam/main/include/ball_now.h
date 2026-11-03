/**
 * @file ball_now.h
 * @brief ESP-NOW communication interface for ball measurements.
 *
 * @details
 * Provides functions for initializing ESP-NOW communication and sending
 * ball detection measurements from the ESP32-CAM to the ESP32-S3.
 */

#ifndef BALL_NOW_H
#define BALL_NOW_H

#include <stdint.h>
#include "esp_err.h"
#include "ball_detection.h"

/**
 * @brief Initialize ESP-NOW communication.
 *
 * Registers the ESP32-S3 as an unencrypted peer using the station interface.
 *
 * @pre Wi-Fi must be initialized before calling this function.
 *
 * @param peer_mac MAC address of the ESP32-S3 receiver.
 *
 * @return
 * - ESP_OK: Initialization successful or already initialized.
 * - ESP_ERR_INVALID_ARG: Invalid peer MAC address.
 * - Other ESP-IDF error codes: ESP-NOW initialization or peer registration failed.
 */
esp_err_t ball_now_init(const uint8_t peer_mac[6]);

/**
 * @brief Send a ball detection measurement to the ESP32-S3.
 *
 * Packages the detection result into a ball_measurement_t message.
 * Detected coordinates are adjusted by the configured camera crop offsets
 * to represent positions in the original camera frame.
 *
 * When no ball is detected, coordinates and area are transmitted as zero.
 *
 * @param sequence Frame sequence number associated with the detection.
 * @param detection Pointer to the ball detection result.
 *
 * @return
 * - ESP_OK: Transmission request accepted.
 * - ESP_ERR_INVALID_STATE: Module not initialized or detection is NULL.
 * - Other ESP-IDF error codes: Transmission request failed.
 *
 * @note ESP_OK indicates that the send request was accepted, not that the
 *       receiver successfully received the message.
 */
esp_err_t ball_now_send_detection(
    uint32_t sequence,
    const ball_detection_result_t *detection
);

#endif /* BALL_NOW_H */
