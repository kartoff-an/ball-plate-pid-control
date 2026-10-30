/**
 * @file camera.h
 * @brief Camera initialization and capture task interface.
 *
 * Provides functions to initialize the camera hardware and start
 * the task responsible for frame capture, ball detection, measurement
 * transmission, and UDP streaming.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "esp_err.h"

/**
 * @brief Initialize the camera hardware.
 *
 * Configures the ESP32-CAM camera interface and initializes
 * the camera driver using the settings defined in camera.c.
 *
 * @return
 * - ESP_OK: Camera initialized successfully.
 * - Other ESP-IDF error code: Camera initialization failed.
 *
 * @note Must be called before camera_start().
 */
esp_err_t camera_init(void);

/**
 * @brief Start the camera task.
 *
 * Creates a FreeRTOS task pinned to the configured CPU core.
 * The task continuously captures frames, performs ball detection,
 * sends measurements to the ESP32-S3, and submits frames for UDP
 * streaming.
 *
 * @return
 * - ESP_OK: Camera task created successfully.
 * - ESP_FAIL: Failed to create the camera task.
 *
 * @note Call camera_init() before starting the task.
 */
esp_err_t camera_start(void);

#endif /* CAMERA_H*/