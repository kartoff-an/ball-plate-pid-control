/**
 * @file vision_tuner.h
 * @brief Runtime configuration interface for ball detection.
 *
 * Provides the interface for starting a UDP listener that receives
 * vision configuration updates from the PC and applies them to the
 * ball detection module without requiring firmware reflashing.
 */

#ifndef VISION_TUNER_H
#define VISION_TUNER_H

#include "esp_err.h"
#include "ball_detection.h"

#define VISION_TUNER_UDP_PORT 9006

/**
 * @brief Start the runtime vision tuning task.
 *
 * Creates a FreeRTOS task that listens for UDP configuration packets
 * from the PC. Valid configuration values are applied to the ball
 * detection module.
 *
 * @return
 * - ESP_OK: Task created successfully.
 * - ESP_ERR_NO_MEM: Insufficient memory to create the task.
 *
 * @note The UDP listener runs independently after successful startup.
 */
esp_err_t vision_tuner_start(void);

/**
 * @brief Save the ball detection configuration to NVS flash.
 *
 * Stores the supplied configuration as a blob in the "vision" NVS
 * namespace under the "config" key. The data is committed to flash
 * and persists across device reboots.
 *
 * @param config Pointer to the ball detection configuration to save.
 *
 * @return
 * - ESP_OK: Configuration saved successfully.
 * - ESP_ERR_INVALID_ARG: Configuration pointer is NULL.
 * - Other ESP error codes: NVS operation failed.
 *
 * @note This function does not apply the configuration to the
 *       ball detection module. Use ball_detection_set_config()
 *       separately when runtime application is required.
 */
esp_err_t vision_tuner_save(const ball_detection_config_t *config);

#endif /* VISION_TUNER_H */
