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

#endif /* VISION_TUNER_H */
