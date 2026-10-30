
#ifndef VISION_TUNER_H
#define VISION_TUNER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_TUNER_UDP_PORT 9006

/**
 * @brief Start the runtime vision tuning UDP task.
 *
 * Listens for configuration packets from the PC and applies valid
 * settings to the ball detection module.
 *
 * @return ESP_OK if the task was created successfully,
 *         ESP_ERR_NO_MEM if task creation failed.
 */
esp_err_t vision_tuner_start(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_TUNER_H */
