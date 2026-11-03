#ifndef CALIBRATION_COMMANDS_H
#define CALIBRATION_COMMANDS_H

#include "esp_err.h"

/**
 * @brief Initialize the runtime calibration command listener.
 *
 * Creates a FreeRTOS task that opens a UDP socket, binds it to the
 * calibration port, and processes incoming servo and PID commands.
 *
 * @return
 *      - ESP_OK: Calibration task created successfully.
 *      - ESP_FAIL: FreeRTOS task creation failed.
 *
 * @note ESP_OK confirms that the task was created. It does not guarantee
 *       that socket creation or UDP port binding succeeded.
 * @note This function should normally be called once during application
 *       initialization.
 */
esp_err_t calibration_commands_init(void);

#endif