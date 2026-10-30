/**
 * @file config_commands.h
 * @brief UDP interface for runtime PID and target configuration.
 *
 * Provides a JSON-based command interface for configuring the ball-and-plate
 * controller without reflashing the ESP32-S3 firmware.
 *
 * The configuration service listens for UDP messages on port 9002.
 *
 * Supported message types:
 *
 * - target: Updates the desired ball position in millimeters.
 * - pid_config: Updates the PID gains for both control axes.
 * - control: Starts, stops, or resets the PID controller.
 *
 * Expected message formats:
 *
 * Target:
 * @code
 * {
 *     "type": "target",
 *     "x": 0.0,
 *     "y": 0.0
 * }
 * @endcode
 *
 * PID configuration:
 * @code
 * {
 *     "type": "pid_config",
 *     "axis_x": {
 *         "kp": 0.5,
 *         "ki": 0.0,
 *         "kd": 0.0
 *     },
 *     "axis_y": {
 *         "kp": 0.5,
 *         "ki": 0.0,
 *         "kd": 0.0
 *     }
 * }
 * @endcode
 *
 * Controller commands:
 * @code
 * {
 *     "type": "control",
 *     "command": "start"
 * }
 * @endcode
 *
 * Supported control commands:
 * - start
 * - stop
 * - reset_pid
 *
 * Each received message is acknowledged with a JSON response containing
 * an operation status and descriptive message.
 */

#ifndef CONFIG_COMMANDS_H
#define CONFIG_COMMANDS_H

#include "esp_err.h"

/**
 * @brief Initialize the UDP configuration command service.
 *
 * Creates a FreeRTOS task that opens a UDP socket, binds it to the
 * configuration command port, and processes incoming JSON messages.
 *
 * The task handles target updates, PID gain configuration, and
 * controller start, stop, and reset commands.
 *
 * @return
 *      - ESP_OK: Configuration task created successfully.
 *      - ESP_FAIL: FreeRTOS task creation failed.
 *
 * @note ESP_OK confirms that the task was created. It does not guarantee
 *       that socket creation or UDP port binding succeeded.
 * @note This function should normally be called once during application
 *       initialization.
 */
esp_err_t config_commands_init(void);

#endif /* CONFIG_COMMANDS_H*/