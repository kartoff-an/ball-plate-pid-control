/**
 * @file pca9685.h
 * @brief PCA9685 PWM driver interface for servo control.
 *
 * Provides initialization and PWM pulse-width configuration functions
 * for the PCA9685 16-channel, 12-bit PWM controller.
 *
 * The driver communicates with the PCA9685 over I2C and configures
 * its PWM frequency to a nominal 50 Hz for servo operation.
 *
 * Servo positions are controlled by specifying pulse widths in
 * microseconds.
 *
 * Typical servo pulse widths:
 * - 1000 us: One end of the servo's operating range.
 * - 1500 us: Typical center position.
 * - 2000 us: Opposite end of the servo's operating range.
 *
 * Actual servo positions depend on the servo model and its calibration.
 */

#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the PCA9685 PWM controller.
 *
 * Configures the I2C master bus, registers the PCA9685 device,
 * and initializes its internal PWM frequency configuration.
 *
 * The driver configures:
 * - I2C address: 0x40.
 * - I2C clock frequency: 100 kHz.
 * - PWM frequency: Nominally 50 Hz.
 * - PWM resolution: 12-bit (4096 ticks per period).
 * - Auto-increment register addressing.
 *
 * @param[in] sda_gpio GPIO number connected to the I2C SDA line.
 * @param[in] scl_gpio GPIO number connected to the I2C SCL line.
 *
 * @return
 *      - ESP_OK: PCA9685 initialized successfully.
 *      - Other ESP-IDF error code: I2C bus initialization, device
 *        registration, or PCA9685 configuration failed.
 *
 * @note The I2C bus and device handles are maintained internally
 *       by the driver.
 * @note This function should normally be called once before
 *       attempting to configure PWM outputs.
 * @note External pull-up resistors on SDA and SCL may be required.
 */
esp_err_t pca9685_init(int sda_gpio, int scl_gpio);

/**
 * @brief Set the PWM pulse width of a PCA9685 channel.
 *
 * Converts the requested pulse width in microseconds into PCA9685
 * PWM ticks using the configured nominal 50 Hz frequency, then
 * writes the corresponding LED channel registers over I2C.
 *
 * Supported channel range: 0 to 15.
 *
 * Supported pulse-width range: 500 to 2500 microseconds.
 *
 * @param[in] channel PCA9685 output channel.
 * @param[in] pulse_us Desired PWM pulse width in microseconds.
 *
 * @return
 *      - ESP_OK: PWM registers written successfully.
 *      - ESP_ERR_INVALID_ARG: Channel or pulse width is outside
 *        the supported range.
 *      - Other ESP-IDF error code: I2C transmission failed.
 *
 * @note The pulse width determines the servo command position.
 *       It does not guarantee a particular mechanical angle.
 * @note The requested pulse width is converted using the nominal
 *       configured PWM frequency. Actual output timing also depends
 *       on the PCA9685 oscillator frequency.
 * @note This function assumes pca9685_init() has completed successfully.
 */
esp_err_t pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us);

#endif /* PCA9685_H*/