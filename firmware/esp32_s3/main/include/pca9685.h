#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t pca9685_init(int sda_gpio, int scl_gpio);
esp_err_t pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us);

#endif
