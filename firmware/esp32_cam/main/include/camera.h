#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include "esp_err.h"

esp_err_t camera_init(void);

esp_err_t camera_start(void);

#endif