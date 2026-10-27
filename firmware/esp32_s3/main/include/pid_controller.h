#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include "esp_err.h"


typedef enum {
    PID_AXIS_X,
    PID_AXIS_Y,
    PID_AXIS_BOTH
} pid_axis_id_t;
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

esp_err_t pid_controller_init(void);

esp_err_t pid_controller_set_gains(pid_axis_id_t axis, const pid_gains_t *gains);

esp_err_t pid_controller_get_gains(pid_axis_id_t axis, pid_gains_t * gains);

esp_err_t pid_controller_set_target(float x_mm, float y_mm);
esp_err_t pid_controller_start(void);
esp_err_t pid_controller_stop(void);
esp_err_t pid_controller_reset(void);
bool pid_controller_is_enabled(void);

#endif
