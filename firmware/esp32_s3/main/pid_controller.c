/**
 * @file pid_controller.c
 * @brief PID control implementation for the ball-and-plate system.
 *
 * Implements periodic ball-position feedback control using measurements
 * received from the camera module.
 *
 * The controller:
 * - Retrieves and validates incoming ball measurements.
 * - Converts pixel coordinates into plate-relative millimeter coordinates.
 * - Calculates position errors against the configured target.
 * - Computes independent PID outputs for the X and Y axes.
 * - Converts PID outputs into servo pulse-width commands.
 * - Publishes measurement, error, and controller telemetry.
 *
 * The control task runs periodically, while PID calculations are performed
 * only when a new valid measurement is received.
 *
 * @note Physical servo output is controlled by CONTROL_OUTPUT_ENABLED.
 * @note Servo commands are issued through the PCA9685 driver.
 */

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#include "pid_controller.h"
#include "ball_receiver.h"
#include "ball_measurement.h"
#include "pca9685.h"
#include "telemetry.h"
#include "kalman_filter.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TELEMETRY_DEBUG             1

#define CONTROL_PERIOD_MS           10
#define MAX_MEASUREMENT_AGE_MS      150

#define FRAME_WIDTH                 160
#define FRAME_HEIGHT                120

#define PLATE_SIZE_MM               220.0f
#define PIXELS_PER_PLATE_AXIS       120.0f
#define HALF_PLATE_SIZE             PLATE_SIZE_MM / 2.0
#define PIXELS_TO_MM                (PLATE_SIZE_MM / PIXELS_PER_PLATE_AXIS)

#define PLATE_CENTER_X_PX           80.0f
#define PLATE_CENTER_Y_PX           60.0f

#define TARGET_X_MM                 0.0f
#define TARGET_Y_MM                 0.0f

#define SERVO_X_CHANNEL             1
#define SERVO_Y_CHANNEL             0

#define SERVO_NEUTRAL_X_US          1450
#define SERVO_NEUTRAL_Y_US          1500
#define SERVO_MIN_US                1000
#define SERVO_MAX_US                2000
#define SERVO_MAX_CORRECTION_US     120.0f
#define SERVO_MAX_SLEW_US_PER_SEC   600.0f

#define PID_KP_MAX                  10.0f
#define PID_KI_MAX                  10.0f
#define PID_KD_MAX                  10.0f

#define KP_X                        0.5f
#define KI_X                        0.0f
#define KD_X                        0.0f

#define KP_Y                        0.5f
#define KI_Y                        0.0f
#define KD_Y                        0.0f

#define SERVO_X_SIGN                -1.0f
#define SERVO_Y_SIGN                -1.0f

#define CONTROL_OUTPUT_ENABLED      1

#define CONTROL_TASK_STACK_SIZE    4096
#define CONTROL_TASK_PRIORITY      6

static const char *TAG = "PID";

static portMUX_TYPE s_pid_config_mux = portMUX_INITIALIZER_UNLOCKED;

static float s_target_x_mm = 0.0f;
static float s_target_y_mm = 0.0f;

static kalman_filter_t s_ball_filter;

static bool s_controller_enabled = false;
static bool s_reset_requested = false;
static bool s_neutral_requested = false;

static uint16_t s_servo_neutral_x_us = SERVO_NEUTRAL_X_US;
static uint16_t s_servo_neutral_y_us = SERVO_NEUTRAL_Y_US;


typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float output_limit;
} pid_axis_t;

static pid_axis_t s_pid_x = {
    .kp = KP_X,
    .ki = KI_X,
    .kd = KD_X,
    .output_limit = SERVO_MAX_CORRECTION_US,
};

static pid_axis_t s_pid_y = {
    .kp = KP_Y,
    .ki = KI_Y,
    .kd = KD_Y,
    .output_limit = SERVO_MAX_CORRECTION_US
};

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float pixels_to_mm(float pixel, float center_pixel)
{
    return (pixel - center_pixel) * PIXELS_TO_MM;
}

static bool gains_are_valid(const pid_gains_t *gains)
{
    return gains != NULL && isfinite(gains->kp) && isfinite(gains->ki) && isfinite(gains->kd) &&
        gains->kp >= 0.0f && gains->kp <= PID_KP_MAX &&
        gains->ki >= 0.0f && gains->ki <= PID_KI_MAX &&
        gains->kd >= 0.0f && gains->kd <= PID_KD_MAX;
}

esp_err_t pid_controller_set_gains(pid_axis_id_t axis, const pid_gains_t *gains)
{
    if ((axis != PID_AXIS_X && axis != PID_AXIS_Y && axis != PID_AXIS_BOTH) || !gains_are_valid(gains)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_pid_config_mux);

    if (axis == PID_AXIS_X || axis == PID_AXIS_BOTH) {
        s_pid_x.kp = gains->kp;
        s_pid_x.ki = gains->ki;
        s_pid_x.kd = gains->kd;
    }

    if (axis == PID_AXIS_Y || axis == PID_AXIS_BOTH) {
        s_pid_y.kp = gains->kp;
        s_pid_y.ki = gains->ki;
        s_pid_y.kd = gains->kd;
    }

    portEXIT_CRITICAL(&s_pid_config_mux);

    ESP_LOGI(TAG, "Gains updated: axis=%d Kp=%.3f Ki=%.3f Kd=%.3f",
            (int)axis,
            (double)gains->kp,
            (double)gains->ki,
            (double)gains->kd
    );

    return ESP_OK;
}

esp_err_t pid_controller_get_gains(pid_axis_id_t axis, pid_gains_t *gains)
{
    if (gains == NULL || (axis != PID_AXIS_X && axis != PID_AXIS_Y)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_pid_config_mux);

    const pid_axis_t *pid = (axis == PID_AXIS_X) ? &s_pid_x : &s_pid_y;

    gains->kp = pid->kp;
    gains->ki = pid->ki;
    gains->kd = pid->kd;

    portEXIT_CRITICAL(&s_pid_config_mux);

    return ESP_OK;
}

static float pid_update(pid_axis_t *pid, float error, float velocity_mm_s, float dt_seconds)
{
    if (dt_seconds <= 0.0f || dt_seconds > 0.25f) {
        pid->integral = 0.0f;
        return 0.0f;
    }

    float kp;
    float ki;
    float kd;

    portENTER_CRITICAL(&s_pid_config_mux);
    kp = pid->kp;
    ki = pid->ki;
    kd = pid->kd;
    portEXIT_CRITICAL(&s_pid_config_mux);

    const float derivative = -velocity_mm_s;
    const float proposed_integral = clampf(pid->integral + error * dt_seconds, -100.0f, 100.0f);
    float candidate_output = kp * error + ki * proposed_integral + kd * derivative;

    bool saturated_high = candidate_output > pid->output_limit;
    bool saturated_low = candidate_output < -pid->output_limit;
    bool drives_further_into_saturation = (saturated_high && error > 0.0f) || (saturated_low && error < 0.0f);

    // prevent the integral from winding up against output limits
    if (!drives_further_into_saturation) {
        pid->integral = proposed_integral;
    }

    float output = kp * error + ki * pid->integral + kd * derivative;

    return clampf(output, -pid->output_limit, pid->output_limit);
}

static float limit_slew(float target, float current, float dt)
{
    float max_change = SERVO_MAX_SLEW_US_PER_SEC * dt;
    return current + clampf(target - current, -max_change, max_change);
}

static esp_err_t set_neutral(uint16_t x_us, uint16_t y_us)
{
    esp_err_t x_error = pca9685_set_pulse_us(SERVO_X_CHANNEL, x_us);
    esp_err_t y_error = pca9685_set_pulse_us(SERVO_Y_CHANNEL, y_us);

    return x_error != ESP_OK ? x_error : y_error;
}

static uint32_t s_telemetry_sequence = 0;

static void control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    bool have_sequence = false;
    uint32_t previous_sequence = 0;
    int64_t previous_update_us = 0;

    float command_x_us = 0.0f;
    float command_y_us = 0.0f;

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        bool controller_enabled;
        bool reset_requested;
        bool neutral_requested;
        float target_x_mm;
        float target_y_mm;
        uint16_t neutral_x_us;
        uint16_t neutral_y_us;

        portENTER_CRITICAL(&s_pid_config_mux);

        controller_enabled = s_controller_enabled;
        reset_requested = s_reset_requested;
        neutral_requested = s_neutral_requested;

        s_reset_requested = false;
        s_neutral_requested = false;

        target_x_mm = s_target_x_mm;
        target_y_mm = s_target_y_mm;

        neutral_x_us = s_servo_neutral_x_us;
        neutral_y_us = s_servo_neutral_y_us;

        portEXIT_CRITICAL(&s_pid_config_mux);

        if (reset_requested) {
            kalman_filter_reset(&s_ball_filter);

            s_pid_x.integral = 0.0f;
            s_pid_y.integral = 0.0f;

            have_sequence = false;
            previous_update_us = 0;

            command_x_us = 0.0f;
            command_y_us = 0.0f;
        }

        if (neutral_requested) {
            set_neutral(neutral_x_us, neutral_y_us);
        }

        ball_measurement_t measurement;
        int64_t age_ms = INT64_MAX;

        const bool available = ball_receiver_get_latest(&measurement, &age_ms);
        const bool valid =
            available &&
            age_ms >= 0 &&
            age_ms <= MAX_MEASUREMENT_AGE_MS &&
            measurement.detected == 1 &&
            measurement.center_x < FRAME_WIDTH &&
            measurement.center_y < FRAME_HEIGHT;

        if (!valid) {
            command_x_us = 0.0f;
            command_y_us = 0.0f;

            kalman_filter_reset(&s_ball_filter);

            s_pid_x.integral = 0.0f;
            s_pid_y.integral = 0.0f;
            have_sequence = false;

            if (CONTROL_OUTPUT_ENABLED) {
                esp_err_t error = set_neutral(neutral_x_us, neutral_y_us);
                if (error != ESP_OK) {
                    ESP_LOGE(TAG, "Neutral command failed: %s", esp_err_to_name(error));
                }
            }

            continue;
        }

        const int64_t now_us = esp_timer_get_time();

        if (!have_sequence || measurement.sequence != previous_sequence) {
            float dt_seconds = 0.0f;

            if (have_sequence && previous_update_us > 0) {
                dt_seconds = (float)(now_us - previous_update_us) / 1000000.0f;
            }

            const float measured_x_mm = pixels_to_mm((float)measurement.center_x, PLATE_CENTER_X_PX);
            const float measured_y_mm = pixels_to_mm((float)measurement.center_y, PLATE_CENTER_Y_PX);

            if (!kalman_filter_update(&s_ball_filter, measured_x_mm, measured_y_mm, dt_seconds)) {
                continue;
            }

            float ball_x_mm;
            float ball_y_mm;
            float ball_vx_mm_s;
            float ball_vy_mm_s;

            kalman_filter_get_state(
                &s_ball_filter,
                &ball_x_mm,
                &ball_y_mm,
                &ball_vx_mm_s,
                &ball_vy_mm_s
            );

            const float error_x = target_x_mm - ball_x_mm;
            const float error_y = target_y_mm - ball_y_mm;

            if (!controller_enabled) {
                s_pid_x.integral = 0.0f;
                s_pid_y.integral = 0.0f;

                command_x_us = 0.0f;
                command_y_us = 0.0f;
            }
            else if (!have_sequence) {
                s_pid_x.integral = 0.0f;
                s_pid_y.integral = 0.0f;
                command_x_us = 0.0f;
                command_y_us = 0.0f;
            }
            else {
                command_x_us = limit_slew(
                    pid_update(&s_pid_x, error_x, ball_vx_mm_s, dt_seconds),
                    command_x_us,
                    dt_seconds
                );
                command_y_us = limit_slew(
                    pid_update(&s_pid_y, error_y, ball_vy_mm_s, dt_seconds),
                    command_y_us,
                    dt_seconds
                );
            }

            previous_sequence = measurement.sequence;
            previous_update_us = now_us;
            have_sequence = true;

            const int x_pulse = (int)(neutral_x_us + SERVO_X_SIGN * command_x_us);
            const int y_pulse = (int)(neutral_y_us + SERVO_Y_SIGN * command_y_us);
            const uint16_t safe_x_pulse = (uint16_t)
                (x_pulse < SERVO_MIN_US ? SERVO_MIN_US :
                 x_pulse > SERVO_MAX_US ? SERVO_MAX_US : x_pulse);
            const uint16_t safe_y_pulse = (uint16_t)
                (y_pulse < SERVO_MIN_US ? SERVO_MIN_US :
                 y_pulse > SERVO_MAX_US ? SERVO_MAX_US : y_pulse);

            telemetry_data_t telemetry = {
                .sequence = s_telemetry_sequence++,
                .ball_detected = measurement.detected == 1,
                .ball_x = measurement.center_x,
                .ball_y = measurement.center_y,
                .measurement_age_ms = age_ms,
                .error_x = error_x,
                .error_y = error_y,
                .output_x_us = command_x_us,
                .output_y_us = command_y_us,
                .servo_x_us = safe_x_pulse,
                .servo_y_us = safe_y_pulse,
                .controller_enabled = controller_enabled && CONTROL_OUTPUT_ENABLED
            };
            esp_err_t telemetry_error = telemetry_publish(&telemetry);

#if TELEMETRY_DEBUG
            if (telemetry_error != ESP_OK) {
                ESP_LOGW(TAG, "Telemetry publish failed: %s", esp_err_to_name(telemetry_error));
            }
#endif

            if (CONTROL_OUTPUT_ENABLED) {
                esp_err_t x_error = pca9685_set_pulse_us(SERVO_X_CHANNEL, safe_x_pulse);
                esp_err_t y_error = pca9685_set_pulse_us(SERVO_Y_CHANNEL, safe_y_pulse);

                if (x_error != ESP_OK || y_error != ESP_OK) {
                    ESP_LOGE(TAG, "Servo update failed");
                    (void)set_neutral(neutral_x_us, neutral_y_us);
                }
            }
        }
    }
}

esp_err_t pid_controller_init(void)
{
    kalman_filter_init(&s_ball_filter);

    BaseType_t result = xTaskCreate(
        control_task,
        "pid_control",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        NULL
    );
    return result == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t pid_controller_set_target(float x_mm, float y_mm)
{
    if (
        !isfinite(x_mm) || !isfinite(y_mm) ||
        x_mm < -HALF_PLATE_SIZE || x_mm > HALF_PLATE_SIZE ||
        y_mm < -HALF_PLATE_SIZE || y_mm > HALF_PLATE_SIZE
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_pid_config_mux);
    s_target_x_mm = x_mm;
    s_target_y_mm = y_mm;
    portEXIT_CRITICAL(&s_pid_config_mux);

    return ESP_OK;
}

esp_err_t pid_controller_start(void)
{
    portENTER_CRITICAL(&s_pid_config_mux);
    s_reset_requested = true;
    s_neutral_requested = false;
    s_controller_enabled = true;
    portEXIT_CRITICAL(&s_pid_config_mux);

    return ESP_OK;
}

esp_err_t pid_controller_reset(void)
{
    portENTER_CRITICAL(&s_pid_config_mux);
    s_reset_requested = true;
    portEXIT_CRITICAL(&s_pid_config_mux);
    return ESP_OK;
}

esp_err_t pid_controller_stop(void)
{
    portENTER_CRITICAL(&s_pid_config_mux);
    s_controller_enabled = false;
    s_neutral_requested = true;
    portEXIT_CRITICAL(&s_pid_config_mux);

    return ESP_OK;
}

esp_err_t pid_controller_set_servo_neutral(uint16_t x_us, uint16_t y_us)
{
    if (x_us < SERVO_MIN_US || x_us > SERVO_MAX_US || y_us < SERVO_MIN_US || y_us > SERVO_MAX_US) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_pid_config_mux);

    s_servo_neutral_x_us = x_us;
    s_servo_neutral_y_us = y_us;

    s_reset_requested = true;
    s_neutral_requested = true;

    portEXIT_CRITICAL(&s_pid_config_mux);

    ESP_LOGI(TAG, "Servo neutral updated: X=%u us Y=%u us", x_us, y_us);

    return ESP_OK;
}