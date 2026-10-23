#include "pca9685.h"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCA9685_ADDR        0x40
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_OSC_HZ      25000000UL
#define PCA9685_PWM_HZ      50
#define PCA9685_TICKS       4096

static const char *TAG = "PCA9685";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(s_device, data, sizeof(data), 100);
}

esp_err_t pca9685_init(int sda_gpio, int scl_gpio)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&bus_config, &s_bus),
        TAG, "I2C bus initialization failed");

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_ADDR,
        .scl_speed_hz = 100000
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &device_config, &s_device),
        TAG, "PCA9685 device registration failed");

    // sleep mode before changing the PWM prescaler
    ESP_RETURN_ON_ERROR(
        write_register(PCA9685_MODE1, 0x10),
        TAG, "Could not enter sleep mode"
    );

    // prescale = round(25 MHz / (4096 * 50 Hz)) - 1
    const uint8_t prescale =
        (uint8_t)((PCA9685_OSC_HZ + (PCA9685_PWM_HZ * PCA9685_TICKS) / 2) /
        (PCA9685_PWM_HZ * PCA9685_TICKS) - 1);

    ESP_RETURN_ON_ERROR(
        write_register(PCA9685_PRESCALE, prescale),
        TAG, "Could not set PWM frequency"
    );

    ESP_RETURN_ON_ERROR(
        write_register(PCA9685_MODE1, 0x20),
        TAG, "Could not wake PCA9685"
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    // auto-increment register addressing
    ESP_RETURN_ON_ERROR(
        write_register(PCA9685_MODE1, 0xA0),
        TAG, "Could not enable auto-increment"
    );

    ESP_LOGI(TAG, "PCA9685 initialized at nominal 50 Hz");
    return ESP_OK;
}

esp_err_t pca9685_set_pulse_us(uint8_t channel, uint16_t pulse_us) {
    if (channel > 15 || pulse_us < 500 || pulse_us > 2500) {
        return ESP_ERR_INVALID_ARG;
    }

    // at 50 Hz, one PWM period is 20,000 us.
    const uint32_t ticks = ((uint32_t)pulse_us * PCA9685_PWM_HZ * PCA9685_TICKS + 500000UL) / 1000000UL;
    const uint8_t reg = PCA9685_LED0_ON_L + (uint8_t)(4 * channel);
    uint8_t data[5] = {reg, 0, 0, (uint8_t)(ticks & 0xFF), (uint8_t)((ticks >> 8) & 0x0F)};

    return i2c_master_transmit(s_device, data, sizeof(data), 100);
}

