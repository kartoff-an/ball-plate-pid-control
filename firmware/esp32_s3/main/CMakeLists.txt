idf_component_register(
    SRCS
        "app_main.c"
        "ball_receiver.c"
        "pca9685.c"
        "pid_controller.c"
        "wifi_ap.c"
        "telemetry.c"
        "calibration_commands.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_common
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        esp_driver_i2c
        nvs_flash
        freertos

)