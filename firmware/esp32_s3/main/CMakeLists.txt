idf_component_register(
    SRCS "ball_receiver.c" "app_main.c"
    INCLUDE_DIRS "include"
    REQUIRES
        wifi_ap
        esp_common
        esp_timer
)