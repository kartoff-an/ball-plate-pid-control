idf_component_register(
    SRCS "ball_now.c" "app_main.c"
    INCLUDE_DIRS "include"
    REQUIRES
        camera_capture
        streaming
        wifi_station
        esp_timer
        esp_common
        ball_detection
)