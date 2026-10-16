idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        camera_capture
        communication
        streaming
        wifi_station
        esp_timer
        esp_common
        ball_detection
)