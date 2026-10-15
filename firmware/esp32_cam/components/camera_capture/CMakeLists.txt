idf_component_register(
    SRCS "camera_capture.c"
    INCLUDE_DIRS "include"
    REQUIRES esp32-camera esp_timer esp_common
)