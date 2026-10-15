idf_component_register(
    SRCS "udp_preview.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_timer
        esp_wifi
        lwip
        freertos
)
