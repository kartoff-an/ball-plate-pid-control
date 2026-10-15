idf_component_register(
    SRCS "wifi_stream.c"
    INCLUDE_DIRS "include"
    REQUIRES
        heap
        esp_common
        lwip
        freertos
        log
)