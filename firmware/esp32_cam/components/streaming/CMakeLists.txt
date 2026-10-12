idf_component_register(
    SRCS "wifi_stream.c"
    INCLUDE_DIRS "include"
    REQUIRES
        camera_capture
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        heap
        nvs_flash
        esp_common
)