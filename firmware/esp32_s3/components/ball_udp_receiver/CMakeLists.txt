idf_component_register(
    SRCS "ball_udp_receiver.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_timer
        lwip
)
