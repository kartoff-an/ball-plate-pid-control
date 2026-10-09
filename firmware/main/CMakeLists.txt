idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES control servo communication state kinematics
)