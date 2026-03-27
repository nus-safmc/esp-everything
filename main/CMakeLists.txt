idf_component_register(
    SRCS
        "main.c"
        "mavlink_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        mavlink
        driver
        freertos
        esp_timer
)