idf_component_register(
    SRCS
        "main.c"
        "mavlink_task.c"
        "tof_task.c"
        "vfh.c"
    INCLUDE_DIRS "."
    REQUIRES
        mavlink
        vl53l5cx
        driver
        freertos
        esp_timer
)

# Suppress -Waddress-of-packed-member from MAVLink and VL53L5CX generated headers
target_compile_options(__idf_main PRIVATE -Wno-address-of-packed-member)