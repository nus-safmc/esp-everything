idf_component_register(
    SRCS
        "main.c"
        "mavlink_task.c"
        "tof_task.c"
        "vfh.c"
        "nav_task.c"
        "at_detect.c"
        "breadcrumb.c"
        "wifi_task.c"
        "odom.c"
    INCLUDE_DIRS "."
    REQUIRES
        mavlink
        vl53l5cx
        driver
        freertos
        esp_timer
        esp-apriltag
        esp32-camera
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        lwip
)

# Suppress -Waddress-of-packed-member from MAVLink and VL53L5CX generated headers
target_compile_options(__idf_main PRIVATE -Wno-address-of-packed-member)