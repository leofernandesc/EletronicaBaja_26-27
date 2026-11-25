idf_component_register(
    SRCS "speed_sensor.c" "speed_measurement.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_timer
    PRIV_REQUIRES heap freertos
)
