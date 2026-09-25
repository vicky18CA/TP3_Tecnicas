idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        freertos
        esp_driver_gpio
        esp_adc
        esp_driver_i2c
        esp_driver_uart
        esp_timer
        log
)