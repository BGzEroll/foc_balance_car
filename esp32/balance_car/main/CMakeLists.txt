idf_component_register(SRCS "main.c" "../user_lib/app.cpp" "../user_lib/test.cpp" "../user_lib/balance.cpp"
    "../user_lib/hw/sensor.cpp" "../user_lib/hw/motor.cpp" "../user_lib/sys_time.cpp"
    INCLUDE_DIRS "." "../user_lib" "../user_lib/hw"
    LDFRAGMENTS "../user_lib/hw/motor.lf"
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_driver_i2c esp_driver_mcpwm esp_timer esp_adc)
