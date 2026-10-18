idf_component_register(SRCS "main.c" "../user_lib/app.cpp" "../user_lib/test.cpp"
    "../user_lib/hw/sensor.cpp" "../user_lib/hw/bat.cpp" "../user_lib/sys_time.cpp"
    INCLUDE_DIRS "." "../user_lib" "../user_lib/hw"
    PRIV_REQUIRES esp_driver_i2c esp_timer esp_adc)
