idf_component_register(SRCS "main.c" "../user_lib/app.cpp" "../user_lib/foc_bench.cpp"
    "../user_lib/hw/sensor.cpp" "../user_lib/sys_time.cpp"
    INCLUDE_DIRS "." "../user_lib" "../user_lib/hw"
    LDFRAGMENTS "motor.lf"
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_mcpwm esp_timer esp_adc)

# Same per-FOC optimization used by v18; other modules keep debug settings.
set_source_files_properties("../user_lib/foc_bench.cpp" "../user_lib/hw/motor.cpp" PROPERTIES COMPILE_OPTIONS "-O2")
