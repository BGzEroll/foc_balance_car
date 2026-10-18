idf_component_register(SRCS "main.c" "../user_lib/app.cpp" "../user_lib/test.cpp" "../user_lib/foc_current.cpp"
    "../user_lib/hw/sensor.cpp" "../user_lib/hw/bat.cpp" "../user_lib/sys_time.cpp"
    INCLUDE_DIRS "." "../user_lib" "../user_lib/hw"
    LDFRAGMENTS "foc_current.lf"
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_timer esp_adc esp_driver_mcpwm)

# 电流环单独使用速度优化，其余模块维持现有调试配置。
set_source_files_properties("../user_lib/foc_current.cpp" PROPERTIES COMPILE_OPTIONS "-O2")
