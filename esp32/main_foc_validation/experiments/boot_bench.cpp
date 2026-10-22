// Test-only translation unit: include the real motor implementation so the
// bench can inspect private diagnostics without extending the production API.
#include "hw/motor.cpp"
#include <stdio.h>

void foc_bench_init()
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("BENCH original_motor baseline auto-alignment bounded=8s\n");
    if(!motor::init()) { puts("INIT_FAILED"); return; }
    const int64_t end = esp_timer_get_time() + 8000000;
    while(esp_timer_get_time() < end)
    {
        motor::foc_info s{};
        motor::get_info(s);
        printf("BOOT %lld stage=%u motor=%u ready=%u fault=%u bus=%.3f run=%u request=%u a=%.6f b=%.6f id=%.5f,%.5f iq=%.5f,%.5f\n",
            esp_timer_get_time(), unsigned(motor::calibration.state), motor::calibration.motor,
            s.ready, unsigned(s.fault.code), s.bus_voltage_V, pwm::is_running(), pwm::is_requested(),
            s.current[0].mechanical_angle_rad, s.current[1].mechanical_angle_rad,
            s.current[0].id_A, s.current[1].id_A, s.current[0].iq_A, s.current[1].iq_A);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    pwm::stop();
    __atomic_store_n(&motor::ready, 0U, __ATOMIC_RELEASE);
    printf("STOPPED run=%u request=%u\n", pwm::is_running(), pwm::is_requested());
}
