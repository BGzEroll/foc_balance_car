namespace math_test
{
    static volatile float sink;
    static __attribute__((noinline)) float library(float a)
    {
        a = fmodf(a, foc_math::TWO_PI);
        if(a < 0) a += foc_math::TWO_PI;
        return sinf(a) + cosf(a);
    }
    static __attribute__((noinline)) float normalized(float a)
    {
        a = foc_math::wrap(a);
        return sinf(a) + cosf(a);
    }
    static __attribute__((noinline)) float combined(float a)
    {
        float s, c; sincosf(foc_math::wrap(a), &s, &c); return s + c;
    }
    static __attribute__((noinline)) float lookup(float a)
    {
        float s, c; foc_math::sincos(foc_math::wrap(a), s, c); return s + c;
    }
}

void math_benchmark()
{
    foc_math::init();
    const char *names[] = {"fmod_sin_cos", "fast_wrap_sin_cos", "fast_wrap_sincosf", "fast_wrap_lut4096"};
    float (*functions[])(float) = {math_test::library, math_test::normalized, math_test::combined, math_test::lookup};
    for(unsigned k = 0; k < 4; ++k)
    {
        uint64_t total = 0;
        uint32_t smallest = UINT32_MAX, largest = 0;
        for(unsigned pass = 0; pass < 24; ++pass)
        {
            float sum = 0;
            const uint32_t began = esp_cpu_get_cycle_count();
            for(unsigned i = 0; i < 512; ++i)
            {
                const float angle = -52.0f + ((i * 31 + pass * 17) % 4096) * (104.0f / 4096);
                sum += functions[k](angle);
            }
            const uint32_t elapsed = esp_cpu_get_cycle_count() - began;
            math_test::sink = sum;
            if(pass >= 4)
            {
                total += elapsed;
                smallest = std::min(smallest, elapsed);
                largest = std::max(largest, elapsed);
            }
        }
        printf("MATH {\"kind\":\"%s\",\"cpu_mhz\":%u,\"cycles_mean\":%.3f,\"cycles_min\":%.3f,\"cycles_max\":%.3f}\n",
            names[k], CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, double(total)/(20*512), double(smallest)/512, double(largest)/512);
    }
}
