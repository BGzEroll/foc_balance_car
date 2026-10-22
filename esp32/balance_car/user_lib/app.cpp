extern void foc_bench_init();

extern "C" void app_init(void)
{
    foc_bench_init();
}
