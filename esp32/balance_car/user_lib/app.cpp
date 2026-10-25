#include "hw/motor.h"
#include "test.h"
#include "pid_config.h"
#include "ota.h"
#include "realtime_config.h"
#include <stdio.h>

extern "C" void app_init(void)
{
    ota::prepare_boot();
    const bool ready = motor::init() && test::init();
    printf("CONTROL schema=pid_current_v1 design=%s Ts_us=5000 max_current_A=6 boot=STOPPED\n",pid_config::DESIGN_SHA256);
    if(realtime_config::WIRELESS_DEBUG_ENABLED) ota::init(ready);
    else printf("DEBUG transport=UART wireless=OFF ready=%u\n", ready);
}
