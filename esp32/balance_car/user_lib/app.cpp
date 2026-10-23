#include "hw/motor.h"
#include "test.h"
#include "ota.h"
#include "realtime_config.h"
#include <stdio.h>

extern "C" void app_init(void)
{
    ota::prepare_boot();
    const bool ready = motor::init() && test::init();
    if(realtime_config::WIRELESS_DEBUG_ENABLED) ota::init(ready);
    else printf("DEBUG transport=UART wireless=OFF ready=%u\n", ready);
}
