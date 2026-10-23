#include "hw/motor.h"
#include "test.h"
#include "ota.h"

extern "C" void app_init(void)
{
    ota::prepare_boot();
    const bool ready = motor::init() && test::init();
    ota::init(ready);
}
