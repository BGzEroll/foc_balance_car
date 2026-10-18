#include "hw/sensor.h"
#include "test.h"

extern "C" void app_init(void)
{
    sensor::init();
    test::init();
}
