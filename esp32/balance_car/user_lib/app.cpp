#include "hw/sensor.h"
#include "hw/bat.h"
#include "test.h"

extern "C" void app_init(void)
{
    sensor::init();
    bat::init();
    test::init();
}
