#include "hw/motor.h"
#include "test.h"

extern "C" void app_init(void)
{
    if(motor::init())
    {
        test::init();
    }
}
