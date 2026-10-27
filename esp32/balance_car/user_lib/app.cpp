#include "hw/motor.h"
#include "controller/balance.h"
#include "test.h"

extern "C" void app_init(void)
{
    if(motor::init())
    {
        if(!balance::init() || !test::init())
        {
            motor::emergency_stop();
        }
    }
}
