#include "hw/motor.h"
#include "controller/balance.h"

extern "C" void app_init(void)
{
    if(motor::init())
    {
        if(!balance::init())
        {
            motor::emergency_stop();
        }
    }
}
