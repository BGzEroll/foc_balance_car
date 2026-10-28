#include "hw/motor.h"
#include "hw/gamepad.h"
#include "controller/balance.h"

extern "C" void app_init(void)
{
    if(gamepad::init() && motor::init())
    {
        if(!balance::init())
        {
            motor::emergency_stop();
        }
    }
}
