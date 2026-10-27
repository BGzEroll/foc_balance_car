#include "hw/motor.h"
#include "hw/gamepad.h"
#include "controller/balance.h"
#include "test.h"

extern "C" void app_init(void)
{
    if(gamepad::init() && motor::init())
    {
        if(!balance::init() || !test::init())
        {
            motor::emergency_stop();
        }
    }
}
