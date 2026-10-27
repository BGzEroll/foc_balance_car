#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <stdint.h>

namespace gamepad
{
    struct state
    {
        uint64_t received_us = 0;
        float forward = 0.0f;
        float turn = 0.0f;
        bool connected = false;
        bool neutral_seen = false;
    };

    bool init();
    void get_state(state &snapshot);
}

#endif
