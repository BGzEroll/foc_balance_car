#pragma once
#include <stdint.h>

namespace motor_sample_time
{
    // Read now_us AFTER the sample snapshot. Only the sample stamp is truncated
    // to 32 bits; keep the run/phase anchor in full esp_timer time.
    inline bool after_start(uint32_t sample_us, uint64_t now_us, uint64_t start_us)
    {
        const uint32_t age = static_cast<uint32_t>(now_us) - sample_us;
        return now_us >= age && now_us - age >= start_us;
    }
}
