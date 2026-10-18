#ifndef FOC_CURRENT_H
#define FOC_CURRENT_H

#include <stdint.h>

namespace foc_current
{
    constexpr uint32_t FREQUENCY_HZ = 1000;
    constexpr uint32_t CPU_MHZ = 160;
    constexpr uint32_t PERIOD_CYCLES = CPU_MHZ * 1000000 / FREQUENCY_HZ;

    struct timing
    {
        uint64_t sum = 0;
        uint32_t maximum = 0;
        uint32_t minimum = UINT32_MAX;
    };

    struct state
    {
        uint32_t count = 0;
        uint32_t probe_without_foc = 0;
        uint32_t probe_with_foc = 0;
        uint32_t adc_invalid = 0;
        uint32_t compare_errors = 0;
        uint32_t deadline_misses = 0;
        uint32_t event_gaps = 0;
        uint32_t interval_min = UINT32_MAX;
        uint32_t interval_max = 0;
        uint64_t raw_sum[5] = {};
        uint16_t raw_min[5] = {4095, 4095, 4095, 4095, 4095};
        uint16_t raw_max[5] = {};
        uint16_t raw[5] = {};
        float baseline_raw[5] = {};
        float current_A[4] = {};
        float bus_V = 0.0f;
        float id_A[2] = {};
        float iq_A[2] = {};
        float duty[6] = {};
        timing acquisition;
        timing calculation;
        timing write;
        timing total;
        timing entry_phase;
        timing read_begin[5];
        timing read_end[5];
    };

    bool init();
    void get_package(state &out);
}

#endif
