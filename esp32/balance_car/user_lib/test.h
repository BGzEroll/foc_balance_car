#ifndef TEST_H
#define TEST_H
#include <stddef.h>
#include <stdint.h>

namespace test
{
    bool init();
    bool network_command(const char *line, char *response, size_t capacity);
    void network_stats(uint32_t &control_updates, uint32_t &max_dt_us,
                       uint32_t &max_calculate_us, uint32_t &active_mode);
}

#endif
