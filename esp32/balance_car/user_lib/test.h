#ifndef TEST_H
#define TEST_H
#include <stddef.h>
#include <stdint.h>

namespace test
{
    // BCU3 PID payload: one control-cycle snapshot; integers and float32, little endian.
#pragma pack(push, 1)
    struct telemetry
    {
        uint64_t sample_us, imu_us;
        uint32_t cycle, dt_us, revision, mode, result, fault, flags, saturated;
        uint32_t imu_age_us, encoder_age_us, current_age_a_us, current_age_b_us, calculate_us;
        float pitch, roll, pitch_rate, yaw, yaw_rate, speed, distance;
        float target_speed, target_pitch, command_a, command_b;
        float iq_a, iq_b, id_a, id_b, raw_a, raw_b, angle_a, angle_b, bus, current_limit_A;
        float velocity_integral, pitch_integral, yaw_integral; // velocity, pitch and yaw error integrals
        float gains[10]; // velocity PI, pitch D/P/I, yaw P/I/D, position P, trim
    };
#pragma pack(pop)
    static_assert(sizeof(telemetry) == 204);
    void get_telemetry(telemetry &snapshot);
    bool init();
    bool network_command(const char *line, char *response, size_t capacity);
    void network_stats(uint32_t &control_updates, uint32_t &max_dt_us,
                       uint32_t &max_calculate_us, uint32_t &active_mode);
}

#endif
