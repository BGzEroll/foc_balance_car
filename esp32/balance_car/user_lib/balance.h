#ifndef BALANCE_H
#define BALANCE_H
#include "pid_config.h"

// Cascaded position P, velocity PI, pitch PID and yaw PID. Output is amperes.
namespace balance
{
    struct observation
    {
        float speed_m_s = 0.0f;
        float pitch_rad = 0.0f;
        float pitch_rate_rad_s = 0.0f;
        float yaw_rad = 0.0f;
        float yaw_rate_rad_s = 0.0f;
    };

    struct reference
    {
        float position_m = 0.0f;
        float speed_m_s = 0.0f;
        float yaw_rad = 0.0f;
        float yaw_rate_rad_s = 0.0f;
        float pitch_bias_rad = 0.0f;
    };

    struct limits
    {
        float current_A = 6.0f;
        float speed_m_s = pid_config::SPEED_LIMIT;
        float pitch_rad = pid_config::PITCH_TARGET_LIMIT;
    };

    struct gains
    {
        float velocity_kp = pid_config::DEFAULT_GAINS[0];
        float velocity_ki = pid_config::DEFAULT_GAINS[1];
        float pitch_kd = pid_config::DEFAULT_GAINS[2]; // common A*s/rad
        float pitch_kp = pid_config::DEFAULT_GAINS[3]; // common A/rad
        float pitch_ki = pid_config::DEFAULT_GAINS[4];
        float yaw_kp = pid_config::DEFAULT_GAINS[5];
        float yaw_ki = pid_config::DEFAULT_GAINS[6];
        float yaw_kd = pid_config::DEFAULT_GAINS[7];
        float position_kp = pid_config::DEFAULT_GAINS[8];
        float pitch_trim_rad = pid_config::DEFAULT_GAINS[9];
    };

    struct state
    {
        float distance_m = 0.0f;
        float previous_speed_m_s = 0.0f;
        float velocity_integral = 0.0f; // velocity-error integral, m
        float pitch_integral = 0.0f; // rad*s
        float yaw_integral = 0.0f;
        bool first_sample = true;
    };

    struct output
    {
        float current_A[2] = {};
        float target_speed_m_s = 0.0f;
        float target_pitch_rad = 0.0f;
        bool saturated = false;
    };

    observation observe(
        const float angle_rad[3],
        const float gyro_rad_s[3],
        const float wheel_speed_rad_s[2]);

    output update(
        const observation &measured,
        const reference &target,
        const limits &limit,
        state &controller,
        float dt_s,
        const gains &tuning = gains{});
}

#endif
