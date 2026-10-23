#ifndef BALANCE_H
#define BALANCE_H

// Fixed LQI update; observe() retains the hardware-tested coordinate mapping.
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
    };

    struct limits
    {
        float current_A = 2.0f;
        float speed_m_s = 0.65f;
        float pitch_rad = 0.13962634f;
    };

    struct gains
    {
        float velocity_kp = 1.4f;
        float velocity_ki = 1.2f;
        float pitch_kd = 1.878010521f; // A*s/rad, total common effort
        float pitch_kp = 30.04816834f;
        float pitch_ki = 0.3756021042f;
        float yaw_kp = 7.887644188f;
        float yaw_ki = 0.7512042084f;
        float yaw_kd = 1.502408417f;
        float position_kp = 1.3f;
        float pitch_trim_rad = -0.001564208089f;
    };

    struct state
    {
        float distance_m = 0.0f;
        float previous_speed_m_s = 0.0f;
        float velocity_integral = 0.0f; // LQI position error integral, m*s
        float pitch_integral = 0.0f; // LQI unused zero
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
        const gains &tuning = gains{});
}

#endif
