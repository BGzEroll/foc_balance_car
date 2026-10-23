#ifndef OLD_BALANCE_H
#define OLD_BALANCE_H

namespace old_balance
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
        float current_A = 0.7f;
        float speed_m_s = 0.65f;
        float pitch_rad = 0.13962634f;
    };

    struct gains
    {
        float velocity_kp = 1.4f;
        float velocity_ki = 1.2f;
        float pitch_kd = 0.025f;
    };

    struct state
    {
        float distance_m = 0.0f;
        float previous_speed_m_s = 0.0f;
        float velocity_integral = 0.0f;
        float pitch_integral = 0.0f;
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
