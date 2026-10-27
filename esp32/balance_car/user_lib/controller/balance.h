#ifndef BALANCE_H
#define BALANCE_H

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
        float speed_m_s = 0.35f;
        float pitch_rad = 0.2617993878f;
    };

    struct gains
    {
        float velocity_kp = 0.95f;
        float velocity_ki = 0.10f;

        float pitch_kp = 20.0f;
        float pitch_ki = 0.0f;
        float pitch_kd = 2.10f;

        float yaw_kp = 9.0f;
        float yaw_ki = 0.60f;
        float yaw_kd = 2.0f;

        float position_kp = 2.20f;
        float pitch_trim_rad = -0.001564208089f;
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
        float dt_s,
        const gains &tuning = gains{});
    bool init(bool enable_output = false);
}

#endif
