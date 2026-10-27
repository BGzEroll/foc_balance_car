#include "contact.h"
#include <math.h>
#include <stdlib.h>

namespace contact
{
    estimate update(observer &state, const sensor::imu_data &imu,
        const float wheel_rad_s[2], float yaw_rate_rad_s, float dt_s)
    {
        estimate out{};
        out.timestamp_us = imu.timestamp_us;
        out.gravity_norm_g = imu.gravity_norm_g;
        float norm2 = 0.0f, rate2 = 0.0f;
        for(uint32_t i = 0; i < 3; ++i)
        {
            out.gravity[i] = imu.gravity[i];
            norm2 += imu.acc[i] * imu.acc[i];
            rate2 += imu.gyro[i] * imu.gyro[i];
            if(abs(int(imu.raw_acc[i])) >= 32700) out.flags |= 1U << i;
            if(abs(int(imu.raw_gyro[i])) >= 32700) out.flags |= 1U << (i + 3);
        }
        out.angular_rate_rad_s = sqrtf(rate2);
        if(!(imu.gravity_norm_g > 0.1f) || !(dt_s > 0.0f && dt_s <= 0.02f))
        {
            state = observer{};
            return out;
        }
        if(state.previous_us && (imu.timestamp_us <= state.previous_us ||
           imu.timestamp_us - state.previous_us > 20000)) state = observer{};
        state.previous_us = imu.timestamp_us;
        out.flags |= 1U << 7;
        out.force_ratio = sqrtf(norm2) / imu.gravity_norm_g;

        const float sx = sinf(imu.angle[0]), cx = cosf(imu.angle[0]);
        const float sy = sinf(imu.angle[1]);
        const float heading_scale = cosf(atan2f(-sy * sx, cx));
        float speed[2];
        for(uint32_t i = 0; i < 2; ++i)
            speed[i] = 0.0325f * heading_scale * (wheel_rad_s[i] - imu.gyro[0]);
        out.yaw_rate_residual_rad_s = (speed[1] - speed[0]) / 0.1325364f - yaw_rate_rad_s;

        // 以前静止重力幅值归一化的水平前向加速度；尚未做六面校准和 IMU 杆臂补偿。
        float accel[3], vertical = 0.0f;
        for(uint32_t i = 0; i < 3; ++i)
        {
            accel[i] = 9.80665f * (imu.acc[i] / imu.gravity_norm_g - imu.gravity[i]);
            vertical += accel[i] * imu.gravity[i];
        }
        const float forward_projection = sqrtf(fmaxf(0.0f, 1.0f - imu.gravity[1] * imu.gravity[1]));
        const float forward_accel = forward_projection > 0.3f ?
            (accel[1] - imu.gravity[1] * vertical) / forward_projection : 0.0f;
        const float gain = dt_s / (0.02f + dt_s);
        if(!state.initialized)
        {
            for(uint32_t i = 0; i < 2; ++i) state.previous_wheel_speed[i] = speed[i];
            for(uint32_t i = 0; i < 3; ++i) state.previous_gyro[i] = imu.gyro[i];
            state.body_accel = forward_accel;
            state.initialized = true;
        }
        state.body_accel += gain * (forward_accel - state.body_accel);
        out.body_accel_m_s2 = state.body_accel;
        for(uint32_t i = 0; i < 2; ++i)
        {
            const float derivative = (speed[i] - state.previous_wheel_speed[i]) / dt_s;
            state.previous_wheel_speed[i] = speed[i];
            state.wheel_accel[i] += gain * (derivative - state.wheel_accel[i]);
            out.wheel_accel_m_s2[i] = state.wheel_accel[i];
            out.residual_m_s2[i] = state.wheel_accel[i] - state.body_accel;
        }
        float angular_accel2 = 0.0f;
        for(uint32_t i = 0; i < 3; ++i)
        {
            const float derivative = (imu.gyro[i] - state.previous_gyro[i]) / dt_s;
            state.previous_gyro[i] = imu.gyro[i];
            state.angular_accel[i] += gain * (derivative - state.angular_accel[i]);
            angular_accel2 += state.angular_accel[i] * state.angular_accel[i];
        }
        out.angular_accel_rad_s2 = sqrtf(angular_accel2);
        // 没有可靠杆臂参数时，大角运动只记录特征，不强判滑移。
        const bool quiet_rotation = out.angular_rate_rad_s < 0.5f &&
            out.angular_accel_rad_s2 < 4.0f && forward_projection > 0.7f;
        if(quiet_rotation) out.flags |= 1U << 6;
        const float wheel_excitation = fmaxf(fabsf(out.wheel_accel_m_s2[0]), fabsf(out.wheel_accel_m_s2[1]));
        const bool excited = wheel_excitation > 1.0f || fabsf(out.body_accel_m_s2) > 1.0f;
        if(excited) out.flags |= 1U << 8;
        const float residual = fmaxf(fabsf(out.residual_m_s2[0]), fabsf(out.residual_m_s2[1]));
        const bool mismatch = quiet_rotation && excited && residual > 3.0f &&
            residual > 0.6f * (wheel_excitation + fabsf(out.body_accel_m_s2) + 0.5f);
        state.low_force_s = out.force_ratio < 0.35f ? state.low_force_s + dt_s : 0.0f;
        state.mismatch_s = mismatch ? state.mismatch_s + dt_s : 0.0f;
        state.matched_s = quiet_rotation && excited && residual < 1.5f ? state.matched_s + dt_s : 0.0f;
        if(out.flags & 0x3fU) out.candidate = condition::IMU_NEAR_RAIL;
        else if(out.force_ratio > 1.8f) out.candidate = condition::HIGH_SPECIFIC_FORCE;
        else if(state.low_force_s >= 0.015f) out.candidate = condition::LOW_SPECIFIC_FORCE;
        else if(state.mismatch_s >= 0.05f) out.candidate = condition::TRACTION_MISMATCH;
        else if(state.matched_s >= 0.05f) out.candidate = condition::MATCHED_ACCELERATION;
        return out;
    }
}
