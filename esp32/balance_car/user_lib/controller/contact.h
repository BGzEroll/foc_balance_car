#ifndef CONTACT_H
#define CONTACT_H

#include "hw/sensor.h"

namespace contact
{
    enum class condition : uint32_t
    {
        UNKNOWN,
        MATCHED_ACCELERATION,
        TRACTION_MISMATCH,
        LOW_SPECIFIC_FORCE,
        HIGH_SPECIFIC_FORCE,
        IMU_NEAR_RAIL
    };

    struct estimate
    {
        uint64_t timestamp_us = 0;
        uint32_t calculate_us = 0;
        uint32_t flags = 0;
        condition candidate = condition::UNKNOWN;
        float gravity[3] = {};
        float gravity_norm_g = 0.0f;
        float force_ratio = 0.0f;
        float body_accel_m_s2 = 0.0f;
        float wheel_accel_m_s2[2] = {};
        float residual_m_s2[2] = {};
        float angular_rate_rad_s = 0.0f;
        float angular_accel_rad_s2 = 0.0f;
        float yaw_rate_residual_rad_s = 0.0f;
    };

    struct observer
    {
        uint64_t previous_us = 0;
        bool initialized = false;
        float previous_wheel_speed[2] = {};
        float previous_gyro[3] = {};
        float wheel_accel[2] = {};
        float angular_accel[3] = {};
        float body_accel = 0.0f;
        float low_force_s = 0.0f;
        float mismatch_s = 0.0f;
        float matched_s = 0.0f;
    };

    // 只输出可核验的特征与候选状态，不向控制器提供接地许可。
    estimate update(observer &state, const sensor::imu_data &imu,
        const float forward_wheel_rad_s[2], float yaw_rate_rad_s, float dt_s);
}

#endif
