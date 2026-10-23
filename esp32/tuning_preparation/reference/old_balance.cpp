#include "old_balance.h"

#include <math.h>

namespace old_balance
{
    namespace
    {
        constexpr float PERIOD_S = 0.005f;
        constexpr float WHEEL_RADIUS_M = 0.0325f;
        constexpr float KT_NM_A = 0.013311959501823515f;
        constexpr float PITCH_TRIM_RAD = -0.0896225217620993f * 0.017453292519943295f;

        constexpr float POSITION_KP = 1.3f;
        constexpr float PITCH_KP = 0.4f;
        constexpr float PITCH_KI = 0.005f;
        constexpr float YAW_KP = 0.105f;
        constexpr float YAW_KI = 0.01f;
        constexpr float YAW_KD = 0.02f;

        constexpr float VELOCITY_INTEGRAL_LIMIT = 1.2f;
        constexpr float PITCH_INTEGRAL_LIMIT = 0.25f;
        constexpr float YAW_INTEGRAL_LIMIT = 0.5f;

        /**
         * @brief 将输入限制在对称范围内
         */
        float clamp(float value, float limit)
        {
            return fmaxf(-limit, fminf(limit, value));
        }
    }

    /**
     * @brief 将传感器姿态和轮速转换为仿真控制器的观测值
     *
     * @param[in] angle_rad 原坐标系的 X、Y、Z 欧拉角，坐标轴为右、前、上
     * @param[in] gyro_rad_s 原坐标系的机体角速度
     * @param[in] wheel_speed_rad_s 左右编码器相对车身的轮速，前进为正
     *
     * @return 前、左、上坐标系的速度、姿态及姿态角速度
     */
    observation observe(
        const float angle_rad[3],
        const float gyro_rad_s[3],
        const float wheel_speed_rad_s[2])
    {
        const float sx = sinf(angle_rad[0]);
        const float cx = cosf(angle_rad[0]);
        const float sy = sinf(angle_rad[1]);
        const float cy = cosf(angle_rad[1]);
        const float roll = atan2f(sy, cy * cx);
        const float yaw_offset = atan2f(-sy * sx, cx);
        const float yaw = angle_rad[2] + yaw_offset;
        const float sr = sinf(roll);
        const float cr = cosf(roll);

        observation measured{};
        measured.pitch_rad = atan2f(-cy * sx, hypotf(cx, sy * sx));
        measured.pitch_rate_rad_s = -cr * gyro_rad_s[0] - sr * gyro_rad_s[2];
        measured.yaw_rad = atan2f(sinf(yaw), cosf(yaw));
        measured.yaw_rate_rad_s = (-sr * gyro_rad_s[0] + cr * gyro_rad_s[2]) /
            fmaxf(0.2f, cosf(measured.pitch_rad));
        measured.speed_m_s = WHEEL_RADIUS_M * cosf(yaw_offset) *
            ((wheel_speed_rad_s[0] + wheel_speed_rad_s[1]) * 0.5f - gyro_rad_s[0]);
        return measured;
    }

    /**
     * @brief 按仿真的位置、速度、俯仰和偏航控制律计算车轮电流
     *
     * @param[in] measured 车身前进坐标下的观测值，俯仰以前倾为正
     * @param[in] target 位置、速度和偏航参考值
     * @param[in] limit 控制输出限制
     * @param[in, out] controller 积分及里程状态
     * @param[in] tuning 速度 PI 和俯仰阻尼，默认使用仿真选定值
     *
     * @return 左右车轮前进为正的电流，单位 A
     */
    output update(
        const observation &measured,
        const reference &target,
        const limits &limit,
        state &controller,
        const gains &tuning)
    {
        if(!controller.first_sample)
        {
            controller.distance_m += PERIOD_S *
                (controller.previous_speed_m_s + measured.speed_m_s) * 0.5f;
        }

        controller.first_sample = false;
        controller.previous_speed_m_s = measured.speed_m_s;

        output out{};
        out.target_speed_m_s = clamp(
            target.speed_m_s + POSITION_KP * (target.position_m - controller.distance_m),
            limit.speed_m_s);

        const float velocity_error = out.target_speed_m_s - measured.speed_m_s;
        const float velocity_integral = clamp(
            controller.velocity_integral + PERIOD_S * velocity_error,
            VELOCITY_INTEGRAL_LIMIT);
        const float target_pitch = tuning.velocity_kp * velocity_error +
                                   tuning.velocity_ki * velocity_integral;
        out.target_pitch_rad = clamp(target_pitch, limit.pitch_rad);

        const float pitch_error = measured.pitch_rad - PITCH_TRIM_RAD -
                                  out.target_pitch_rad;
        const float pitch_integral = clamp(
            controller.pitch_integral + PERIOD_S * pitch_error,
            PITCH_INTEGRAL_LIMIT);
        const float yaw_delta = target.yaw_rad - measured.yaw_rad;
        const float yaw_error = atan2f(sinf(yaw_delta), cosf(yaw_delta));
        const float yaw_integral = clamp(
            controller.yaw_integral + PERIOD_S * yaw_error,
            YAW_INTEGRAL_LIMIT);

        const float common = (PITCH_KP / KT_NM_A) * pitch_error +
                             (PITCH_KI / KT_NM_A) * pitch_integral +
                             (tuning.pitch_kd / KT_NM_A) * measured.pitch_rate_rad_s;
        const float differential = (YAW_KP / KT_NM_A) * yaw_error +
                                   (YAW_KI / KT_NM_A) * yaw_integral +
                                   (YAW_KD / KT_NM_A) *
                                       (target.yaw_rate_rad_s - measured.yaw_rate_rad_s);
        const float current[2] =
        {
            (common - differential) * 0.5f,
            (common + differential) * 0.5f
        };

        out.saturated = fmaxf(fabsf(current[0]), fabsf(current[1])) > limit.current_A;

        if(!out.saturated)
        {
            controller.pitch_integral = pitch_integral;
            controller.yaw_integral = yaw_integral;

            if(fabsf(target_pitch) <= limit.pitch_rad)
            {
                controller.velocity_integral = velocity_integral;
            }
        }

        out.current_A[0] = clamp(current[0], limit.current_A);
        out.current_A[1] = clamp(current[1], limit.current_A);
        return out;
    }
}
