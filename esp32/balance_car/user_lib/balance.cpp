#include "balance.h"
#include "lqi_config.h"

#include <math.h>

namespace balance
{
    namespace
    {
        constexpr float WHEEL_RADIUS_M = 0.0325f;

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
     * @brief 固定LQI：六状态加位置及偏航误差积分，直接输出电流
     *
     * @param[in] measured 车身前进坐标下的观测值，俯仰以前倾为正
     * @param[in] target 位置、速度和偏航参考值
     * @param[in] limit 控制输出限制
     * @param[in, out] controller 积分及里程状态
     * @param[in] tuning 速度 PI 和俯仰阻尼，默认使用仿真选定值
     *
     * @return 左右车轮前进为正的电流，单位 A
     */
    output update(const observation &measured,const reference &target,const limits &limit,
                  state &controller,const gains &tuning)
    {
        (void)tuning;
        if(!controller.first_sample)
            controller.distance_m += lqi_config::PERIOD_S *
                (controller.previous_speed_m_s + measured.speed_m_s) * 0.5f;
        controller.first_sample=false;controller.previous_speed_m_s=measured.speed_m_s;
        const float yaw=measured.yaw_rad-target.yaw_rad;
        const float error[8]={controller.distance_m-target.position_m,
            measured.speed_m_s-target.speed_m_s,measured.pitch_rad-lqi_config::HARDWARE_TRIM_RAD,
            measured.pitch_rate_rad_s,atan2f(sinf(yaw),cosf(yaw)),
            measured.yaw_rate_rad_s-target.yaw_rate_rad_s,controller.velocity_integral,controller.yaw_integral};
        float effort[2]={};
        for(unsigned row=0;row<2;++row)
            for(unsigned column=0;column<8;++column)
                effort[row]-=lqi_config::K[row][column]*error[column];
        const float current[2]={(effort[0]-effort[1])*0.5f,(effort[0]+effort[1])*0.5f};
        const float cap=fminf(6.0f,fabsf(limit.current_A));
        output out{};out.saturated=fmaxf(fabsf(current[0]),fabsf(current[1]))>cap;
        out.current_A[0]=clamp(current[0],cap);out.current_A[1]=clamp(current[1],cap);
        out.target_speed_m_s=target.speed_m_s;out.target_pitch_rad=lqi_config::HARDWARE_TRIM_RAD;
        if(!out.saturated)
        {
            controller.velocity_integral=clamp(controller.velocity_integral+lqi_config::PERIOD_S*error[0],lqi_config::POSITION_INTEGRAL_LIMIT);
            controller.yaw_integral=clamp(controller.yaw_integral+lqi_config::PERIOD_S*error[4],lqi_config::YAW_INTEGRAL_LIMIT);
        }
        controller.pitch_integral=0.0f;return out;
    }
}
