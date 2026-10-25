#include "balance.h"


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

    output update(const observation &measured,const reference &target,const limits &limit,
                  state &controller,float dt_s,const gains &tuning)
    {
        if(!controller.first_sample)
            controller.distance_m += dt_s*(controller.previous_speed_m_s+measured.speed_m_s)*.5f;
        controller.first_sample=false;controller.previous_speed_m_s=measured.speed_m_s;
        output out{};
        out.target_speed_m_s=clamp(target.speed_m_s+tuning.position_kp*(target.position_m-controller.distance_m),limit.speed_m_s);
        const float ve=out.target_speed_m_s-measured.speed_m_s;
        const float vi=clamp(controller.velocity_integral+dt_s*ve,pid_config::VELOCITY_INTEGRAL_LIMIT);
        const float lean=tuning.velocity_kp*ve+tuning.velocity_ki*vi+target.pitch_bias_rad;
        out.target_pitch_rad=tuning.pitch_trim_rad+clamp(lean,limit.pitch_rad);
        const float pe=measured.pitch_rad-out.target_pitch_rad;
        const float pi=tuning.pitch_ki>0?clamp(controller.pitch_integral+dt_s*pe,pid_config::PITCH_INTEGRAL_LIMIT):0;
        const float ye=atan2f(sinf(target.yaw_rad-measured.yaw_rad),cosf(target.yaw_rad-measured.yaw_rad));
        const float yi=clamp(controller.yaw_integral+dt_s*ye,pid_config::YAW_INTEGRAL_LIMIT);
        const float common=tuning.pitch_kp*pe+tuning.pitch_ki*pi+tuning.pitch_kd*measured.pitch_rate_rad_s;
        const float differential=tuning.yaw_kp*ye+tuning.yaw_ki*yi+tuning.yaw_kd*(target.yaw_rate_rad_s-measured.yaw_rate_rad_s);
        const float current[2]={(common-differential)*.5f,(common+differential)*.5f};
        const float cap=fminf(6.0f,fabsf(limit.current_A));
        out.saturated=fmaxf(fabsf(current[0]),fabsf(current[1]))>cap;
        out.current_A[0]=clamp(current[0],cap);out.current_A[1]=clamp(current[1],cap);
        if(!out.saturated){
            controller.pitch_integral=pi;controller.yaw_integral=yi;
            if(fabsf(lean)<=limit.pitch_rad)controller.velocity_integral=vi;
        }
        return out;
    }
}
