
#include "balance.h"

#include "hw/motor.h"
#include "hw/sensor.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

namespace balance
{
    namespace
    {
        constexpr float WHEEL_RADIUS_M = 0.0325f;
        constexpr float MAX_CURRENT_A = 6.0f;

        constexpr float VELOCITY_INTEGRAL_LIMIT = 0.5f;
        constexpr float PITCH_INTEGRAL_LIMIT = 0.25f;
        constexpr float YAW_INTEGRAL_LIMIT = 0.5f;

        constexpr float FORWARD_SIGN[2] = {-1.0f, 1.0f};

        constexpr float BALANCE_CURRENT_A = 2.0f;
        constexpr float ARM_PITCH_RAD = 0.1745329252f;
        constexpr float ARM_SPEED_M_S = 0.10f;
        constexpr float MAX_TILT_RAD = 0.7853981634f;

        constexpr uint64_t IMU_TIMEOUT_US = 20000;
        constexpr uint64_t ENCODER_TIMEOUT_US = 1500;
        constexpr uint64_t CONTROL_TIMEOUT_US = 10000;

        constexpr uint32_t ARM_SAMPLES = 40;

        bool output_enabled = false;
        bool initialized = false;

        /**
         * @brief 将输入限制在对称范围内
         *
         * @param[in] value 待限制的数值
         * @param[in] limit 正数限幅值
         *
         * @return 限幅后的数值
         */
        float clamp(float value, float limit)
        {
            return fmaxf(-limit, fminf(limit, value));
        }

        /**
         * @brief 检查微秒时间戳是否有效且未超时
         *
         * @param[in] timestamp_us 采样时间戳
         * @param[in] now_us 当前时间
         * @param[in] timeout_us 最大允许延迟
         *
         * @return true 数据有效且未超时
         * @return false 时间戳无效或数据已超时
         */
        bool fresh(uint64_t timestamp_us, uint64_t now_us, uint64_t timeout_us)
        {
            return timestamp_us != 0 &&
                   now_us >= timestamp_us &&
                   now_us - timestamp_us <= timeout_us;
        }
    }

    /**
     * @brief 将传感器姿态和轮速转换为平衡控制观测量
     *
     * @param[in] angle_rad 原坐标系 X、Y、Z 欧拉角，坐标轴向右、向前、向上，单位 rad
     * @param[in] gyro_rad_s 原坐标系机体角速度，单位 rad/s
     * @param[in] wheel_speed_rad_s 左右轮相对车身的转速，前进为正，单位 rad/s
     *
     * @return 前、左、上坐标系下的速度、俯仰角及偏航状态
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
        measured.yaw_rate_rad_s =
            (-sr * gyro_rad_s[0] + cr * gyro_rad_s[2]) /
            fmaxf(0.2f, cosf(measured.pitch_rad));
        measured.speed_m_s = WHEEL_RADIUS_M * cosf(yaw_offset) *
            ((wheel_speed_rad_s[0] + wheel_speed_rad_s[1]) * 0.5f - gyro_rad_s[0]);

        return measured;
    }

    /**
     * @brief 执行位置 P、速度 PI、俯仰 PID 及偏航 PID 控制
     *
     * @param[in] measured 当前控制观测量
     * @param[in] target 目标位置、速度、姿态及前馈量
     * @param[in] limit 目标电流、速度和俯仰角限制
     * @param[in, out] controller 积分状态及累计位置
     * @param[in] dt_s 本次控制周期，单位 s
     * @param[in] tuning PID 增益及俯仰零点修正
     *
     * @return 左右轮目标电流及控制器输出状态
     *
     * @note 不执行电机输出，调用方负责检查数据有效性与控制周期
     */
    output update(
        const observation &measured,
        const reference &target,
        const limits &limit,
        state &controller,
        float dt_s,
        const gains &tuning)
    {
        if(!controller.first_sample)
        {
            controller.distance_m +=
                dt_s * (controller.previous_speed_m_s + measured.speed_m_s) * 0.5f;
        }

        controller.first_sample = false;
        controller.previous_speed_m_s = measured.speed_m_s;

        output out{};

        const float position_error = target.position_m - controller.distance_m;
        out.target_speed_m_s = clamp(
            target.speed_m_s + tuning.position_kp * position_error,
            limit.speed_m_s);

        const float velocity_error = out.target_speed_m_s - measured.speed_m_s;
        const float velocity_integral = clamp(
            controller.velocity_integral + dt_s * velocity_error,
            VELOCITY_INTEGRAL_LIMIT);

        const float lean =
            tuning.velocity_kp * velocity_error +
            tuning.velocity_ki * velocity_integral +
            target.pitch_bias_rad;

        out.target_pitch_rad =
            tuning.pitch_trim_rad + clamp(lean, limit.pitch_rad);

        const float pitch_error = measured.pitch_rad - out.target_pitch_rad;
        const float pitch_integral = tuning.pitch_ki > 0.0f ?
            clamp(controller.pitch_integral + dt_s * pitch_error,
                  PITCH_INTEGRAL_LIMIT) : 0.0f;

        const float yaw_delta = target.yaw_rad - measured.yaw_rad;
        const float yaw_error = atan2f(sinf(yaw_delta), cosf(yaw_delta));
        const float yaw_integral = clamp(
            controller.yaw_integral + dt_s * yaw_error,
            YAW_INTEGRAL_LIMIT);

        const float common =
            tuning.pitch_kp * pitch_error +
            tuning.pitch_ki * pitch_integral +
            tuning.pitch_kd * measured.pitch_rate_rad_s;

        const float differential =
            tuning.yaw_kp * yaw_error +
            tuning.yaw_ki * yaw_integral +
            tuning.yaw_kd * (target.yaw_rate_rad_s - measured.yaw_rate_rad_s);

        const float current[2] =
        {
            (common - differential) * 0.5f,
            (common + differential) * 0.5f
        };

        const float current_limit = fminf(MAX_CURRENT_A, fabsf(limit.current_A));
        out.saturated =
            fmaxf(fabsf(current[0]), fabsf(current[1])) > current_limit;

        out.current_A[0] = clamp(current[0], current_limit);
        out.current_A[1] = clamp(current[1], current_limit);

        if(!out.saturated)
        {
            controller.pitch_integral = pitch_integral;
            controller.yaw_integral = yaw_integral;

            if(fabsf(lean) <= limit.pitch_rad)
            {
                controller.velocity_integral = velocity_integral;
            }
        }

        return out;
    }

    namespace
    {
        /**
         * @brief 在 IMU 事件驱动下执行姿态估计及平衡控制
         *
         * @param[in] arg 未使用
         *
         * @note 在 Core 0 执行，FOC 任务仍在 Core 1
         */
        void control_task(void *)
        {
            state controller{};
            gains tuning{};
            reference target{};
            limits limit{};
            limit.current_A = BALANCE_CURRENT_A;

            bool previous_ready = false;
            bool active = false;
            bool stopped = false;

            uint32_t arm_samples = 0;
            uint64_t previous_imu_us = 0;
            uint64_t last_command_us = 0;

            while(true)
            {
                if(!sensor::wait_for_imu())
                {
                    const uint64_t now_us =
                        static_cast<uint64_t>(esp_timer_get_time());

                    if(active && last_command_us != 0 &&
                       now_us - last_command_us > IMU_TIMEOUT_US)
                    {
                        motor::emergency_stop();
                        active = false;
                        stopped = true;
                    }

                    continue;
                }

                sensor::package sensors{};
                motor::foc_info foc{};

                const bool sensor_ok = sensor::get_package(sensors);
                const bool motor_ok = motor::get_info(foc);
                const uint64_t now_us =
                    static_cast<uint64_t>(esp_timer_get_time());

                if(!sensor_ok || !motor_ok || !foc.ready || foc.fault.active)
                {
                    if(active)
                    {
                        motor::emergency_stop();
                        active = false;
                        stopped = true;
                    }

                    previous_ready = false;
                    arm_samples = 0;
                    continue;
                }

                if(!previous_ready)
                {
                    sensor::reset_attitude();

                    controller = state{};
                    target = reference{};

                    previous_imu_us = 0;
                    previous_ready = true;
                }

                const bool fresh_data =
                    fresh(sensors.imu.timestamp_us, now_us, IMU_TIMEOUT_US) &&
                    fresh(sensors.left_encoder.timestamp_us, now_us, ENCODER_TIMEOUT_US) &&
                    fresh(sensors.right_encoder.timestamp_us, now_us, ENCODER_TIMEOUT_US) &&
                    foc.current[0].valid &&
                    foc.current[1].valid &&
                    fabsf(foc.current[0].encoder_direction) == 1.0f &&
                    fabsf(foc.current[1].encoder_direction) == 1.0f;

                const uint64_t dt_us = previous_imu_us == 0 ? 5000 :
                    sensors.imu.timestamp_us > previous_imu_us ?
                    sensors.imu.timestamp_us - previous_imu_us : 0;

                if(!fresh_data || dt_us == 0 || dt_us > CONTROL_TIMEOUT_US)
                {
                    if(active)
                    {
                        motor::emergency_stop();
                        active = false;
                        stopped = true;
                    }

                    arm_samples = 0;

                    if(!active && dt_us > CONTROL_TIMEOUT_US)
                    {
                        sensor::reset_attitude();
                        previous_imu_us = 0;
                    }

                    continue;
                }

                previous_imu_us = sensors.imu.timestamp_us;

                const float wheel_angle_rad[2] =
                {
                    FORWARD_SIGN[0] * foc.current[0].encoder_direction *
                        sensors.left_encoder.angle_rad,

                    FORWARD_SIGN[1] * foc.current[1].encoder_direction *
                        sensors.right_encoder.angle_rad
                };

                if(!sensor::update_attitude(
                        sensors,
                        wheel_angle_rad,
                        active,
                        active))
                {
                    if(active)
                    {
                        motor::emergency_stop();
                        active = false;
                        stopped = true;
                    }

                    arm_samples = 0;
                    continue;
                }

                const float wheel_speed_rad_s[2] =
                {
                    FORWARD_SIGN[0] * foc.current[0].encoder_direction *
                        sensors.left_encoder.speed_rad_s,

                    FORWARD_SIGN[1] * foc.current[1].encoder_direction *
                        sensors.right_encoder.speed_rad_s
                };

                const observation measured = observe(
                    sensors.imu.angle,
                    sensors.imu.gyro,
                    wheel_speed_rad_s);

                const bool valid =
                    isfinite(measured.pitch_rad) &&
                    isfinite(measured.pitch_rate_rad_s) &&
                    isfinite(measured.yaw_rad) &&
                    isfinite(measured.yaw_rate_rad_s) &&
                    isfinite(measured.speed_m_s) &&
                    isfinite(sensors.imu.angle[0]) &&
                    isfinite(sensors.imu.angle[1]);

                if(!valid ||
                   fabsf(sensors.imu.angle[0]) > MAX_TILT_RAD ||
                   fabsf(sensors.imu.angle[1]) > MAX_TILT_RAD)
                {
                    if(active)
                    {
                        motor::emergency_stop();
                        active = false;
                        stopped = true;
                    }

                    arm_samples = 0;
                    continue;
                }

                const float dt = static_cast<float>(dt_us) * 1.0e-6f;

                if(!output_enabled)
                {
                    // 仅更新控制器状态，不向电机下发电流
                    update(measured, target, limit, controller, dt, tuning);
                    continue;
                }

                if(stopped)
                {
                    continue;
                }

                if(!active)
                {
                    if(fabsf(measured.pitch_rad) <= ARM_PITCH_RAD &&
                       fabsf(measured.speed_m_s) <= ARM_SPEED_M_S)
                    {
                        ++arm_samples;
                    }
                    else
                    {
                        arm_samples = 0;
                    }

                    if(arm_samples < ARM_SAMPLES)
                    {
                        continue;
                    }

                    active = true;
                    controller = state{};
                    target = reference{};
                    target.yaw_rad = measured.yaw_rad;
                }

                const output command = update(
                    measured,
                    target,
                    limit,
                    controller,
                    dt,
                    tuning);

                if(!isfinite(command.current_A[0]) ||
                   !isfinite(command.current_A[1]))
                {
                    motor::emergency_stop();
                    active = false;
                    stopped = true;
                    continue;
                }

                bool accepted = true;

                for(uint32_t i = 0; i < 2; ++i)
                {
                    motor::target_current current{};
                    current.iq_A = FORWARD_SIGN[i] * command.current_A[i];

                    accepted = motor::set_target_current(
                        static_cast<motor::motor_id>(i),
                        current) && accepted;
                }

                if(!accepted)
                {
                    motor::emergency_stop();
                    active = false;
                    stopped = true;
                    continue;
                }

                last_command_us = now_us;
            }
        }
    }

    /**
     * @brief 创建 200Hz 平衡控制任务
     *
     * @param[in] enable_output 是否启用自动平衡电流输出
     *
     * @return true 任务已创建
     * @return false 任务创建失败
     *
     * @note 默认只运行观测和控制计算，不向电机下发电流
     */
    bool init(bool enable_output)
    {
        if(initialized)
        {
            return true;
        }

        output_enabled = enable_output;

        if(xTaskCreatePinnedToCore(
                control_task, "balance_control", 8192,
                nullptr, 4, nullptr, 0) != pdPASS)
        {
            return false;
        }

        initialized = true;
        return true;
    }
}
