#include "balance.h"

#include "hw/motor.h"
#include "hw/gamepad.h"
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
        constexpr float MAX_CURRENT_A = 4.0f;

        constexpr float VELOCITY_INTEGRAL_LIMIT = 0.5f;
        constexpr float PITCH_INTEGRAL_LIMIT = 0.25f;
        constexpr float YAW_INTEGRAL_LIMIT = 0.5f;

        constexpr float FORWARD_SIGN[2] = {-1.0f, 1.0f};

        constexpr float ARM_PITCH_RAD = 0.1745329252f;
        constexpr float ARM_SPEED_M_S = 0.10f;
        constexpr float MAX_TILT_RAD = 0.7853981634f;

        constexpr uint64_t IMU_TIMEOUT_US = 20000;
        constexpr uint64_t ENCODER_TIMEOUT_US = 1500;
        constexpr uint64_t CONTROL_TIMEOUT_US = 20000;

        constexpr uint32_t ARM_SAMPLES = 40;

        enum class action : uint32_t
        {
            NONE,
            START,
            STOP
        };

        bool output_enabled = false;
        bool initialized = false;
        action requested_action = action::NONE;
        status latest_status{};
        portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
        reference requested_target{};

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
        // 积分只学习低速稳态偏置，不积累加速、制动过程中的大速度误差。
        const bool learn_velocity_bias = fabsf(out.target_speed_m_s) < 0.10f &&
            fabsf(measured.speed_m_s) < 0.10f;
        const float velocity_integral = clamp(
            controller.velocity_integral +
                (learn_velocity_bias ? dt_s * velocity_error : 0.0f),
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

        const float current_limit = fminf(MAX_CURRENT_A, fabsf(limit.current_A));
        const float balance_current = clamp(common * 0.5f, current_limit);
        const float yaw_limit = fmaxf(0.0f, current_limit - fabsf(balance_current));
        const float yaw_current = clamp(differential * 0.5f, yaw_limit);
        const bool balance_saturated = fabsf(common * 0.5f) > current_limit;
        const bool yaw_saturated = fabsf(differential * 0.5f) > yaw_limit;
        out.saturated = balance_saturated || yaw_saturated;
        out.current_A[0] = balance_current - yaw_current;
        out.current_A[1] = balance_current + yaw_current;

        if(!balance_saturated)
        {
            controller.pitch_integral = pitch_integral;

            if(fabsf(lean) <= limit.pitch_rad)
            {
                controller.velocity_integral = velocity_integral;
            }
        }
        if(!yaw_saturated)
        {
            controller.yaw_integral = yaw_integral;
        }

        return out;
    }

    namespace
    {
        constexpr uint64_t GAMEPAD_TIMEOUT_US = 300000;
        constexpr float MANUAL_SPEED_M_S = 0.9f;
        constexpr float MANUAL_YAW_RATE_RAD_S = 3.1415926536f;

        struct session
        {
            state controller{};
            reference target{};
            limits limit{};
            output held_command{};

            uint64_t previous_imu_us = 0;
            uint64_t last_control_us = 0;
            uint32_t arm_samples = 0;

            bool previous_ready = false;
            bool active = false;
            bool stopped = false;
            bool manual_control = false;
            bool manual_braking = false;
            float brake_speed_m_s = 0.0f;
        };

        /**
         * @brief 将车体正方向电流转换为两台电机的目标电流
         *
         * @param[in, out] control 当前运行状态
         * @param[in] command 车体正方向电流
         *
         * @return true 两台电机均接收命令
         * @return false 命令失败，已锁存停机
         */
        bool send_current(session &control, const output &command)
        {
            bool accepted = true;

            for(uint32_t i = 0; i < 2; ++i)
            {
                motor::target_current current{};
                current.iq_A = FORWARD_SIGN[i] * command.current_A[i];
                accepted = motor::set_target_current(
                    static_cast<motor::motor_id>(i), current) && accepted;
            }

            if(!accepted)
            {
                motor::emergency_stop();
                control.active = false;
                control.stopped = true;
            }

            return accepted;
        }

        /**
         * @brief 清除控制历史和遥控停车状态，不改变硬件故障
         */
        void reset_control(session &control)
        {
            control.active = false;
            control.arm_samples = 0;
            control.manual_control = false;
            control.manual_braking = false;
            control.brake_speed_m_s = 0.0f;
            control.held_command = output{};
            control.last_control_us = 0;
            control.controller = state{};
            control.target = reference{};

            portENTER_CRITICAL(&status_lock);
            latest_status.active = false;
            latest_status.command = output{};
            requested_target = reference{};
            portEXIT_CRITICAL(&status_lock);
        }

        /**
         * @brief 软退出平衡，保留电机校准并等待扶正
         */
        void standby(session &control)
        {
            reset_control(control);

            if(output_enabled && !control.stopped)
            {
                send_current(control, output{});
            }
        }

        /**
         * @brief 发布最近一次控制状态，不执行串口或日志输出
         */
        void publish_status(
            const session &control,
            const observation &measured,
            const output &command,
            uint64_t now_us,
            uint64_t imu_us,
            uint32_t dt_us,
            uint32_t calculate_us)
        {
            status snapshot{};
            snapshot.sample_us = now_us;
            snapshot.imu_us = imu_us;
            snapshot.dt_us = dt_us;
            snapshot.calculate_us = calculate_us;
            snapshot.active = control.active;
            snapshot.measured = measured;
            snapshot.controller = control.controller;
            snapshot.command = command;

            portENTER_CRITICAL(&status_lock);
            latest_status = snapshot;
            portEXIT_CRITICAL(&status_lock);
        }

        /**
         * @brief 连续确认直立、低速且摇杆回中后恢复平衡
         */
        bool arm(
            session &control,
            const observation &measured,
            const gamepad::state &pad,
            float upright_cos,
            uint64_t now_us)
        {
            if(!send_current(control, output{}))
            {
                return false;
            }

            const bool centered = !pad.connected ||
                (fresh(pad.received_us, now_us, GAMEPAD_TIMEOUT_US) &&
                 pad.forward == 0.0f && pad.turn == 0.0f);

            if(upright_cos >= cosf(ARM_PITCH_RAD) &&
               fabsf(measured.speed_m_s) <= ARM_SPEED_M_S &&
               fabsf(measured.pitch_rate_rad_s) <= 0.35f && centered)
            {
                ++control.arm_samples;
            }
            else
            {
                control.arm_samples = 0;
            }

            if(control.arm_samples < ARM_SAMPLES)
            {
                return false;
            }

            control.active = true;
            control.controller = state{};
            control.target = reference{};
            control.target.yaw_rad = measured.yaw_rad;

            portENTER_CRITICAL(&status_lock);
            requested_target = control.target;
            portEXIT_CRITICAL(&status_lock);
            return true;
        }

        /**
         * @brief 将手柄输入转换为速度、偏航角速度和停车位置
         *
         * @note 松杆制动到首次零速附近即锁定停车点，掉线按松杆处理
         */
        void update_reference(
            session &control,
            const observation &measured,
            const gamepad::state &pad,
            float dt_s,
            uint64_t now_us)
        {
            if(!pad.neutral_seen && !control.manual_control)
            {
                portENTER_CRITICAL(&status_lock);
                control.target = requested_target;
                portEXIT_CRITICAL(&status_lock);
                return;
            }

            if(!control.manual_control)
            {
                control.target.position_m = control.controller.distance_m;
                control.target.speed_m_s = 0.0f;
                control.target.yaw_rate_rad_s = 0.0f;
                control.target.yaw_rad = measured.yaw_rad;
            }

            control.manual_control = true;
            control.limit.speed_m_s = MANUAL_SPEED_M_S;

            const bool input_fresh = pad.connected && pad.neutral_seen &&
                fresh(pad.received_us, now_us, GAMEPAD_TIMEOUT_US);
            const float speed = input_fresh ? MANUAL_SPEED_M_S * pad.forward : 0.0f;
            const float turn = input_fresh ? MANUAL_YAW_RATE_RAD_S * pad.turn : 0.0f;
            const float distance_m = control.controller.distance_m +
                (control.controller.first_sample ? 0.0f :
                    dt_s * (control.controller.previous_speed_m_s + measured.speed_m_s) * 0.5f);

            if(speed != 0.0f)
            {
                control.manual_braking = true;
                control.brake_speed_m_s = measured.speed_m_s;
            }

            if(control.manual_braking)
            {
                control.target.position_m = distance_m;

                if(speed == 0.0f &&
                   (fabsf(measured.speed_m_s) < 0.03f ||
                    measured.speed_m_s * control.brake_speed_m_s <= 0.0f))
                {
                    control.manual_braking = false;
                }
            }

            if(turn != 0.0f || control.target.yaw_rate_rad_s != 0.0f)
            {
                control.target.yaw_rad = measured.yaw_rad;
                control.controller.yaw_integral = 0.0f;
            }

            control.target.speed_m_s = speed;
            control.target.yaw_rate_rad_s = turn;
        }

        /**
         * @brief 在 Core 0 按 IMU 更新执行姿态估计和平衡控制
         *
         * @note 通知只用于唤醒，数据新鲜度和积分间隔均由采样时间戳决定
         */
        void control_task(void *)
        {
            session control{};
            const gains tuning{};

            while(true)
            {
                sensor::wait_for_imu();

                sensor::package sensors{};
                motor::foc_info foc{};
                const bool sensor_ok = sensor::get_package(sensors);
                const bool motor_ok = motor::get_info(foc);
                const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
                const action request = __atomic_exchange_n(
                    &requested_action, action::NONE, __ATOMIC_ACQ_REL);

                if(request == action::STOP)
                {
                    standby(control);
                    output_enabled = false;
                }

                if(!sensor_ok || !motor_ok || !foc.ready || foc.fault.active)
                {
                    if(control.active)
                    {
                        motor::emergency_stop();
                        control.stopped = true;
                    }

                    control.previous_ready = false;
                    reset_control(control);
                    continue;
                }

                if(!control.previous_ready || request == action::START)
                {
                    sensor::reset_attitude();
                    reset_control(control);
                    control.previous_imu_us = 0;
                    control.previous_ready = true;

                    if(request == action::START)
                    {
                        output_enabled = true;
                    }
                }

                const bool fresh_data =
                    fresh(sensors.imu.timestamp_us, now_us, IMU_TIMEOUT_US) &&
                    fresh(sensors.left_encoder.timestamp_us, now_us, ENCODER_TIMEOUT_US) &&
                    fresh(sensors.right_encoder.timestamp_us, now_us, ENCODER_TIMEOUT_US) &&
                    foc.current[0].valid && foc.current[1].valid &&
                    fabsf(foc.current[0].encoder_direction) == 1.0f &&
                    fabsf(foc.current[1].encoder_direction) == 1.0f;
                const uint64_t dt_us = control.previous_imu_us == 0 ? 5000 :
                    sensors.imu.timestamp_us > control.previous_imu_us ?
                    sensors.imu.timestamp_us - control.previous_imu_us : 0;

                if(dt_us == 0 || !fresh_data || dt_us > CONTROL_TIMEOUT_US)
                {
                    if(control.active && dt_us <= CONTROL_TIMEOUT_US &&
                       fresh(control.last_control_us, now_us, CONTROL_TIMEOUT_US) &&
                       fresh(sensors.imu.timestamp_us, now_us, IMU_TIMEOUT_US))
                    {
                        // 短时迟到保持上一有效电流，不清除积分和停车位置。
                        send_current(control, control.held_command);
                    }
                    else
                    {
                        standby(control);

                        if(dt_us > CONTROL_TIMEOUT_US)
                        {
                            sensor::reset_attitude();
                            control.previous_imu_us = 0;
                        }
                    }

                    continue;
                }

                control.previous_imu_us = sensors.imu.timestamp_us;

                const float wheel_angle_rad[2] =
                {
                    FORWARD_SIGN[0] * foc.current[0].encoder_direction *
                        sensors.left_encoder.angle_rad,
                    FORWARD_SIGN[1] * foc.current[1].encoder_direction *
                        sensors.right_encoder.angle_rad
                };

                if(!sensor::update_attitude(
                        sensors, wheel_angle_rad, control.active, control.active))
                {
                    standby(control);
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
                    sensors.imu.angle, sensors.imu.gyro, wheel_speed_rad_s);
                const bool valid =
                    isfinite(measured.pitch_rad) && isfinite(measured.pitch_rate_rad_s) &&
                    isfinite(measured.yaw_rad) && isfinite(measured.yaw_rate_rad_s) &&
                    isfinite(measured.speed_m_s) &&
                    isfinite(sensors.imu.angle[0]) && isfinite(sensors.imu.angle[1]);

                if(!valid)
                {
                    standby(control);
                    continue;
                }

                const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;
                const float upright_cos =
                    cosf(sensors.imu.angle[0]) * cosf(sensors.imu.angle[1]);
                output command{};
                uint32_t calculate_us = 0;

                if(upright_cos < cosf(MAX_TILT_RAD))
                {
                    standby(control);
                }
                else if(!output_enabled)
                {
                    update(measured, control.target, control.limit, control.controller, dt_s, tuning);
                }
                else if(!control.stopped)
                {
                    gamepad::state pad{};
                    gamepad::get_state(pad);

                    if(control.active || arm(control, measured, pad, upright_cos, now_us))
                    {
                        update_reference(control, measured, pad, dt_s, now_us);
                        const uint64_t began_us = esp_timer_get_time();
                        command = update(
                            measured, control.target, control.limit, control.controller, dt_s, tuning);
                        calculate_us = esp_timer_get_time() - began_us;

                        if(!isfinite(command.current_A[0]) || !isfinite(command.current_A[1]))
                        {
                            motor::emergency_stop();
                            control.active = false;
                            control.stopped = true;
                            command = output{};
                        }
                        else if(send_current(control, command))
                        {
                            control.held_command = command;
                            control.last_control_us = now_us;
                        }
                        else
                        {
                            command = output{};
                        }
                    }
                }

                publish_status(control, measured, command, now_us,
                    sensors.imu.timestamp_us, dt_us, calculate_us);
            }
        }
    }

    /**
     * @brief 创建 200 Hz 平衡任务，默认仅运行观测和控制计算
     *
     * @param[in] enable_output 是否允许扶正后自动进入平衡
     *
     * @return true 任务已创建
     * @return false 任务创建失败
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

    /**
     * @brief 在电机校准完成且无故障时请求启动平衡
     *
     * @return true 请求已接收，后续仍需连续确认直立和低速
     * @return false 控制器未初始化或电机未就绪
     */
    bool start()
    {
        motor::foc_info foc{};

        if(!initialized || !motor::get_info(foc) || !foc.ready || foc.fault.active)
        {
            return false;
        }

        __atomic_store_n(&requested_action, action::START, __ATOMIC_RELEASE);
        return true;
    }

    /**
     * @brief 请求停止平衡输出，保留校准且不清除硬件故障
     */
    void stop()
    {
        __atomic_store_n(&requested_action, action::STOP, __ATOMIC_RELEASE);
    }

    /**
     * @brief 设置非遥控模式下的位置、速度和姿态参考
     *
     * @param[in] target 控制参考，单位与 reference 字段一致
     *
     * @return true 正在平衡且参考已接收
     * @return false 未在平衡或参考包含非有限值
     */
    bool set_reference(const reference &target)
    {
        if(!isfinite(target.position_m) || !isfinite(target.speed_m_s) ||
           !isfinite(target.yaw_rad) || !isfinite(target.yaw_rate_rad_s) ||
           !isfinite(target.pitch_bias_rad))
        {
            return false;
        }

        portENTER_CRITICAL(&status_lock);
        const bool active = latest_status.active;

        if(active)
        {
            requested_target = target;
        }

        portEXIT_CRITICAL(&status_lock);
        return active;
    }

    /**
     * @brief 获取最近一次控制状态快照
     */
    void get_status(status &snapshot)
    {
        portENTER_CRITICAL(&status_lock);
        snapshot = latest_status;
        portEXIT_CRITICAL(&status_lock);
    }
}
