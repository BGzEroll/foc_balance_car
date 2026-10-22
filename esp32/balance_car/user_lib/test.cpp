#include "test.h"

#include "hw/motor.h"
#include "hw/sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include <math.h>

#include <stdio.h>

namespace test
{
    namespace
    {
        constexpr uint32_t DISPLAY_PERIOD_MS = 200;

        // 双电机定速测试：0 仅显示，1 按校准正方向以 10 rad/s 转动
        constexpr uint32_t MOTOR_SPEED_TEST = 1;

        constexpr uint32_t SPEED_PERIOD_MS = 1;
        constexpr float TARGET_SPEED_RAD_S = 10.0f;
        constexpr float SPEED_KP = 0.2f;
        constexpr float SPEED_KI = 0.4f;
        constexpr float CURRENT_LIMIT_A = 0.7f;

        /**
         * @brief 按 v18 的斜坡、编码器门控和速度 PI 执行定速测试
         */
        void speed_task(void *)
        {
            float integral[2] = {};
            uint64_t previous[2] = {};
            uint64_t seen[2] = {};
            int64_t began_us = 0;
            TickType_t wake = xTaskGetTickCount();

            while(true)
            {
                vTaskDelayUntil(&wake, pdMS_TO_TICKS(SPEED_PERIOD_MS));

                motor::foc_info foc{};
                motor::get_info(foc);

                if(!foc.ready || foc.fault.active)
                {
                    began_us = 0;

                    for(uint32_t i = 0; i < 2; ++i)
                    {
                        integral[i] = 0.0f;
                        previous[i] = 0;
                        seen[i] = 0;
                    }
                    continue;
                }

                if(began_us == 0)
                {
                    began_us = esp_timer_get_time();
                }

                sensor::package sensors{};
                sensor::get_package(sensors);
                const sensor::encoder_data encoders[2] =
                {
                    sensors.left_encoder,
                    sensors.right_encoder
                };

                for(uint32_t i = 0; i < 2; ++i)
                {
                    const motor::current_info &current = foc.current[i];
                    const sensor::encoder_data &encoder = encoders[i];

                    if(!current.valid || current.encoder_direction == 0.0f ||
                       encoder.timestamp_us == 0 || encoder.timestamp_us == seen[i] ||
                       (previous[i] && current.timestamp_us - previous[i] < 1000))
                    {
                        continue;
                    }

                    const float t = static_cast<float>(
                        static_cast<int64_t>(current.timestamp_us) - began_us) * 1.0e-6f;
                    const float speed_ref = t < 0.1f ? 0.0f :
                        (t < 0.75f ? TARGET_SPEED_RAD_S * (t - 0.1f) / 0.65f :
                                      TARGET_SPEED_RAD_S);
                    const float dt = previous[i] ?
                        static_cast<float>(current.timestamp_us - previous[i]) * 1.0e-6f :
                        0.001f;
                    previous[i] = current.timestamp_us;
                    seen[i] = encoder.timestamp_us;

                    const float error = speed_ref -
                        current.encoder_direction * encoder.speed_rad_s;
                    const float candidate = integral[i] + SPEED_KI * dt * error;
                    const float output = SPEED_KP * error + candidate;

                    motor::target_current target{};
                    target.iq_A = fmaxf(
                        -CURRENT_LIMIT_A, fminf(CURRENT_LIMIT_A, output));

                    if(fabsf(output) <= CURRENT_LIMIT_A || output * error < 0.0f)
                    {
                        integral[i] = candidate;
                    }

                    motor::set_target_current(
                        static_cast<motor::motor_id>(i), target);
                }
            }
        }

        /**
         * @brief 显示传感器与电流环快照，不下发目标电流
         */
        void display_task(void *)
        {
            while(true)
            {
                sensor::package sensors{};
                const bool sensor_ready = sensor::get_package(sensors);

                motor::foc_info foc{};
                motor::get_info(foc);

                printf(
                    "ready=%u imu=%u bus=%.3f V fault=%lu motor=%u\n",
                    static_cast<unsigned>(foc.ready),
                    static_cast<unsigned>(sensor_ready),
                    foc.bus_voltage_V,
                    static_cast<unsigned long>(foc.fault.code),
                    static_cast<unsigned>(foc.fault.motor_index));

                for(uint32_t i = 0; i < 2; ++i)
                {
                    const motor::current_info &current = foc.current[i];

                    printf(
                        "M%u valid=%u speed=%+.3f rad/s "
                        "Id=%+.3f/%+.3f A Iq=%+.3f/%+.3f A\n",
                        static_cast<unsigned>(i + 1),
                        static_cast<unsigned>(current.valid),
                        current.mechanical_speed_rad_s,
                        current.id_A,
                        foc.target[i].id_A,
                        current.iq_A,
                        foc.target[i].iq_A);
                }

                printf("\n");

                vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
            }
        }
    }

    /**
     * @brief 启动快照显示及可选定速测试，电流环由 motor 模块负责
     *
     * @return true 任务创建成功
     * @return false 任务创建失败
     */
    bool init()
    {
        if(xTaskCreatePinnedToCore(
                display_task, "foc_test", 4096,
                nullptr, 1, nullptr, 0) != pdPASS)
        {
            return false;
        }

        if(MOTOR_SPEED_TEST == 1 &&
           xTaskCreatePinnedToCore(
               speed_task, "speed_test", 4096,
               nullptr, 2, nullptr, 0) != pdPASS)
        {
            return false;
        }

        return true;
    }
}
