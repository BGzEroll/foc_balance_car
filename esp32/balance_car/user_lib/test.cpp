
#include "test.h"

#include "hw/sensor.h"
#include "hw/bat.h"
#include "sys_time.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

namespace test
{
    namespace
    {
        constexpr uint32_t ROW_COUNT = 9;
        constexpr uint32_t ROW_SIZE = 160;

        constexpr uint32_t SENSOR_STACK_SIZE = 4096;
        constexpr uint32_t DISPLAY_STACK_SIZE = 4096;

        constexpr UBaseType_t SENSOR_PRIORITY = 5;
        constexpr UBaseType_t DISPLAY_PRIORITY = 1;

        constexpr BaseType_t SENSOR_CORE = 1;
        constexpr BaseType_t DISPLAY_CORE = 0;

        constexpr uint32_t SENSOR_WAIT_MS = 1;
        constexpr uint32_t DISPLAY_PERIOD_MS = 100;

        // 仅由显示任务使用，放在静态区以减少任务栈占用。
        char previous[ROW_COUNT][ROW_SIZE] = {};
        char current[ROW_COUNT][ROW_SIZE] = {};
        char output[ROW_COUNT * (ROW_SIZE + 20) + 32] = {};
        bool first_draw = true;

        /**
         * @brief 计算数据距当前时刻的年龄
         *
         * @param[in] now_us 当前时间，单位 us
         * @param[in] timestamp_us 数据时间戳，单位 us
         *
         * @return 数据年龄，单位 ms；无效时间戳返回 -1
         */
        float get_age_ms(uint64_t now_us, uint64_t timestamp_us)
        {
            if(timestamp_us == 0 || now_us < timestamp_us)
            {
                return -1.0f;
            }

            return static_cast<float>(now_us - timestamp_us) * 0.001f;
        }

        /**
         * @brief 处理传感器事件并持续推进异步采样
         *
         * Sensor 的初始化、轮询及 I2C 总线恢复均由本任务负责。
         */
        void sensor_task(void *)
        {
            if(!sensor::init())
            {
                printf("Sensor initialization failed.\n");
                vTaskDelete(nullptr);
                return;
            }

            while(true)
            {
                uint32_t notifications = 0;

                xTaskNotifyWait(
                    0,
                    UINT32_MAX,
                    &notifications,
                    pdMS_TO_TICKS(SENSOR_WAIT_MS));

                // 电机尚未接入，当前允许执行 I2C 总线恢复。
                sensor::poll(notifications, true);
            }
        }

        /**
         * @brief 绘制传感器与电池实时数据
         *
         * @param[in] p 传感器快照
         * @param[in] ready IMU 是否已发布有效数据
         * @param[in] battery 电池状态
         */
        void draw(
            const sensor::package &p,
            bool ready,
            const bat::state &battery)
        {
            const uint64_t now_us = sys_time::get_us_tick();

            snprintf(
                current[0],
                ROW_SIZE,
                "平衡车实时数据    角度：rad    角速度：rad/s");

            if(ready)
            {
                snprintf(
                    current[1],
                    ROW_SIZE,
                    "传感器状态：已就绪    IMU 数据年龄：%7.1f ms",
                    get_age_ms(now_us, p.imu.timestamp_us));
            }
            else
            {
                snprintf(
                    current[1],
                    ROW_SIZE,
                    "传感器状态：等待有效 IMU 数据或静止校准中");
            }

            if(p.left_encoder.timestamp_us != 0)
            {
                snprintf(
                    current[2],
                    ROW_SIZE,
                    "左编码器 M1  角度：%+11.4f  速度：%+10.3f  年龄：%6.1f ms",
                    p.left_encoder.angle_rad,
                    p.left_encoder.speed_rad_s,
                    get_age_ms(now_us, p.left_encoder.timestamp_us));
            }
            else
            {
                snprintf(
                    current[2],
                    ROW_SIZE,
                    "左编码器 M1    等待数据");
            }

            if(p.right_encoder.timestamp_us != 0)
            {
                snprintf(
                    current[3],
                    ROW_SIZE,
                    "右编码器 M2  角度：%+11.4f  速度：%+10.3f  年龄：%6.1f ms",
                    p.right_encoder.angle_rad,
                    p.right_encoder.speed_rad_s,
                    get_age_ms(now_us, p.right_encoder.timestamp_us));
            }
            else
            {
                snprintf(
                    current[3],
                    ROW_SIZE,
                    "右编码器 M2    等待数据");
            }

            if(ready)
            {
                snprintf(
                    current[4],
                    ROW_SIZE,
                    "加速度 (g)       X：%+8.4f  Y：%+8.4f  Z：%+8.4f",
                    p.imu.acc[0],
                    p.imu.acc[1],
                    p.imu.acc[2]);

                snprintf(
                    current[5],
                    ROW_SIZE,
                    "角速度 (rad/s)   X：%+8.4f  Y：%+8.4f  Z：%+8.4f",
                    p.imu.gyro[0],
                    p.imu.gyro[1],
                    p.imu.gyro[2]);

                snprintf(
                    current[6],
                    ROW_SIZE,
                    "姿态 (rad)    俯仰：%+8.4f  横滚：%+8.4f  偏航：%+8.4f",
                    p.imu.angle[0],
                    p.imu.angle[1],
                    p.imu.angle[2]);

                snprintf(
                    current[7],
                    ROW_SIZE,
                    "IMU 温度：%6.2f 摄氏度",
                    p.imu.temperature);
            }
            else
            {
                snprintf(current[4], ROW_SIZE, "加速度 (g)       等待有效数据");
                snprintf(current[5], ROW_SIZE, "角速度 (rad/s)   等待有效数据");
                snprintf(current[6], ROW_SIZE, "姿态 (rad)       等待有效数据");
                snprintf(current[7], ROW_SIZE, "IMU 温度：等待有效数据");
            }

            if(battery.valid)
            {
                snprintf(
                    current[8],
                    ROW_SIZE,
                    "PVCC 母线电压：%7.3f V",
                    battery.voltage_V);
            }
            else
            {
                snprintf(
                    current[8],
                    ROW_SIZE,
                    "PVCC 母线电压：采样无效");
            }

            uint32_t size = 0;

            if(first_draw)
            {
                size += snprintf(
                    output + size,
                    sizeof(output) - size,
                    "\033[2J");
            }

            for(uint32_t row = 0; row < ROW_COUNT; ++row)
            {
                if(first_draw ||
                   strcmp(current[row], previous[row]) != 0)
                {
                    size += snprintf(
                        output + size,
                        sizeof(output) - size,
                        "\033[%" PRIu32 ";1H\033[2K%s",
                        row + 1,
                        current[row]);

                    strcpy(previous[row], current[row]);
                }
            }

            first_draw = false;

            if(size != 0)
            {
                size += snprintf(
                    output + size,
                    sizeof(output) - size,
                    "\033[10;1H\n");

                fwrite(output, 1, size, stdout);
                fflush(stdout);
            }
        }

        /**
         * @brief 串口数据显示任务
         *
         * 仅读取传感器快照，不直接操作 Sensor 的 I2C 状态机。
         */
        void display_task(void *)
        {
            gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
            gpio_set_level(GPIO_NUM_7, 1);

            bat::state battery{};
            bat::get_package(battery);

            while(true)
            {
                sensor::package p{};
                const bool ready = sensor::get_package(p);

                bat::get_package(battery);

                draw(p, ready, battery);

                vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
            }
        }
    }

    /**
     * @brief 启动传感器轮询与数据显示任务
     *
     * @return true 两个任务均创建成功
     * @return false 至少一个任务创建失败
     */
    bool init()
    {
        if(xTaskCreatePinnedToCore(
                sensor_task,
                "sensor_poll",
                SENSOR_STACK_SIZE,
                nullptr,
                SENSOR_PRIORITY,
                nullptr,
                SENSOR_CORE) != pdPASS)
        {
            return false;
        }

        if(xTaskCreatePinnedToCore(
                display_task,
                "sensor_test",
                DISPLAY_STACK_SIZE,
                nullptr,
                DISPLAY_PRIORITY,
                nullptr,
                DISPLAY_CORE) != pdPASS)
        {
            return false;
        }

        return true;
    }
}
