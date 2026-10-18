#include "test.h"
#include "hw/sensor.h"
#include "hw/bat.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace test
{
    namespace
    {
        constexpr uint32_t ROW_COUNT = 9;
        constexpr uint32_t ROW_SIZE = 160;

        // 仅由测试任务使用，放在静态区以减少任务栈占用。
        char previous[ROW_COUNT][ROW_SIZE] = {};
        char current[ROW_COUNT][ROW_SIZE] = {};
        char output[ROW_COUNT * (ROW_SIZE + 20) + 32] = {};
        bool first_draw = true;

        void draw(const sensor::package &p, bool ready, const bat::state &battery)
        {
            snprintf(current[0], ROW_SIZE, "平衡车实时数据    角度：rad    角速度：rad/s");
            snprintf(current[1], ROW_SIZE, "传感器状态：%s", ready ? "已就绪" : "等待有效数据或静止校准中");

            if(p.left_encoder.timestamp_us != 0)
            {
                snprintf(current[2], ROW_SIZE, "左编码器 M1    角度：%+11.4f    速度：%+10.3f",
                         p.left_encoder.angle_rad, p.left_encoder.speed_rad_s);
            }
            else
            {
                snprintf(current[2], ROW_SIZE, "左编码器 M1    等待数据");
            }

            if(p.right_encoder.timestamp_us != 0)
            {
                snprintf(current[3], ROW_SIZE, "右编码器 M2    角度：%+11.4f    速度：%+10.3f",
                         p.right_encoder.angle_rad, p.right_encoder.speed_rad_s);
            }
            else
            {
                snprintf(current[3], ROW_SIZE, "右编码器 M2    等待数据");
            }

            if(ready)
            {
                snprintf(current[4], ROW_SIZE, "加速度 (g)       X：%+8.4f  Y：%+8.4f  Z：%+8.4f",
                         p.imu.acc[0], p.imu.acc[1], p.imu.acc[2]);
                snprintf(current[5], ROW_SIZE, "角速度 (rad/s)   X：%+8.4f  Y：%+8.4f  Z：%+8.4f",
                         p.imu.gyro[0], p.imu.gyro[1], p.imu.gyro[2]);
                snprintf(current[6], ROW_SIZE, "姿态 (rad)    俯仰：%+8.4f  横滚：%+8.4f  偏航：%+8.4f",
                         p.imu.angle[0], p.imu.angle[1], p.imu.angle[2]);
                snprintf(current[7], ROW_SIZE, "IMU 温度：%6.2f 摄氏度", p.imu.temperature);
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
                snprintf(current[8], ROW_SIZE, "PVCC 母线电压：%7.3f V", battery.voltage_V);
            }
            else
            {
                snprintf(current[8], ROW_SIZE, "PVCC 母线电压：采样无效");
            }

            uint32_t size = 0;
            if(first_draw)
            {
                // 首次清屏，后续使用光标定位，只重写发生变化的行。
                size += snprintf(output + size, sizeof(output) - size, "\033[2J");
            }
            for(uint32_t row = 0; row < ROW_COUNT; ++row)
            {
                if(first_draw || strcmp(current[row], previous[row]) != 0)
                {
                    size += snprintf(output + size, sizeof(output) - size,
                                     "\033[%" PRIu32 ";1H\033[2K%s", row + 1, current[row]);
                    strcpy(previous[row], current[row]);
                }
            }
            first_draw = false;

            if(size != 0)
            {
                // 光标停在面板下方；换行让串口监视器及时输出缓存的片段。
                size += snprintf(output + size, sizeof(output) - size, "\033[10;1H\n");
                fwrite(output, 1, size, stdout);
                fflush(stdout);
            }
        }

        void task(void *)
        {
            // 临时保持母线通路开启：GPIO7 拉高后保持输出，不需要循环重复写入。
            gpio_set_level(GPIO_NUM_7, 1);
            gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);

            bat::state battery{};
            bat::get_package(battery);
            uint32_t bat_count = 0;
            while(true)
            {
                sensor::package p{};
                const bool ready = sensor::get_package(p);
                if(++bat_count >= 10)
                {
                    bat::get_package(battery);
                    bat_count = 0;
                }
                draw(p, ready, battery);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }

    bool init()
    {
        return xTaskCreate(task, "sensor_test", 4096, nullptr, 1, nullptr) == pdPASS;
    }
}
