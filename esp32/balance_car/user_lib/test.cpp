#include "test.h"
#include "hw/sensor.h"
#include "hw/bat.h"
#include "driver/gpio.h"
#include "esp_cpu.h"
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
        constexpr uint32_t ROW_COUNT = 16;
        constexpr uint32_t ROW_SIZE = 160;

        // 仅由测试任务使用，放在静态区以减少任务栈占用。
        char previous[ROW_COUNT][ROW_SIZE] = {};
        char current[ROW_COUNT][ROW_SIZE] = {};
        char output[ROW_COUNT * (ROW_SIZE + 20) + 32] = {};
        bool first_draw = true;

        constexpr uint32_t FPU_ITERATIONS = 50000;
        constexpr uint32_t FPU_ROUNDS = 7;

        struct benchmark_result
        {
            uint32_t integer_cycles = 0;
            uint32_t float_cycles = 0;
            uint32_t double_cycles = 0;
            uint32_t float_min = 0;
            uint32_t float_max = 0;
            uint32_t double_min = 0;
            uint32_t double_max = 0;
            uint32_t integer_value = 0;
            float float_value = 0.0f;
            double double_value = 0.0;
        };

        benchmark_result benchmark;

        /**
         * @brief 测试单精度浮点乘加循环，保留独立函数以便核对汇编
         *
         * @param[in] value 初始值
         * @param[in] iterations 迭代次数
         *
         * @return 循环结束后的单精度浮点数
         */
        __attribute__((noinline, noclone)) float kernel_float(float value, uint32_t iterations)
        {
            for(uint32_t i = 0; i < iterations; ++i)
            {
                value = value * 0.99991f + 0.00013f;
            }
            return value;
        }

        /**
         * @brief 使用相同公式测试双精度浮点乘加循环
         *
         * @param[in] value 初始值
         * @param[in] iterations 迭代次数
         *
         * @return 循环结束后的双精度浮点数
         */
        __attribute__((noinline, noclone)) double kernel_double(double value, uint32_t iterations)
        {
            for(uint32_t i = 0; i < iterations; ++i)
            {
                value = value * 0.99991 + 0.00013;
            }
            return value;
        }

        /**
         * @brief 测试无符号整数乘加基线，与浮点公式不具有数值等价性
         *
         * @param[in] value 初始值
         * @param[in] iterations 迭代次数
         *
         * @return 按无符号整数规则回绕后的计算结果
         */
        __attribute__((noinline, noclone)) uint32_t kernel_integer(uint32_t value, uint32_t iterations)
        {
            for(uint32_t i = 0; i < iterations; ++i)
            {
                value = value * 1664525U + 1013904223U;
            }
            return value;
        }

        /**
         * @brief 对测速样本排序并取中位数
         *
         * @param[in,out] samples 七轮周期计数，返回时按升序排列
         *
         * @return 周期计数中位数
         */
        uint32_t median_cycles(uint32_t samples[FPU_ROUNDS])
        {
            for(uint32_t i = 1; i < FPU_ROUNDS; ++i)
            {
                const uint32_t value = samples[i];
                uint32_t j = i;
                while(j > 0 && samples[j - 1] > value)
                {
                    samples[j] = samples[j - 1];
                    --j;
                }
                samples[j] = value;
            }
            return samples[FPU_ROUNDS / 2];
        }

        /**
         * @brief 在测试任务中执行一次测速，首轮预热后取七轮中位数
         */
        void run_benchmark()
        {
            uint32_t integer_cycles[FPU_ROUNDS] = {};
            uint32_t float_cycles[FPU_ROUNDS] = {};
            uint32_t double_cycles[FPU_ROUNDS] = {};

            // 保留中断和调度，结果包含实际运行干扰，不代表单条指令的延迟。
            for(uint32_t round = 0; round <= FPU_ROUNDS; ++round)
            {
                uint32_t begin = esp_cpu_get_cycle_count();
                asm volatile("" ::: "memory");
                benchmark.integer_value = kernel_integer(1U, FPU_ITERATIONS);
                asm volatile("" ::: "memory");
                const uint32_t integer_elapsed = esp_cpu_get_cycle_count() - begin;

                begin = esp_cpu_get_cycle_count();
                asm volatile("" ::: "memory");
                benchmark.float_value = kernel_float(0.5f, FPU_ITERATIONS);
                asm volatile("" ::: "memory");
                const uint32_t float_elapsed = esp_cpu_get_cycle_count() - begin;

                begin = esp_cpu_get_cycle_count();
                asm volatile("" ::: "memory");
                benchmark.double_value = kernel_double(0.5, FPU_ITERATIONS);
                asm volatile("" ::: "memory");
                const uint32_t double_elapsed = esp_cpu_get_cycle_count() - begin;

                if(round != 0)
                {
                    integer_cycles[round - 1] = integer_elapsed;
                    float_cycles[round - 1] = float_elapsed;
                    double_cycles[round - 1] = double_elapsed;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            benchmark.integer_cycles = median_cycles(integer_cycles);
            benchmark.float_cycles = median_cycles(float_cycles);
            benchmark.double_cycles = median_cycles(double_cycles);
            benchmark.float_min = float_cycles[0];
            benchmark.float_max = float_cycles[FPU_ROUNDS - 1];
            benchmark.double_min = double_cycles[0];
            benchmark.double_max = double_cycles[FPU_ROUNDS - 1];
        }

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

            snprintf(current[9], ROW_SIZE, "FPU 测速：CPU=%" PRIu32 " MHz，每组 %" PRIu32 " 次，7 轮中位数",
                     static_cast<uint32_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ), FPU_ITERATIONS);
            snprintf(current[10], ROW_SIZE, "整数乘加：%" PRIu32 " 周期，%.3f 周期/次",
                     benchmark.integer_cycles, static_cast<float>(benchmark.integer_cycles) / FPU_ITERATIONS);
            snprintf(current[11], ROW_SIZE, "float ：%" PRIu32 " 周期，%.3f 周期/次，范围 %" PRIu32 "~%" PRIu32,
                     benchmark.float_cycles, static_cast<float>(benchmark.float_cycles) / FPU_ITERATIONS,
                     benchmark.float_min, benchmark.float_max);
            snprintf(current[12], ROW_SIZE, "double：%" PRIu32 " 周期，%.3f 周期/次，范围 %" PRIu32 "~%" PRIu32,
                     benchmark.double_cycles, static_cast<float>(benchmark.double_cycles) / FPU_ITERATIONS,
                     benchmark.double_min, benchmark.double_max);
            snprintf(current[13], ROW_SIZE, "本循环 double / float 耗时比：%.2f 倍",
                     static_cast<float>(benchmark.double_cycles) / benchmark.float_cycles);
            snprintf(current[14], ROW_SIZE, "结果校验：整数=%" PRIu32 "，float=%.7f，double=%.7f",
                     benchmark.integer_value, benchmark.float_value, benchmark.double_value);
            snprintf(current[15], ROW_SIZE, "同为乘加循环；整数仅作基线，周期包含中断和循环开销");

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
                size += snprintf(output + size, sizeof(output) - size, "\033[17;1H\n");
                fwrite(output, 1, size, stdout);
                fflush(stdout);
            }
        }

        void task(void *)
        {
            // 临时保持母线通路开启：GPIO7 拉高后保持输出，不需要循环重复写入。
            gpio_set_level(GPIO_NUM_7, 1);
            gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);

            run_benchmark();

            bat::state battery{};
            bat::get_package(battery);
            uint32_t bat_count = 0;
            while(true)
            {
                sensor::package p{};
                const bool ready = sensor::get_package(p);
                if(++bat_count >= 1)
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
        // 固定到核心 0，避免测速期间任务迁核导致周期计数失去可比性。
        return xTaskCreatePinnedToCore(task, "sensor_test", 4096, nullptr, 1, nullptr, 0) == pdPASS;
    }
}
