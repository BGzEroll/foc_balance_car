#include "test.h"
#include "hw/sensor.h"
#include "foc_current.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace test
{
    namespace
    {
        constexpr uint32_t ROW_COUNT = 28;
        constexpr uint32_t ROW_SIZE = 320;

        // 仅由测试任务使用，放在静态区以减少任务栈占用。
        char previous[ROW_COUNT][ROW_SIZE] = {};
        char current[ROW_COUNT][ROW_SIZE] = {};
        char output[ROW_COUNT * (ROW_SIZE + 20) + 32] = {};
        bool first_draw = true;

        void draw(const sensor::package &p, bool ready, const foc_current::state &foc)
        {
            snprintf(current[0], ROW_SIZE, "平衡车实时数据    角度：rad    角速度：rad/s");
            const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
            snprintf(current[1], ROW_SIZE, "传感器状态：%s；数据年龄 ms：左=%.3f 右=%.3f IMU=%.3f；sensor 核心1 优先级5",
                     ready ? "已就绪" : "等待有效数据或静止校准中",
                     p.left_encoder.timestamp_us == 0 ? -1.0 : static_cast<double>(now_us - p.left_encoder.timestamp_us) * 0.001,
                     p.right_encoder.timestamp_us == 0 ? -1.0 : static_cast<double>(now_us - p.right_encoder.timestamp_us) * 0.001,
                     p.imu.timestamp_us == 0 ? -1.0 : static_cast<double>(now_us - p.imu.timestamp_us) * 0.001);

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

            snprintf(current[8], ROW_SIZE, "PVCC 母线电压：%7.3f V；FOC 无引脚输出，使用测试电角度", foc.bus_V);
            snprintf(current[9], ROW_SIZE, "FOC n=%" PRIu32 " hz=%" PRIu32 " adc_invalid=%" PRIu32 " cmp_error=%" PRIu32 " late=%" PRIu32 " gaps=%" PRIu32,
                     foc.count, foc_current::FREQUENCY_HZ, foc.adc_invalid, foc.compare_errors, foc.deadline_misses, foc.event_gaps);
            snprintf(current[10], ROW_SIZE, "相电流 A：M1 %+8.4f %+8.4f；M2 %+8.4f %+8.4f",
                     foc.current_A[0], foc.current_A[1], foc.current_A[2], foc.current_A[3]);
            const foc_current::timing *timings[5] = {&foc.acquisition, &foc.calculation, &foc.write, &foc.total, &foc.entry_phase};
            const char *names[5] = {"ADC 五路", "双电机 FOC", "比较器写入", "本轮执行总计", "零点至驱动入口"};
            for(uint32_t i = 0; i < 5; ++i)
            {
                const auto &t = *timings[i];
                snprintf(current[11 + i], ROW_SIZE, "%s us：mean=%.3f min=%.3f max=%.3f", names[i],
                    static_cast<double>(t.sum) / static_cast<double>(foc.count) / foc_current::CPU_MHZ,
                    static_cast<double>(t.minimum) / foc_current::CPU_MHZ,
                    static_cast<double>(t.maximum) / foc_current::CPU_MHZ);
            }
            snprintf(current[16], ROW_SIZE, "周期余量 us：mean=%.3f min=%.3f；入口间隔=%.3f..%.3f；周期=%.1f",
                     static_cast<double>(foc.margin.sum) / foc.count / foc_current::CPU_MHZ,
                     static_cast<double>(foc.margin.minimum) / foc_current::CPU_MHZ,
                     static_cast<double>(foc.interval_min) / foc_current::CPU_MHZ,
                     static_cast<double>(foc.interval_max) / foc_current::CPU_MHZ,
                     1000000.0 / foc_current::FREQUENCY_HZ);
            for(uint32_t i = 0; i < 5; ++i)
            {
                const double mean = static_cast<double>(foc.raw_sum[i]) / foc.count;
                const double variance = static_cast<double>(foc.raw_square_sum[i]) / foc.count - mean * mean;
                snprintf(current[17 + i], ROW_SIZE,
                    "ADC%" PRIu32 " raw=%" PRIu32 " mean=%.2f std=%.2f range=%" PRIu32 "..%" PRIu32 " base=%.2f spike=%" PRIu32 "；读数区间 us=%.2f..%.2f 最大结束=%.2f",
                    i, static_cast<uint32_t>(foc.raw[i]), mean, sqrt(variance > 0.0 ? variance : 0.0),
                    static_cast<uint32_t>(foc.raw_min[i]), static_cast<uint32_t>(foc.raw_max[i]), foc.baseline_raw[i],
                    foc.raw_spikes[i],
                    static_cast<double>(foc.read_begin[i].sum) / foc.count / foc_current::CPU_MHZ,
                    static_cast<double>(foc.read_end[i].sum) / foc.count / foc_current::CPU_MHZ,
                    static_cast<double>(foc.read_end[i].maximum) / foc_current::CPU_MHZ);
            }
            snprintf(current[22], ROW_SIZE, "内部占空比 M1=%.3f %.3f %.3f；M2=%.3f %.3f %.3f",
                     foc.duty[0], foc.duty[1], foc.duty[2], foc.duty[3], foc.duty[4], foc.duty[5]);
            if(foc.probe_completed)
            {
                snprintf(current[23], ROW_SIZE, "业务吞吐对照：无FOC=%" PRIu32 " 有FOC=%" PRIu32 " 剩余=%.2f%%；FPU上下文错误=%" PRIu32,
                         foc.probe_without_foc, foc.probe_with_foc,
                         static_cast<double>(foc.probe_with_foc) * 100.0 / foc.probe_without_foc, foc.probe_fpu_errors);
            }
            else
            {
                snprintf(current[23], ROW_SIZE, "业务吞吐探针未完成；FOC 和传感器均运行于核心1，显示独立运行于核心0");
            }
            snprintf(current[24], ROW_SIZE, "模式=%s；FOC 核心1 %s=%" PRIu32 "；触发=%" PRIu32 " 通知覆盖=%" PRIu32,
                     foc_current::RUN_IN_TASK ? "RTOS任务" : "中断",
                     foc_current::RUN_IN_TASK ? "任务优先级" : "中断等级",
                     foc_current::RUN_IN_TASK ? foc_current::TASK_PRIORITY : 1,
                     foc.trigger_count, foc.notification_skips);
            const foc_current::timing *extra[3] = {&foc.wake_latency, &foc.completion, &foc.notify_isr};
            const char *extra_names[3] = {"通知中断入口至任务执行", "零点至本轮完成", "通知中断开销"};
            for(uint32_t i = 0; i < 3; ++i)
            {
                const auto &t = *extra[i];
                snprintf(current[25 + i], ROW_SIZE, "%s us：mean=%.3f min=%.3f max=%.3f", extra_names[i],
                    static_cast<double>(t.sum) / static_cast<double>(foc.count) / foc_current::CPU_MHZ,
                    static_cast<double>(t.minimum) / foc_current::CPU_MHZ,
                    static_cast<double>(t.maximum) / foc_current::CPU_MHZ);
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
                size += snprintf(output + size, sizeof(output) - size, "\033[29;1H\n");
                fwrite(output, 1, size, stdout);
                fflush(stdout);
            }
        }

        void task(void *)
        {
            int64_t first_cycle_seen_us = 0;
            while(true)
            {
                sensor::package p{};
                const bool ready = sensor::get_package(p);
                foc_current::state foc{};
                foc_current::get_package(foc);
                if(foc.count != 0)
                {
                    const int64_t now_us = esp_timer_get_time();
                    if(first_cycle_seen_us == 0){first_cycle_seen_us = now_us;}
                    // 正常时等半秒吞吐探针结束再打印，过载时仍能独立显示诊断数据。
                    if(foc.probe_completed || now_us - first_cycle_seen_us >= 1000000)
                    {
                        draw(p, ready, foc);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        /**
         * @brief 先启动核心 0 观察任务，再在核心 1 初始化电流环
         */
        void start(void *)
        {
            gpio_set_level(GPIO_NUM_7, 1);
            gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
            vTaskDelay(pdMS_TO_TICKS(300));
            xTaskCreatePinnedToCore(task, "sensor_test", 6144, nullptr, 1, nullptr, 0);
            foc_current::init();
            vTaskDelete(nullptr);
        }
    }

    bool init()
    {
        return xTaskCreatePinnedToCore(start, "foc_start", 4096, nullptr, 1, nullptr, 1) == pdPASS;
    }
}
