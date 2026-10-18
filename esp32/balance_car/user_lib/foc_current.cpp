#include "foc_current.h"

#include <math.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_private/adc_private.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace foc_current
{
    namespace
    {
        constexpr uint32_t RESOLUTION_HZ = 10000000;
        constexpr uint32_t PERIOD_TICKS = RESOLUTION_HZ / FREQUENCY_HZ;
        constexpr uint32_t PEAK_TICKS = PERIOD_TICKS / 2;
        constexpr float DT = 1.0f / static_cast<float>(FREQUENCY_HZ);
        constexpr float TWO_PI = 6.28318530718f;
        constexpr float INV_SQRT3 = 0.57735026919f;
        constexpr float SQRT3_HALF = 0.86602540378f;
        constexpr float CURRENT_GAIN = 4.0f; // 5mΩ × 50 倍，对应每伏 4A；实际方向留待通电标定。
        constexpr uint32_t BASELINE_COUNT = 512;
        constexpr adc_channel_t CHANNEL[5] = {
            ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_0};

        adc_oneshot_unit_handle_t adc = nullptr;
        mcpwm_timer_handle_t timer = nullptr;
        mcpwm_cmpr_handle_t comparator[6] = {};
        uint16_t voltage_mv[5][4096] = {};
        float offset_V[4] = {};
        float integral_d[2] = {};
        float integral_q[2] = {};
        float electrical_test_angle = 0.0f;
        uint32_t previous_cycle = 0;
        int64_t previous_event_us = 0;
        state latest;
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

        /**
         * @brief 用半秒普通任务吞吐量对照测量中断对业务代码的实际影响
         * @return 半秒内完成的循环次数，包含现有传感器任务的影响
         */
        uint32_t probe()
        {
            uint32_t count = 0;
            const int64_t deadline = esp_timer_get_time() + 500000;
            while(esp_timer_get_time() < deadline){++count;}
            return count;
        }

        /**
         * @brief 累计周期计数形式的耗时统计
         * @param[in,out] out 统计结果
         * @param[in] cycles 本次 CPU 周期数
         */
        inline void record(timing &out, uint32_t cycles)
        {
            out.sum += cycles;
            if(cycles > out.maximum){out.maximum = cycles;}
            if(cycles < out.minimum){out.minimum = cycles;}
        }

        /**
         * @brief 执行单电机完整电流环数学链路，输出仅写内部比较器
         * @param[in] motor 电机序号
         * @param[in] ia A 相电流，单位 A
         * @param[in] ib B 相电流，单位 A
         * @param[in] bus 母线电压，单位 V
         * @param[in] angle 测试电角度，单位 rad
         * @param[out] id d 轴电流，单位 A
         * @param[out] iq q 轴电流，单位 A
         * @param[out] duty 三相占空比
         */
        void calculate(uint32_t motor, float ia, float ib, float bus, float angle,
                       float &id, float &iq, float *duty)
        {
            const float sine = sinf(angle);
            const float cosine = cosf(angle);
            const float alpha = ia;
            const float beta = (ia + 2.0f * ib) * INV_SQRT3;
            id = alpha * cosine + beta * sine;
            iq = beta * cosine - alpha * sine;

            // 仅为无输出性能测试设置 PI 与参考值，不代表电机已完成参数整定。
            constexpr float KP = 0.5f;
            constexpr float KI = 20.0f;
            const float error_d = -id;
            const float error_q = 0.2f - iq;
            const float candidate_d = integral_d[motor] + KI * DT * error_d;
            const float candidate_q = integral_q[motor] + KI * DT * error_q;
            float vd = KP * error_d + candidate_d;
            float vq = KP * error_q + candidate_q;
            const float limit = bus * INV_SQRT3 * 0.90f;
            const float magnitude_squared = vd * vd + vq * vq;
            if(magnitude_squared > limit * limit)
            {
                const float scale = limit / sqrtf(magnitude_squared);
                vd *= scale;
                vq *= scale;
                // 矢量饱和时回算积分，避免积分继续增长。
                integral_d[motor] = vd - KP * error_d;
                integral_q[motor] = vq - KP * error_q;
            }
            else
            {
                integral_d[motor] = candidate_d;
                integral_q[motor] = candidate_q;
            }

            const float va = vd * cosine - vq * sine;
            const float vb = vd * sine + vq * cosine;
            const float phase[3] = {va, -0.5f * va + SQRT3_HALF * vb,
                                   -0.5f * va - SQRT3_HALF * vb};
            const float common = -0.5f * (fmaxf(phase[0], fmaxf(phase[1], phase[2])) +
                                          fminf(phase[0], fminf(phase[1], phase[2])));
            for(uint32_t i = 0; i < 3; ++i)
            {
                duty[i] = 0.5f + (phase[i] + common) / bus;
            }
        }

        /**
         * @brief 在一级 MCPWM 零点中断依次采样五路 ADC 并计算双电机电流环
         * @param[in] event 定时器进入驱动中断时的相位
         * @return false 不请求任务切换
         */
        bool IRAM_ATTR on_empty(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t *event, void *)
        {
            const uint32_t start = esp_cpu_get_cycle_count();
            const uint32_t phase_ticks = event->direction == MCPWM_TIMER_DIRECTION_UP ?
                event->count_value : PERIOD_TICKS - event->count_value;
            const uint32_t entry_cycles = phase_ticks * CPU_MHZ / 10;
            const int64_t event_us = esp_timer_get_time() - phase_ticks / 10;
            uint32_t begin[5];
            uint32_t end[5];
            uint16_t raw[5];
            uint32_t invalid = 0;

            // 该私有 API 固定依赖 ESP-IDF 6.0.2；此测试独占 ADC1，无其它 ADC 调用者。
            for(uint32_t i = 0; i < 5; ++i)
            {
                int value = 0; // IDF API 的输出参数要求 int*，不能以本平台的 int32_t* 代替。
                begin[i] = esp_cpu_get_cycle_count() - start;
                adc_oneshot_read_isr(adc, CHANNEL[i], &value);
                end[i] = esp_cpu_get_cycle_count() - start;
                if(value <= 0 || value >= 4095){++invalid;}
                raw[i] = static_cast<uint16_t>(value & 4095);
            }
            const uint32_t acquired = esp_cpu_get_cycle_count();
            float current[4];
            for(uint32_t i = 0; i < 4; ++i)
            {
                current[i] = (static_cast<float>(voltage_mv[i][raw[i]]) * 0.001f - offset_V[i]) * CURRENT_GAIN;
            }
            const float bus = static_cast<float>(voltage_mv[4][raw[4]]) * 0.003f;
            // 极对数和电角度零位尚未标定，采用连续扫角覆盖完整三角函数与 SVPWM 路径。
            electrical_test_angle += TWO_PI * 3.0f * DT;
            if(electrical_test_angle >= TWO_PI){electrical_test_angle -= TWO_PI;}
            float id[2];
            float iq[2];
            float duty[6];
            const float calculation_bus = fmaxf(bus, 1.0f);
            calculate(0, current[0], current[1], calculation_bus, electrical_test_angle, id[0], iq[0], duty);
            calculate(1, current[2], current[3], calculation_bus, electrical_test_angle + 1.0f, id[1], iq[1], duty + 3);
            const uint32_t calculated = esp_cpu_get_cycle_count();
            uint32_t errors = 0;
            for(uint32_t i = 0; i < 6; ++i)
            {
                errors += mcpwm_comparator_set_compare_value(comparator[i],
                    static_cast<uint32_t>(duty[i] * static_cast<float>(PEAK_TICKS))) != ESP_OK;
            }
            const uint32_t written = esp_cpu_get_cycle_count();

            portENTER_CRITICAL_ISR(&mux);
            ++latest.count;
            latest.adc_invalid += invalid;
            latest.compare_errors += errors;
            if(previous_cycle != 0)
            {
                const uint32_t interval = start - previous_cycle;
                if(interval < latest.interval_min){latest.interval_min = interval;}
                if(interval > latest.interval_max){latest.interval_max = interval;}
                const int64_t periods = (event_us - previous_event_us + 500000 / FREQUENCY_HZ) /
                                         (1000000 / FREQUENCY_HZ);
                if(periods > 1){latest.event_gaps += static_cast<uint32_t>(periods - 1);}
            }
            previous_cycle = start;
            previous_event_us = event_us;
            record(latest.entry_phase, entry_cycles);
            record(latest.acquisition, acquired - start);
            record(latest.calculation, calculated - acquired);
            record(latest.write, written - calculated);
            for(uint32_t i = 0; i < 5; ++i)
            {
                latest.raw[i] = raw[i];
                latest.raw_sum[i] += raw[i];
                if(raw[i] < latest.raw_min[i]){latest.raw_min[i] = raw[i];}
                if(raw[i] > latest.raw_max[i]){latest.raw_max[i] = raw[i];}
                record(latest.read_begin[i], entry_cycles + begin[i]);
                record(latest.read_end[i], entry_cycles + end[i]);
            }
            for(uint32_t i = 0; i < 4; ++i){latest.current_A[i] = current[i];}
            for(uint32_t i = 0; i < 2; ++i){latest.id_A[i] = id[i]; latest.iq_A[i] = iq[i];}
            for(uint32_t i = 0; i < 6; ++i){latest.duty[i] = duty[i];}
            latest.bus_V = bus;
            const uint32_t elapsed = esp_cpu_get_cycle_count() - start;
            record(latest.total, elapsed);
            if(entry_cycles + elapsed >= PERIOD_CYCLES){++latest.deadline_misses;}
            portEXIT_CRITICAL_ISR(&mux);
            return false;
        }

        /**
         * @brief 初始化唯一 ADC1 实例、校准查找表及无电流输出时的零点
         */
        void init_adc()
        {
            adc_oneshot_unit_init_cfg_t unit{};
            unit.unit_id = ADC_UNIT_1;
            ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
            adc_oneshot_chan_cfg_t channel{};
            channel.atten = ADC_ATTEN_DB_12;
            channel.bitwidth = ADC_BITWIDTH_12;
            for(uint32_t i = 0; i < 5; ++i)
            {
                ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, CHANNEL[i], &channel));
                adc_cali_curve_fitting_config_t config{};
                config.unit_id = ADC_UNIT_1;
                config.chan = CHANNEL[i];
                config.atten = ADC_ATTEN_DB_12;
                config.bitwidth = ADC_BITWIDTH_12;
                adc_cali_handle_t calibration = nullptr;
                ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&config, &calibration));
                for(uint32_t raw = 0; raw < 4096; ++raw)
                {
                    int mv = 0;
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, raw, &mv));
                    voltage_mv[i][raw] = static_cast<uint16_t>(mv < 0 ? 0 : mv);
                }
                ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(calibration));
            }
            for(uint32_t sample = 0; sample < BASELINE_COUNT; ++sample)
            {
                for(uint32_t i = 0; i < 5; ++i)
                {
                    int raw = 0;
                    ESP_ERROR_CHECK(adc_oneshot_read(adc, CHANNEL[i], &raw));
                    latest.baseline_raw[i] += static_cast<float>(raw) / static_cast<float>(BASELINE_COUNT);
                    if(i < 4)
                    {
                        offset_V[i] += static_cast<float>(voltage_mv[i][raw]) *
                                       (0.001f / static_cast<float>(BASELINE_COUNT));
                    }
                }
            }
        }
    }

    /**
     * @brief 在当前核心初始化无引脚输出的 MCPWM 中断电流环测试
     * @return true 初始化完成，任何配置失败由 ESP_ERROR_CHECK 报告
     */
    bool init()
    {
        // 六个控制脚保持输入下拉，不创建 PWM 发生器，不连接 GPIO 矩阵。
        gpio_config_t pins{};
        pins.pin_bit_mask = (1ULL << 35) | (1ULL << 34) | (1ULL << 33) |
                            (1ULL << 12) | (1ULL << 11) | (1ULL << 10);
        pins.mode = GPIO_MODE_INPUT;
        pins.pull_down_en = GPIO_PULLDOWN_ENABLE;
        ESP_ERROR_CHECK(gpio_config(&pins));
        ESP_ERROR_CHECK(gpio_dump_io_configuration(stdout, pins.pin_bit_mask));
        init_adc();
        vTaskDelay(pdMS_TO_TICKS(2500));
        latest.probe_without_foc = probe();
        mcpwm_timer_config_t config{};
        config.group_id = 0;
        config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        config.resolution_hz = RESOLUTION_HZ;
        config.period_ticks = PERIOD_TICKS;
        config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
        config.intr_priority = 1;
        ESP_ERROR_CHECK(mcpwm_new_timer(&config, &timer));
        for(uint32_t i = 0; i < 3; ++i)
        {
            mcpwm_oper_handle_t oper = nullptr;
            mcpwm_operator_config_t oper_config{};
            oper_config.group_id = 0;
            oper_config.intr_priority = 1;
            ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
            ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
            for(uint32_t j = 0; j < 2; ++j)
            {
                mcpwm_comparator_config_t cmp_config{};
                cmp_config.flags.update_cmp_on_tez = true;
                ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &comparator[i * 2 + j]));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i * 2 + j], PEAK_TICKS / 2));
            }
        }
        mcpwm_timer_event_callbacks_t callbacks{};
        callbacks.on_empty = on_empty;
        ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &callbacks, nullptr));
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
        const uint32_t with_foc = probe();
        portENTER_CRITICAL(&mux);
        latest.probe_with_foc = with_foc;
        portEXIT_CRITICAL(&mux);
        return true;
    }

    /**
     * @brief 复制累计测量结果，中断只在此短临界区可能等待
     * @param[out] out ADC 数据与周期统计
     */
    void get_package(state &out)
    {
        portENTER_CRITICAL(&mux);
        out = latest;
        portEXIT_CRITICAL(&mux);
    }
}
