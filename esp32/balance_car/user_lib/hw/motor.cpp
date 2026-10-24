#include "motor.h"

#include "sensor.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "hal/adc_ll.h"
#pragma GCC diagnostic pop
#include "soc/mcpwm_struct.h"
#include "soc/sens_reg.h"

namespace adc
{
    namespace
    {
        constexpr uint32_t CPU_MHZ = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        constexpr uint32_t BASELINE_COUNT = 512;
        constexpr adc_channel_t CHANNEL[5] =
        {
            ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
            ADC_CHANNEL_4, ADC_CHANNEL_0
        };

        adc_oneshot_unit_handle_t handle = nullptr;
        uint16_t voltage_mv[4096] = {};
        float offset_V[4] = {};
        uint16_t current_low[4] = {};
        uint16_t current_high[4] = {};
        uint16_t bus_low = 0;
        uint16_t bus_high = 4095;
    }

    /**
     * @brief 初始化 ADC1、校准查找表和四路零电流偏置
     *
     * @note 初始化时电机 PWM 必须保持关闭
     */
    void init()
    {
        adc_oneshot_unit_init_cfg_t unit{};
        unit.unit_id = ADC_UNIT_1;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &handle));

        adc_oneshot_chan_cfg_t config{};
        config.atten = ADC_ATTEN_DB_12;
        config.bitwidth = ADC_BITWIDTH_12;

        for(uint32_t ch = 0; ch < 5; ++ch)
        {
            ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, CHANNEL[ch], &config));
        }

        // ESP32-S3 的 ADC1 通道在相同衰减和位宽下共用电压校准表
        adc_cali_curve_fitting_config_t cal_config{};
        cal_config.unit_id = ADC_UNIT_1;
        cal_config.chan = CHANNEL[0];
        cal_config.atten = ADC_ATTEN_DB_12;
        cal_config.bitwidth = ADC_BITWIDTH_12;

        adc_cali_handle_t calibration = nullptr;
        ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cal_config, &calibration));

        for(uint32_t raw = 0; raw < 4096; ++raw)
        {
            int mv = 0;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, raw, &mv));
            voltage_mv[raw] = static_cast<uint16_t>(mv < 0 ? 0 : mv);
        }

        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(calibration));

        for(uint32_t n = 0; n < BASELINE_COUNT; ++n)
        {
            for(uint32_t ch = 0; ch < 4; ++ch)
            {
                int raw = 0;
                ESP_ERROR_CHECK(adc_oneshot_read(handle, CHANNEL[ch], &raw));
                offset_V[ch] += voltage_mv[raw] * (0.001f / BASELINE_COUNT);
            }
        }

        for(uint32_t ch = 0; ch < 4; ++ch)
        {
            for(uint32_t raw = 0; raw < 4096; ++raw)
            {
                const float current =
                    (voltage_mv[raw] * 0.001f - offset_V[ch]) * 4.0f;

                // ADC 过流保护
                if(current < -6.0f)
                {
                    current_low[ch] = raw;
                }

                if(current <= 6.0f)
                {
                    current_high[ch] = raw;
                }
            }
        }

        for(uint32_t raw = 0; raw < 4096; ++raw)
        {
            const float voltage = voltage_mv[raw] * 0.003f;

            if(voltage < 5.0f)
            {
                bus_low = raw;
            }

            if(voltage <= 13.0f)
            {
                bus_high = raw;
            }
        }
    }

    /**
     * @brief 在 MCPWM ISR 内采集一个 ADC1 通道
     *
     * @param[in] channel ADC 通道
     * @param[out] raw 原始 ADC 值
     *
     * @return true 转换完成
     * @return false ADC 忙或转换超时
     */
    bool IRAM_ATTR read(adc_channel_t channel, uint16_t &raw)
    {
        const uint32_t idle = esp_cpu_get_cycle_count();

        while(SENS.sar_slave_addr1.meas_status != 0)
        {
            if(esp_cpu_get_cycle_count() - idle >= CPU_MHZ * 20 &&
               SENS.sar_slave_addr1.meas_status != 0)
            {
                return false;
            }
        }

        adc_oneshot_ll_set_channel(ADC_UNIT_1, channel);
        adc_oneshot_ll_start(ADC_UNIT_1);

        const uint32_t start = esp_cpu_get_cycle_count();

        while(!adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE))
        {
            if(esp_cpu_get_cycle_count() - start >= CPU_MHZ * 20 &&
               !adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE))
            {
                return false;
            }
        }

        raw = static_cast<uint16_t>(adc_oneshot_ll_get_raw_result(ADC_UNIT_1));
        return true;
    }

    /**
     * @brief 在 ISR 内检查两相电流的原始值是否超限
     *
     * @param[in] index 电机索引
     * @param[in] a A 相原始采样
     * @param[in] b B 相原始采样
     *
     * @return true 原始采样超出 ±2 A 对应的 ADC 范围
     */
    bool IRAM_ATTR overcurrent(uint32_t index, uint16_t a, uint16_t b)
    {
        const uint32_t ch = index * 2;
        return a < current_low[ch] || a > current_high[ch] ||
               b < current_low[ch + 1] || b > current_high[ch + 1];
    }

    /**
     * @brief 判断母线采样是否在 5–13 V 范围内
     *
     * @param[in] raw ADC 原始值
     *
     * @return true 在范围内
     */
    bool IRAM_ATTR valid_bus(uint16_t raw)
    {
        return raw >= bus_low && raw <= bus_high;
    }

    /**
     * @brief 转换一相电流
     *
     * @param[in] channel 电流 ADC 通道索引 0–3
     * @param[in] raw ADC 原始值
     * @param[in] index 电机索引
     *
     * @return 测量电流，单位 A
     */
    float current(uint32_t channel, uint16_t raw, uint32_t index)
    {
        constexpr float GAIN[2] = {-4.0f, -4.0f};
        return (voltage_mv[raw] * 0.001f - offset_V[channel]) * GAIN[index];
    }

    /**
     * @brief 转换母线电压
     *
     * @param[in] raw ADC 原始值
     *
     * @return 母线电压，单位 V
     */
    float voltage(uint16_t raw)
    {
        return voltage_mv[raw] * 0.003f;
    }


}

namespace pwm
{
    constexpr uint32_t PERIOD = 8000;
    constexpr uint32_t CYCLES_PER_TICK = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ / 80;
    constexpr uint32_t NEUTRAL = 2000;
    constexpr uint32_t SAMPLE_TIMEOUT_US = 1000;
    constexpr uint32_t BUS_TIMEOUT_US = 5000;

    struct sample
    {
        uint32_t timestamp_us = 0;
        uint32_t cycle = 0;
        uint32_t sequence = 0;
        uint32_t raw_a = 0;
        uint32_t raw_b = 0;
        uint32_t raw_bus = 0;
        uint32_t duty_a = NEUTRAL;
        uint32_t duty_b = NEUTRAL;
        uint32_t duty_c = NEUTRAL;
        uint32_t a_start = 0;
        uint32_t a_end = 0;
        uint32_t b_start = 0;
        uint32_t b_end = 0;
        uint32_t flags = 0;
    };

    struct command
    {
        uint32_t generation = 0;
        uint32_t sample_us = 0;
        uint32_t duty_a = NEUTRAL;
        uint32_t duty_b = NEUTRAL;
        uint32_t duty_c = NEUTRAL;
    };

    namespace
    {
        constexpr int32_t PINS[6] = {35, 34, 33, 12, 11, 10};

        template<typename T>
        struct mailbox
        {
            uint32_t version = 0;
            T value{};
        };

        static_assert(sizeof(sample) % sizeof(uint32_t) == 0);
        static_assert(sizeof(command) % sizeof(uint32_t) == 0);

        mcpwm_timer_handle_t timer = nullptr;
        mcpwm_timer_handle_t sample_timer = nullptr;
        mcpwm_sync_handle_t sync = nullptr;
        mcpwm_oper_handle_t operators[3] = {};
        mcpwm_cmpr_handle_t comparators[6] = {};
        mcpwm_gen_handle_t generators[6] = {};

        mailbox<sample> samples[2] = {};
        mailbox<command> commands[2] = {};

        uint32_t queued[6] =
        {
            NEUTRAL, NEUTRAL, NEUTRAL,
            NEUTRAL, NEUTRAL, NEUTRAL
        };
        uint32_t applied[6] =
        {
            NEUTRAL, NEUTRAL, NEUTRAL,
            NEUTRAL, NEUTRAL, NEUTRAL
        };

        uint32_t wanted = 0;
        uint32_t enabled = 0;
        uint32_t pending = 0;
        uint32_t mask = 0;
        uint32_t generation = 0;
        uint32_t cycle = 0;
        uint32_t enable_cycle = 0;
        uint32_t heartbeat_us = 0;
        uint32_t last_valid_sample_us[2] = {};
        uint32_t bus_raw = 0;
        uint32_t bus_us = 0;

        uint32_t fault = 0;
        uint32_t fault_motor = 0xFF;
        uint32_t fault_raw_a = 0;
        uint32_t fault_raw_b = 0;
        uint32_t fault_us = 0;

        /**
         * @brief 获取当前 32 位微秒时间
         *
         * @return 当前微秒计数
         */
        uint32_t IRAM_ATTR now32()
        {
            return static_cast<uint32_t>(esp_timer_get_time());
        }

        /**
         * @brief 读取主 PWM 定时器的等效相位
         *
         * @return 周期内相位计数
         */
        uint32_t IRAM_ATTR phase()
        {
            mcpwm_timer_status_reg_t status{};
            status.val = MCPWM0.timer[0].timer_status.val;
            const uint32_t value = status.timer_direction
                ? PERIOD - status.timer_value
                : status.timer_value;
            return value % PERIOD;
        }

        /**
         * @brief 向跨核邮箱写入完整数据
         *
         * @param[in, out] box 邮箱
         * @param[in] value 新数据
         */
        template<typename T>
        void IRAM_ATTR publish(mailbox<T> &box, const T &value)
        {
            __atomic_add_fetch(&box.version, 1U, __ATOMIC_ACQ_REL);
            uint32_t *dst = reinterpret_cast<uint32_t *>(&box.value);
            const uint32_t *src = reinterpret_cast<const uint32_t *>(&value);

            for(uint32_t i = 0; i < sizeof(T) / sizeof(uint32_t); ++i)
            {
                __atomic_store_n(dst + i, src[i], __ATOMIC_RELAXED);
            }

            __atomic_add_fetch(&box.version, 1U, __ATOMIC_RELEASE);
        }

        /**
         * @brief 从跨核邮箱读取一致的快照
         *
         * @param[in] box 邮箱
         * @param[out] value 结果
         *
         * @return true 取得一致数据
         */
        template<typename T>
        bool IRAM_ATTR consume(const mailbox<T> &box, T &value)
        {
            for(uint32_t attempt = 0; attempt < 3; ++attempt)
            {
                const uint32_t version =
                    __atomic_load_n(&box.version, __ATOMIC_ACQUIRE);

                if(version & 1U)
                {
                    continue;
                }

                uint32_t *dst = reinterpret_cast<uint32_t *>(&value);
                const uint32_t *src = reinterpret_cast<const uint32_t *>(&box.value);

                for(uint32_t i = 0; i < sizeof(T) / sizeof(uint32_t); ++i)
                {
                    dst[i] = __atomic_load_n(src + i, __ATOMIC_RELAXED);
                }

                __atomic_thread_fence(__ATOMIC_ACQUIRE);

                if(version == __atomic_load_n(&box.version, __ATOMIC_ACQUIRE))
                {
                    return true;
                }
            }

            return false;
        }
    }

    /**
     * @brief 立即关闭六路 PWM 输出
     *
     * @note 可以从 MCPWM ISR 调用
     */
    void IRAM_ATTR stop()
    {
        __atomic_store_n(&wanted, 0U, __ATOMIC_RELEASE);

        for(uint32_t i = 0; i < 6; ++i)
        {
            if(generators[i] != nullptr)
            {
                mcpwm_generator_set_force_level(generators[i], 0, true);
            }
        }

        __atomic_store_n(&enabled, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&pending, 0U, __ATOMIC_RELEASE);
    }

    /**
     * @brief 锁存首个故障并立即停机
     *
     * @param[in] code 故障类型
     * @param[in] index 故障电机索引，0xFF 为公共故障
     * @param[in] raw_a 第一相 ADC 原始值
     * @param[in] raw_b 第二相 ADC 原始值
     */
    void IRAM_ATTR trip(
        motor::fault_code code,
        uint8_t index = 0xFF,
        uint16_t raw_a = 0,
        uint16_t raw_b = 0)
    {
        uint32_t expected = 0;

        if(__atomic_compare_exchange_n(
            &fault, &expected, static_cast<uint32_t>(code), false,
            __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
        {
            __atomic_store_n(&fault_motor, index, __ATOMIC_RELAXED);
            __atomic_store_n(&fault_raw_a, raw_a, __ATOMIC_RELAXED);
            __atomic_store_n(&fault_raw_b, raw_b, __ATOMIC_RELAXED);
            __atomic_store_n(&fault_us, now32(), __ATOMIC_RELEASE);
        }

        stop();
    }

    /**
     * @brief 获取当前锁存故障
     *
     * @return 故障代码，NONE 表示无故障
     */
    motor::fault_code get_fault_code()
    {
        return static_cast<motor::fault_code>(
            __atomic_load_n(&fault, __ATOMIC_ACQUIRE));
    }

    /**
     * @brief 导出故障快照
     *
     * @return 已锁存的故障信息
     */
    motor::fault_info get_fault()
    {
        motor::fault_info out{};
        out.code = get_fault_code();
        out.active = out.code != motor::fault_code::NONE;
        out.motor_index = __atomic_load_n(&fault_motor, __ATOMIC_RELAXED);
        out.raw_a = __atomic_load_n(&fault_raw_a, __ATOMIC_RELAXED);
        out.raw_b = __atomic_load_n(&fault_raw_b, __ATOMIC_RELAXED);

        const uint32_t stamp = __atomic_load_n(&fault_us, __ATOMIC_ACQUIRE);

        if(stamp != 0)
        {
            const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
            const uint32_t age = static_cast<uint32_t>(now) - stamp;
            out.timestamp_us = now >= age ? now - age : 0;
        }

        return out;
    }

    /**
     * @brief 检查当前是否已经释放 PWM 输出
     *
     * @return true 输出处于运行状态
     */
    bool is_running()
    {
        return __atomic_load_n(&enabled, __ATOMIC_ACQUIRE) != 0;
    }

    /**
     * @brief 检查是否有输出请求
     *
     * @return true 正在启动或运行
     */
    bool is_requested()
    {
        return __atomic_load_n(&wanted, __ATOMIC_ACQUIRE) != 0;
    }

    /**
     * @brief 获取当前输出代号，用于丢弃前一轮电机命令
     *
     * @return 当前运行代号
     */
    uint32_t get_generation()
    {
        return __atomic_load_n(&generation, __ATOMIC_ACQUIRE);
    }

    /**
     * @brief 更新控制任务心跳
     *
     * @param[in] timestamp_us 当前 32 位微秒计数
     */
    void heartbeat(uint32_t timestamp_us)
    {
        __atomic_store_n(&heartbeat_us, timestamp_us, __ATOMIC_RELEASE);
    }

    /**
     * @brief 请求单电机或双电机开始输出
     *
     * @param[in] new_mask bit0 为左电机，bit1 为右电机
     *
     * @return true 接受启动请求
     * @return false 已经运行或存在故障
     */
    bool start(uint32_t new_mask)
    {
        if(new_mask == 0 || new_mask > 3 ||
           is_requested() || is_running() ||
           get_fault_code() != motor::fault_code::NONE)
        {
            return false;
        }

        const uint32_t now = now32();
        __atomic_store_n(&mask, new_mask, __ATOMIC_RELEASE);
        __atomic_add_fetch(&generation, 1U, __ATOMIC_ACQ_REL);

        for(uint32_t i = 0; i < 2; ++i)
        {
            last_valid_sample_us[i] = now;
        }

        __atomic_store_n(&wanted, 1U, __ATOMIC_RELEASE);
        return true;
    }

    /**
     * @brief 获取当前母线 ADC 值与最近成功采样时间
     *
     * @param[out] raw 原始母线 ADC 值
     * @param[out] stamp_us 原始 32 位微秒时间戳
     */
    void get_bus(uint16_t &raw, uint32_t &stamp_us)
    {
        raw = __atomic_load_n(&bus_raw, __ATOMIC_ACQUIRE);
        stamp_us = __atomic_load_n(&bus_us, __ATOMIC_ACQUIRE);
    }

    /**
     * @brief 读取最新电机电流采样快照
     *
     * @param[in] index 电机索引
     * @param[out] out 采样数据
     *
     * @return true 数据一致
     */
    bool get_sample(uint32_t index, sample &out)
    {
        return consume(samples[index], out);
    }

    /**
     * @brief 提交一组 PWM 占空比命令
     *
     * @param[in] index 电机索引
     * @param[in] value 控制任务计算的命令
     */
    void set_command(uint32_t index, const command &value)
    {
        publish(commands[index], value);
    }

    /**
     * @brief PWM 周期零点回调：看门狗及母线采样
     *
     * @return false 不要求任务切换
     */
    bool IRAM_ATTR on_empty(
        mcpwm_timer_handle_t,
        const mcpwm_timer_event_data_t *,
        void *)
    {
        const uint32_t now = now32();
        ++cycle;

        for(uint32_t i = 0; i < 6; ++i)
        {
            applied[i] = queued[i];
        }

        if(pending && static_cast<int32_t>(cycle - enable_cycle) >= 0)
        {
            __atomic_store_n(&enabled, 1U, __ATOMIC_RELEASE);
            __atomic_store_n(&pending, 0U, __ATOMIC_RELEASE);
        }

        if(!wanted && (enabled || pending))
        {
            stop();
        }

        if(wanted)
        {
            if(static_cast<int32_t>(
                   now - __atomic_load_n(&heartbeat_us, __ATOMIC_ACQUIRE)) > 3000)
            {
                trip(motor::fault_code::TASK_TIMEOUT);
            }
            else if(now - __atomic_load_n(&bus_us, __ATOMIC_ACQUIRE) > BUS_TIMEOUT_US)
            {
                trip(motor::fault_code::BUS_STALE);
            }
            else if(enabled)
            {
                const uint32_t active = __atomic_load_n(&mask, __ATOMIC_ACQUIRE);

                for(uint32_t i = 0; i < 2; ++i)
                {
                    if((active & (1U << i)) &&
                       now - last_valid_sample_us[i] > SAMPLE_TIMEOUT_US)
                    {
                        trip(motor::fault_code::SAMPLE_TIMEOUT, i);
                        break;
                    }
                }
            }
        }

        static uint32_t divider = 0;

        if(++divider >= 10)
        {
            divider = 0;
            uint16_t raw = 0;

            if(adc::read(ADC_CHANNEL_0, raw))
            {
                __atomic_store_n(&bus_raw, raw, __ATOMIC_RELEASE);
                __atomic_store_n(&bus_us, now32(), __ATOMIC_RELEASE);

                if(enabled && !adc::valid_bus(raw))
                {
                    trip(motor::fault_code::BUS_VOLTAGE);
                }
            }
            else if(enabled)
            {
                trip(motor::fault_code::ADC_READ);
            }
        }

        return false;
    }

    /**
     * @brief PWM 峰值回调：在正确时隙释放输出强制状态
     *
     * @return false 不要求任务切换
     */
    bool IRAM_ATTR on_peak(
        mcpwm_timer_handle_t,
        const mcpwm_timer_event_data_t *,
        void *)
    {
        if(!enabled && !pending && wanted && fault == 0)
        {
            const uint32_t position = phase();

            if(position >= 4000 && position < 6400)
            {
                const uint32_t active = __atomic_load_n(&mask, __ATOMIC_ACQUIRE);

                for(uint32_t op = 0; op < 3; ++op)
                {
                    mcpwm_gen_force_reg_t force{};
                    force.val = MCPWM0.operators[op].gen_force.val;
                    force.gen_cntuforce_upmethod = 1;
                    force.gen_a_cntuforce_mode =
                        (active & (1U << ((2U * op) / 3U))) ? 0 : 1;
                    force.gen_b_cntuforce_mode =
                        (active & (1U << ((2U * op + 1U) / 3U))) ? 0 : 1;
                    MCPWM0.operators[op].gen_force.val = force.val;
                }

                enable_cycle = cycle + 1U;
                __atomic_store_n(&pending, 1U, __ATOMIC_RELEASE);
            }
        }

        return false;
    }

    /**
     * @brief 在验证过的相位窗口更新六路比较值
     *
     * @return false 不要求任务切换
     */
    bool IRAM_ATTR on_commit(
        mcpwm_timer_handle_t,
        const mcpwm_timer_event_data_t *,
        void *)
    {
        command value[2]{};

        if(!consume(commands[0], value[0]) || !consume(commands[1], value[1]))
        {
            return false;
        }

        const uint32_t active = __atomic_load_n(&mask, __ATOMIC_ACQUIRE);
        const uint32_t tag = __atomic_load_n(&generation, __ATOMIC_ACQUIRE);
        const uint32_t now = now32();

        for(uint32_t i = 0; i < 2; ++i)
        {
            if(!wanted || !(active & (1U << i)) || value[i].generation != tag)
            {
                value[i] = command{};
            }
            else if(enabled && value[i].sample_us != 0 &&
                    now - value[i].sample_us > SAMPLE_TIMEOUT_US)
            {
                trip(motor::fault_code::SAMPLE_TIMEOUT, i);
                return false;
            }
        }

        const uint32_t begin = phase();

        if(begin < 6400 || begin > 7440)
        {
            return false;
        }

        const uint32_t duty[6] =
        {
            value[0].duty_a, value[0].duty_b, value[0].duty_c,
            value[1].duty_a, value[1].duty_b, value[1].duty_c
        };

        for(uint32_t i = 0; i < 6; ++i)
        {
            if(duty[i] != queued[i] &&
               mcpwm_comparator_set_compare_value(comparators[i], duty[i]) != ESP_OK)
            {
                trip(motor::fault_code::PWM_UPDATE);
                return false;
            }
        }

        const uint32_t end = phase();

        if(end < begin || end > 7840)
        {
            trip(motor::fault_code::PWM_UPDATE);
            return false;
        }

        for(uint32_t i = 0; i < 6; ++i)
        {
            queued[i] = duty[i];
        }

        return false;
    }

    /**
     * @brief 每个采样触发时隙交替采集一台电机的两相电流
     *
     * @return false 不要求任务切换
     */
    bool IRAM_ATTR on_sample(
        mcpwm_timer_handle_t,
        const mcpwm_timer_event_data_t *,
        void *)
    {
        static uint32_t turn = 0;
        static uint32_t sequence[2] = {};

        const uint32_t index = (turn++) & 1U;
        sample out{};
        out.timestamp_us = now32();
        out.cycle = cycle;
        out.sequence = ++sequence[index];
        out.duty_a = applied[3U * index];
        out.duty_b = applied[3U * index + 1U];
        out.duty_c = applied[3U * index + 2U];

        const bool output_enabled = enabled && (mask & (1U << index));
        out.a_start = phase();
        const uint32_t cycles = esp_cpu_get_cycle_count();

        uint16_t a = 0;
        uint16_t b = 0;
        const bool ok_a = adc::read(adc::CHANNEL[index * 2], a);
        out.raw_a = a;
        out.a_end = out.a_start +
            (esp_cpu_get_cycle_count() - cycles + CYCLES_PER_TICK - 1U) / CYCLES_PER_TICK;
        out.b_start = out.a_end;
        const bool ok_b = adc::read(adc::CHANNEL[index * 2 + 1], b);
        out.raw_b = b;
        out.b_end = out.a_start +
            (esp_cpu_get_cycle_count() - cycles + CYCLES_PER_TICK - 1U) / CYCLES_PER_TICK;
        out.raw_bus = __atomic_load_n(&bus_raw, __ATOMIC_ACQUIRE);

        const bool window_a = !output_enabled ||
            (out.a_start >= out.duty_a + 160U &&
             out.a_end + 160U <= PERIOD - out.duty_a);
        const bool window_b = !output_enabled ||
            (out.b_start >= out.duty_b + 160U &&
             out.b_end + 160U <= PERIOD - out.duty_b);
        const bool raw_ok = ok_a && ok_b && a > 0 && b > 0 && a < 4095 && b < 4095;
        const bool same_cycle = out.cycle == cycle && out.b_end < PERIOD;

        out.flags = (output_enabled ? 1U : 0U) |
                    (ok_a ? 2U : 0U) |
                    (ok_b ? 4U : 0U) |
                    (window_a ? 8U : 0U) |
                    (window_b ? 16U : 0U) |
                    (same_cycle ? 32U : 0U);

        const bool over = adc::overcurrent(index, a, b);

        if(output_enabled && (!raw_ok || over))
        {
            trip(raw_ok ? motor::fault_code::OVERCURRENT
                        : motor::fault_code::ADC_READ, index, a, b);
        }

        if(output_enabled && raw_ok && !over && window_a && window_b && same_cycle)
        {
            __atomic_store_n(&last_valid_sample_us[index],
                             out.timestamp_us, __ATOMIC_RELEASE);
        }

        publish(samples[index], out);
        return false;
    }

    /**
     * @brief 配置固定六路 MCPWM、双定时器与全部 ISR 回调
     *
     * @note 初始化后六路输出仍被强制为低电平
     */
    void init()
    {
        gpio_config_t pins{};
        for(uint32_t i = 0; i < 6; ++i)
        {
            pins.pin_bit_mask |= 1ULL << PINS[i];
        }
        pins.mode = GPIO_MODE_INPUT;
        pins.pull_down_en = GPIO_PULLDOWN_ENABLE;
        ESP_ERROR_CHECK(gpio_config(&pins));

        mcpwm_timer_config_t config{};
        config.group_id = 0;
        config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        config.resolution_hz = 80000000;
        config.period_ticks = PERIOD;
        config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
        config.intr_priority = 3;
        ESP_ERROR_CHECK(mcpwm_new_timer(&config, &timer));

        for(uint32_t op = 0; op < 3; ++op)
        {
            mcpwm_operator_config_t op_config{};
            op_config.group_id = 0;
            op_config.intr_priority = 3;
            ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &operators[op]));
            ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[op], timer));

            for(uint32_t j = 0; j < 2; ++j)
            {
                const uint32_t index = 2U * op + j;
                mcpwm_comparator_config_t cmp{};
                cmp.flags.update_cmp_on_tez = true;
                ESP_ERROR_CHECK(mcpwm_new_comparator(operators[op], &cmp, &comparators[index]));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[index], NEUTRAL));

                mcpwm_generator_config_t gen{};
                gen.gen_gpio_num = PINS[index];
                ESP_ERROR_CHECK(mcpwm_new_generator(operators[op], &gen, &generators[index]));
                ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[index], 0, true));
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                    generators[index],
                    MCPWM_GEN_TIMER_EVENT_ACTION(
                        MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                        MCPWM_GEN_ACTION_HIGH)));
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                    generators[index],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(
                        MCPWM_TIMER_DIRECTION_UP, comparators[index],
                        MCPWM_GEN_ACTION_LOW)));
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                    generators[index],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(
                        MCPWM_TIMER_DIRECTION_DOWN, comparators[index],
                        MCPWM_GEN_ACTION_HIGH)));
            }
        }

        ESP_ERROR_CHECK(mcpwm_new_timer(&config, &sample_timer));
        mcpwm_timer_sync_src_config_t sync_config{};
        sync_config.timer_event = MCPWM_TIMER_EVENT_EMPTY;
        ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timer, &sync_config, &sync));
        mcpwm_timer_sync_phase_config_t phase_config{};
        phase_config.sync_src = sync;
        phase_config.count_value = 1600;
        phase_config.direction = MCPWM_TIMER_DIRECTION_UP;
        ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(sample_timer, &phase_config));

        mcpwm_timer_event_callbacks_t adc_events{};
        adc_events.on_full = on_sample;
        adc_events.on_empty = on_commit;
        ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(
            sample_timer, &adc_events, nullptr));

        mcpwm_timer_event_callbacks_t pwm_events{};
        pwm_events.on_empty = on_empty;
        pwm_events.on_full = on_peak;
        ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(
            timer, &pwm_events, nullptr));

        ESP_ERROR_CHECK(mcpwm_timer_enable(sample_timer));
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    }

    /**
     * @brief 启动 PWM 和同步采样定时器，但不释放 PWM 强制低电平
     */
    void start_timers()
    {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(
            sample_timer, MCPWM_TIMER_START_NO_STOP));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(
            timer, MCPWM_TIMER_START_NO_STOP));
    }
}

namespace foc
{
    namespace
    {
        constexpr float TWO_PI = 6.28318530718f;
        constexpr float INV_SQRT3 = 0.57735026919f;
        constexpr float KP = 0.060318579f;
        constexpr float KI = 226.194672f;
        constexpr float BUS_LIMIT_RATIO = 0.48f;
        constexpr float POLE_PAIRS = 7.0f;
        constexpr uint32_t SINE_TABLE_SIZE = 4096;
        constexpr uint32_t SINE_TABLE_MASK = SINE_TABLE_SIZE - 1;

        struct state
        {
            float direction = 0.0f;
            float zero_rad = 0.0f;
            float integral_d = 0.0f;
            float integral_q = 0.0f;
            uint32_t last_sample_us = 0;
            bool aligned = false;
        };

        state motors[2] = {};
        DRAM_ATTR float sine_table[SINE_TABLE_SIZE] = {};

        /**
         * @brief 归一化电角度
         *
         * @param[in] angle 输入弧度
         *
         * @return [0, 2π) 范围内的角度
         */
        float wrap(float angle)
        {
            // 单圈角度使用快速归一化，大角度保留通用处理
            if(fabsf(angle) > 64.0f)
            {
                angle = fmodf(angle, TWO_PI);
            }
            else
            {
                angle -= static_cast<int32_t>(angle * (1.0f / TWO_PI)) * TWO_PI;
            }

            if(angle < 0.0f)
            {
                angle += TWO_PI;
            }

            return angle >= TWO_PI ? 0.0f : angle;
        }

        /**
         * @brief 通过正弦表和线性插值计算电角度的正弦、余弦
         *
         * @param[in] angle 已归一化的电角度，单位 rad，允许校准末端的 2π
         * @param[out] sn 电角度正弦
         * @param[out] cs 电角度余弦
         */
        void sincos(float angle, float &sn, float &cs)
        {
            const float position = angle * (SINE_TABLE_SIZE / TWO_PI);
            const uint32_t whole = static_cast<uint32_t>(position);
            const float fraction = position - static_cast<float>(whole);
            const uint32_t index = whole & SINE_TABLE_MASK;
            const uint32_t cosine = (index + SINE_TABLE_SIZE / 4) & SINE_TABLE_MASK;

            sn = sine_table[index] + fraction *
                (sine_table[(index + 1) & SINE_TABLE_MASK] - sine_table[index]);
            cs = sine_table[cosine] + fraction *
                (sine_table[(cosine + 1) & SINE_TABLE_MASK] - sine_table[cosine]);
        }

        /**
         * @brief 将 d/q 轴电压转换为三相比较值
         *
         * @param[in] vd d 轴电压
         * @param[in] vq q 轴电压
         * @param[in] cs 电角度余弦
         * @param[in] sn 电角度正弦
         * @param[in] bus 母线电压
         *
         * @return 三相 PWM 命令
         */
        pwm::command modulation(float vd, float vq, float cs, float sn, float bus)
        {
            const float alpha = vd * cs - vq * sn;
            const float beta = vd * sn + vq * cs;
            const float phase[3] =
            {
                alpha,
                -0.5f * alpha + 0.8660254f * beta,
                -0.5f * alpha - 0.8660254f * beta
            };

            const float highest = fmaxf(phase[0], fmaxf(phase[1], phase[2]));
            const float lowest = fminf(phase[0], fminf(phase[1], phase[2]));
            const float zero = -0.5f * (highest + lowest);
            pwm::command out{};
            uint32_t *duty[3] = {&out.duty_a, &out.duty_b, &out.duty_c};

            for(uint32_t i = 0; i < 3; ++i)
            {
                const float value =
                    (0.5f + (phase[i] + zero) / bus) * 4000.0f + 0.5f;
                const uint32_t count = static_cast<uint32_t>(fmaxf(value, 0.0f));
                *duty[i] = count < 320 ? 320 : (count > 3680 ? 3680 : count);
            }

            return out;
        }
    }

    /**
     * @brief 初始化内部 RAM 中的正弦查找表
     *
     * @note 在控制任务和 PWM 定时器启动前调用
     */
    void init()
    {
        for(uint32_t i = 0; i < SINE_TABLE_SIZE; ++i)
        {
            sine_table[i] = sinf(static_cast<float>(i) * (TWO_PI / SINE_TABLE_SIZE));
        }
    }

    /**
     * @brief 完成本次上电的电机方向和电角度零偏标定
     *
     * @param[in] index 电机索引
     * @param[in] direction 编码器方向，取 +1 或 -1
     * @param[in] mechanical_angle_rad 电角度为零时的机械角度
     */
    void set_alignment(uint32_t index, float direction, float mechanical_angle_rad)
    {
        motors[index].direction = direction;
        motors[index].zero_rad = wrap(direction * POLE_PAIRS * mechanical_angle_rad);
        motors[index].aligned = true;
    }

    /**
     * @brief 在进入正常电流环前清空 PI 积分及时间基准
     */
    void reset()
    {
        for(uint32_t i = 0; i < 2; ++i)
        {
            motors[i].integral_d = 0.0f;
            motors[i].integral_q = 0.0f;
            motors[i].last_sample_us = 0;
        }
    }

    /**
     * @brief 处理新电流采样并计算下一个 PWM 命令
     *
     * @param[in] index 电机索引
     * @param[in] raw 双相电流样本
     * @param[in] encoder 编码器快照
     * @param[in] target 期望 d/q 轴电流
     * @param[in] aligning 是否采用开环校准电压
     * @param[in] align_theta 校准电角度
     * @param[in] align_voltage 校准电压
     * @param[out] info 实际测量信息
     * @param[out] output PWM 命令
     *
     * @return true 成功生成命令
     * @return false 电流 PI 时间间隔或参数无效
     */
    bool update(
        uint32_t index,
        const pwm::sample &raw,
        const sensor::encoder_data &encoder,
        const motor::target_current &target,
        bool aligning,
        float align_theta,
        float align_voltage,
        motor::current_info &info,
        pwm::command &output)
    {
        const float bus = adc::voltage(raw.raw_bus);
        if(!isfinite(bus) || bus < 5.0f || bus > 13.0f)
        {
            return false;
        }

        float theta = align_theta;

        if(!aligning)
        {
            if(!motors[index].aligned)
            {
                return false;
            }

            const int32_t age = static_cast<int32_t>(
                raw.timestamp_us - static_cast<uint32_t>(encoder.timestamp_us));
            const float predicted_angle =
                encoder.single_turn_rad + encoder.speed_rad_s * (age * 1.0e-6f);
            theta = wrap(motors[index].direction * POLE_PAIRS * predicted_angle -
                         motors[index].zero_rad);
        }

        const float ia = adc::current(index * 2, raw.raw_a, index);
        const float ib = adc::current(index * 2 + 1, raw.raw_b, index);
        const float ic = -ia - ib;
        const float beta = (ia + 2.0f * ib) * INV_SQRT3;
        float cs = 0.0f;
        float sn = 0.0f;
        sincos(theta, sn, cs);
        const float id = ia * cs + beta * sn;
        const float iq = beta * cs - ia * sn;

        const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t age = static_cast<uint32_t>(now) - raw.timestamp_us;
        info.timestamp_us = now >= age ? now - age : 0;
        info.electrical_angle_rad = theta;
        info.mechanical_angle_rad = encoder.angle_rad;
        info.mechanical_speed_rad_s = encoder.speed_rad_s;
        info.encoder_direction = aligning ? 0.0f : motors[index].direction;
        info.phase_A[0] = ia;
        info.phase_A[1] = ib;
        info.phase_A[2] = ic;
        info.id_A = id;
        info.iq_A = iq;
        info.valid = true;

        float vd = align_voltage;
        float vq = 0.0f;

        if(!aligning)
        {
            const uint32_t dt_us = motors[index].last_sample_us
                ? raw.timestamp_us - motors[index].last_sample_us
                : 200;
            motors[index].last_sample_us = raw.timestamp_us;

            if(dt_us == 0 || dt_us > pwm::SAMPLE_TIMEOUT_US)
            {
                return false;
            }

            const float dt = dt_us * 1.0e-6f;
            const float error_d = target.id_A - id;
            const float error_q = target.iq_A - iq;
            const float candidate_d = motors[index].integral_d + KI * dt * error_d;
            const float candidate_q = motors[index].integral_q + KI * dt * error_q;

            vd = KP * error_d + candidate_d;
            vq = KP * error_q + candidate_q;
            const float limit = BUS_LIMIT_RATIO * bus;
            const float magnitude_squared = vd * vd + vq * vq;

            if(magnitude_squared > limit * limit)
            {
                const float scale = limit / sqrtf(magnitude_squared);
                vd *= scale;
                vq *= scale;
                motors[index].integral_d = vd - KP * error_d;
                motors[index].integral_q = vq - KP * error_q;
            }
            else
            {
                motors[index].integral_d = candidate_d;
                motors[index].integral_q = candidate_q;
            }
        }

        output = modulation(vd, vq, cs, sn, bus);
        output.generation = pwm::get_generation();
        output.sample_us = raw.timestamp_us;
        return true;
    }
}

namespace motor
{
    namespace
    {
        constexpr uint32_t ENCODER_TIMEOUT_US = 1500;
        constexpr uint32_t WAIT_TIMEOUT_US = 3000000;
        constexpr uint32_t ALIGN_GAP_US = 2000000;
        constexpr uint32_t ALIGN_LOCK_US = 200000;
        constexpr uint32_t ALIGN_LOCK_TIMEOUT_US = 600000;
        constexpr uint32_t ALIGN_SWEEP_US = 2400000;
        constexpr uint32_t ALIGN_SETTLE_US = 100000;
        constexpr uint32_t ALIGN_FINAL_US = 400000;
        constexpr uint32_t ALIGN_FINAL_TIMEOUT_US = 700000;
        constexpr float ALIGN_VOLTAGE = 0.18f;
        constexpr float ALIGN_MIN_DELTA = 1.2f;
        constexpr float ALIGN_MAX_DELTA = 2.3f;
        constexpr float ALIGN_MAX_SPEED = 0.5f;
        constexpr float TWO_PI = 6.28318530718f;
        constexpr float MAX_TARGET_A = 6.0f;

        enum class stage : uint8_t
        {
            WAITING,
            LOCK,
            SWEEP,
            FINAL,
            STOP,
            RUN
        };

        struct session
        {
            stage state = stage::WAITING;
            uint8_t motor = 0;
            uint32_t began_us = 0;
            uint32_t boot_us = 0;
            uint32_t seen_sequence[2] = {};
            uint64_t seen_encoder_us[2] = {};
            float start_angle = 0.0f;
            double final_angle_sum = 0.0;
            uint16_t stable_count = 0;
            uint16_t final_count = 0;
        };

        session calibration{};
        foc_info latest{};
        portMUX_TYPE info_lock = portMUX_INITIALIZER_UNLOCKED;
        uint32_t initialized = 0;
        uint32_t task_ready = 0;
        uint32_t sensor_ok = 0;
        uint32_t ready = 0;

        /**
         * @brief 获取当前 32 位微秒时间
         *
         * @return 当前计数
         */
        uint32_t now32()
        {
            return static_cast<uint32_t>(esp_timer_get_time());
        }

        /**
         * @brief 进入某台电机的开环锁定校准
         *
         * @param[in] index 电机索引
         * @param[in] now 当前时间
         */
        void begin_alignment(uint8_t index, uint32_t now)
        {
            calibration.motor = index;
            calibration.state = stage::LOCK;
            calibration.began_us = now;
            calibration.stable_count = 0;
            calibration.final_count = 0;
            calibration.final_angle_sum = 0.0;

            if(!pwm::start(1U << index))
            {
                pwm::trip(fault_code::ALIGNMENT, index);
            }
        }

        /**
         * @brief 按最新状态公开测得的电流数据
         *
         * @param[in] index 电机索引
         * @param[in] current 反馈数据
         */
        void publish_current(uint32_t index, const current_info &current)
        {
            portENTER_CRITICAL(&info_lock);
            latest.current[index] = current;
            portEXIT_CRITICAL(&info_lock);
        }

        /**
         * @brief 获取目标电流快照
         *
         * @param[in] index 电机索引
         *
         * @return 目标电流
         */
        target_current get_target(uint32_t index)
        {
            portENTER_CRITICAL(&info_lock);
            const target_current value = latest.target[index];
            portEXIT_CRITICAL(&info_lock);
            return value;
        }

        /**
         * @brief 检查传感器时间戳是否有效且足够新
         *
         * @param[in] encoder 编码器信息
         * @param[in] now64 当前 64 位微秒时间
         *
         * @return true 编码器数据新鲜
         */
        bool fresh(const sensor::encoder_data &encoder, uint64_t now64)
        {
            return encoder.timestamp_us != 0 &&
                   now64 >= encoder.timestamp_us &&
                   now64 - encoder.timestamp_us <= ENCODER_TIMEOUT_US;
        }

        /**
         * @brief 推进开机方向辨识与零偏校准状态机
         *
         * @param[in] sensors 最新编码器快照
         * @param[in] now 当前 32 位微秒时间
         * @param[in] now64 当前 64 位微秒时间
         */
        void calibrate(const sensor::package &sensors, uint32_t now, uint64_t now64)
        {
            const sensor::encoder_data encoders[2] =
            {
                sensors.left_encoder,
                sensors.right_encoder
            };

            if(calibration.state == stage::WAITING)
            {
                uint16_t bus_raw = 0;
                uint32_t bus_us = 0;
                pwm::get_bus(bus_raw, bus_us);

                const bool bus_valid = bus_us != 0 &&
                    now32() - bus_us <= pwm::BUS_TIMEOUT_US &&
                    adc::valid_bus(bus_raw);

                if(bus_valid && fresh(encoders[0], now64) && fresh(encoders[1], now64))
                {
                    begin_alignment(0, now);
                }
                else if(now - calibration.boot_us > WAIT_TIMEOUT_US)
                {
                    pwm::trip(bus_valid ? fault_code::SENSOR_TIMEOUT
                                        : fault_code::BUS_VOLTAGE);
                }
                return;
            }

            if(calibration.state == stage::RUN)
            {
                for(uint32_t i = 0; i < 2; ++i)
                {
                    if(pwm::is_requested() &&
                       (__atomic_load_n(&pwm::mask, __ATOMIC_ACQUIRE) & (1U << i)) &&
                       !fresh(encoders[i], now64))
                    {
                        pwm::trip(fault_code::SENSOR_TIMEOUT, i);
                    }
                }
                return;
            }

            if(calibration.state == stage::STOP)
            {
                if(pwm::is_running() || pwm::is_requested())
                {
                    return;
                }

                if(calibration.motor == 0)
                {
                    if(now - calibration.began_us < ALIGN_GAP_US)
                    {
                        return;
                    }

                    begin_alignment(1, now);
                }
                else
                {
                    foc::reset();
                    calibration.state = stage::RUN;
                    calibration.began_us = now;

                    if(!pwm::start(3U))
                    {
                        pwm::trip(fault_code::ALIGNMENT);
                    }
                }
                return;
            }

            const uint8_t index = calibration.motor;
            const sensor::encoder_data &encoder = encoders[index];

            if(!fresh(encoder, now64))
            {
                pwm::trip(fault_code::SENSOR_TIMEOUT, index);
                return;
            }

            const bool updated = encoder.timestamp_us != calibration.seen_encoder_us[index];
            if(updated)
            {
                calibration.seen_encoder_us[index] = encoder.timestamp_us;
            }

            const uint32_t elapsed = now - calibration.began_us;

            if(calibration.state == stage::LOCK)
            {
                if(updated && elapsed >= ALIGN_SETTLE_US)
                {
                    calibration.stable_count =
                        fabsf(encoder.speed_rad_s) < ALIGN_MAX_SPEED
                            ? calibration.stable_count + 1U : 0U;
                }

                if(elapsed >= ALIGN_LOCK_US && calibration.stable_count >= 10)
                {
                    calibration.start_angle = encoder.angle_rad;
                    calibration.state = stage::SWEEP;
                    calibration.began_us = now;
                }
                else if(elapsed > ALIGN_LOCK_TIMEOUT_US)
                {
                    pwm::trip(fault_code::ALIGNMENT, index);
                }
            }
            else if(calibration.state == stage::SWEEP && elapsed >= ALIGN_SWEEP_US)
            {
                const float delta = encoder.angle_rad - calibration.start_angle;

                if(fabsf(delta) < ALIGN_MIN_DELTA || fabsf(delta) > ALIGN_MAX_DELTA)
                {
                    pwm::trip(fault_code::ALIGNMENT, index);
                }
                else
                {
                    calibration.start_angle = delta > 0.0f ? 1.0f : -1.0f;
                    calibration.state = stage::FINAL;
                    calibration.began_us = now;
                    calibration.final_angle_sum = 0.0;
                    calibration.final_count = 0;
                }
            }
            else if(calibration.state == stage::FINAL)
            {
                if(updated && elapsed >= ALIGN_SETTLE_US)
                {
                    if(fabsf(encoder.speed_rad_s) < ALIGN_MAX_SPEED)
                    {
                        calibration.final_angle_sum += encoder.angle_rad;
                        ++calibration.final_count;
                    }
                    else
                    {
                        calibration.final_angle_sum = 0.0;
                        calibration.final_count = 0;
                    }
                }

                if(elapsed >= ALIGN_FINAL_US && calibration.final_count >= 10)
                {
                    const float final_angle = static_cast<float>(
                        calibration.final_angle_sum / calibration.final_count);
                    foc::set_alignment(index, calibration.start_angle, final_angle);
                    pwm::stop();
                    calibration.state = stage::STOP;
                    calibration.began_us = now;
                }
                else if(elapsed > ALIGN_FINAL_TIMEOUT_US)
                {
                    pwm::trip(fault_code::ALIGNMENT, index);
                }
            }
        }

        /**
         * @brief 处理单个电机的新 ADC 样本并提交 FOC 命令
         *
         * @param[in] index 电机索引
         * @param[in] sample 最新电流样本
         * @param[in] encoder 最新编码器数据
         */
        void process_sample(
            uint32_t index,
            const pwm::sample &sample,
            const sensor::encoder_data &encoder)
        {
            if(!pwm::is_running() || (sample.flags & 63U) != 63U)
            {
                return;
            }

            if(now32() - sample.timestamp_us > pwm::SAMPLE_TIMEOUT_US)
            {
                pwm::trip(fault_code::SAMPLE_TIMEOUT, index);
                return;
            }

            uint16_t bus_raw = 0;
            uint32_t bus_stamp = 0;
            pwm::get_bus(bus_raw, bus_stamp);

            if(bus_stamp == 0 || now32() - bus_stamp > pwm::BUS_TIMEOUT_US)
            {
                pwm::trip(fault_code::BUS_STALE);
                return;
            }

            if(!adc::valid_bus(sample.raw_bus))
            {
                pwm::trip(fault_code::BUS_VOLTAGE);
                return;
            }

            const bool aligning = calibration.state == stage::LOCK ||
                                  calibration.state == stage::SWEEP ||
                                  calibration.state == stage::FINAL;

            if(aligning && index != calibration.motor)
            {
                return;
            }

            if(!aligning && calibration.state != stage::RUN)
            {
                return;
            }

            float theta = 0.0f;
            float voltage = 0.0f;

            if(aligning)
            {
                const uint32_t elapsed = sample.timestamp_us - calibration.began_us;

                if(calibration.state == stage::SWEEP)
                {
                    theta = 2.0f * TWO_PI * fminf(
                        static_cast<float>(elapsed) / ALIGN_SWEEP_US, 1.0f);
                    voltage = ALIGN_VOLTAGE;
                }
                else
                {
                    voltage = ALIGN_VOLTAGE * fminf(
                        static_cast<float>(elapsed) / 50000.0f, 1.0f);
                }
            }

            current_info current{};
            pwm::command output{};

            if(!foc::update(
                index, sample, encoder, get_target(index),
                aligning, theta, voltage, current, output))
            {
                pwm::trip(fault_code::SAMPLE_TIMEOUT, index);
                return;
            }

            publish_current(index, current);
            pwm::set_command(index, output);
        }

        /**
         * @brief 唯一的 Sensor 与双电机 FOC 控制任务
         *
         * @param[in] arg 未使用
         */
        void task(void *)
        {
            const bool ok = sensor::init();
            __atomic_store_n(&sensor_ok, ok ? 1U : 0U, __ATOMIC_RELEASE);
            __atomic_store_n(&task_ready, 1U, __ATOMIC_RELEASE);

            if(!ok)
            {
                pwm::trip(fault_code::SENSOR_INIT);
                vTaskDelete(nullptr);
                return;
            }

            calibration.boot_us = now32();
            calibration.began_us = calibration.boot_us;

            while(true)
            {
                uint32_t notifications = 0;
                xTaskNotifyWait(0, UINT32_MAX, &notifications, 0);
                sensor::poll(notifications, !pwm::is_requested());

                sensor::package sensors{};
                sensor::get_package(sensors);

                const uint64_t now64 = static_cast<uint64_t>(esp_timer_get_time());
                const uint32_t now = static_cast<uint32_t>(now64);
                pwm::heartbeat(now);

                if(pwm::get_fault_code() == fault_code::NONE)
                {
                    calibrate(sensors, now, now64);
                }

                for(uint32_t index = 0; index < 2; ++index)
                {
                    pwm::sample sample{};

                    if(!pwm::get_sample(index, sample) ||
                       sample.sequence == calibration.seen_sequence[index])
                    {
                        continue;
                    }

                    calibration.seen_sequence[index] = sample.sequence;

                    if(!(sample.flags & 1U) || !pwm::is_running())
                    {
                        continue;
                    }

                    if(pwm::get_fault_code() != fault_code::NONE ||
                       static_cast<int32_t>(sample.timestamp_us - calibration.began_us) < 0)
                    {
                        continue;
                    }

                    const sensor::encoder_data &encoder =
                        index == 0 ? sensors.left_encoder : sensors.right_encoder;

                    if(!fresh(encoder, now64))
                    {
                        pwm::trip(fault_code::SENSOR_TIMEOUT, index);
                        break;
                    }

                    if(fabsf(encoder.speed_rad_s) > 40.0f)
                    {
                        pwm::trip(fault_code::SENSOR_TIMEOUT, index);
                        break;
                    }

                    process_sample(index, sample, encoder);
                }

                if(calibration.state == stage::RUN && pwm::is_running() &&
                   pwm::get_fault_code() == fault_code::NONE)
                {
                    __atomic_store_n(&ready, 1U, __ATOMIC_RELEASE);
                }

                if(pwm::get_fault_code() != fault_code::NONE)
                {
                    __atomic_store_n(&ready, 0U, __ATOMIC_RELEASE);
                }

                taskYIELD();
            }
        }
    }

    /**
     * @brief 初始化电机硬件并启动自动校准任务
     *
     * @return true 初始化成功
     * @return false 传感器任务初始化失败
     */
    bool init()
    {
        if(__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
        {
            return true;
        }

        // 固定硬件的 PVCC 门控；PWM 在初始化期间保持强制低电平
        pwm::init();
        ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_7, 1));
        vTaskDelay(pdMS_TO_TICKS(10));
        adc::init();
        foc::init();

        if(xTaskCreatePinnedToCore(task, "motor_control", 8192,
                                   nullptr, 5, nullptr, 1) != pdPASS)
        {
            gpio_set_level(GPIO_NUM_7, 0);
            return false;
        }

        while(!__atomic_load_n(&task_ready, __ATOMIC_ACQUIRE))
        {
            vTaskDelay(1);
        }

        if(!__atomic_load_n(&sensor_ok, __ATOMIC_ACQUIRE))
        {
            gpio_set_level(GPIO_NUM_7, 0);
            return false;
        }

        pwm::start_timers();
        __atomic_store_n(&initialized, 1U, __ATOMIC_RELEASE);
        return true;
    }

    /**
     * @brief 设置指定电机的目标 d/q 轴电流
     *
     * @param[in] id 电机编号
     * @param[in] target 目标电流，单位 A
     *
     * @return true 已接收目标电流
     */
    bool set_target_current(motor_id id, const target_current &target)
    {
        const uint32_t index = static_cast<uint32_t>(id);

        if(index > 1 ||
           !isfinite(target.id_A) || !isfinite(target.iq_A) ||
           target.id_A * target.id_A + target.iq_A * target.iq_A >
               MAX_TARGET_A * MAX_TARGET_A ||
           !__atomic_load_n(&ready, __ATOMIC_ACQUIRE) ||
           pwm::get_fault_code() != fault_code::NONE)
        {
            return false;
        }

        portENTER_CRITICAL(&info_lock);
        latest.target[index] = target;
        portEXIT_CRITICAL(&info_lock);
        return true;
    }

    /**
     * @brief 返回当前目标、测量值、母线电压和故障快照
     *
     * @param[out] snapshot 状态快照
     *
     * @return true 模块已初始化
     */
    bool get_info(foc_info &snapshot)
    {
        if(!__atomic_load_n(&initialized, __ATOMIC_ACQUIRE))
        {
            snapshot = foc_info{};
            return false;
        }

        portENTER_CRITICAL(&info_lock);
        snapshot = latest;
        portEXIT_CRITICAL(&info_lock);

        snapshot.initialized = true;
        snapshot.fault = pwm::get_fault();
        snapshot.ready = __atomic_load_n(&ready, __ATOMIC_ACQUIRE) != 0 &&
                         snapshot.fault.code == fault_code::NONE;

        uint16_t bus_raw = 0;
        uint32_t bus_us = 0;
        pwm::get_bus(bus_raw, bus_us);
        const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t age = static_cast<uint32_t>(now) - bus_us;

        if(bus_us != 0 && age <= pwm::BUS_TIMEOUT_US)
        {
            snapshot.bus_voltage_V = adc::voltage(bus_raw);
            snapshot.bus_timestamp_us = now >= age ? now - age : 0;
        }

        return true;
    }
}
