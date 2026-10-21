
#include "motor.h"
#include "sensor.h"

#include <math.h>
#include <stdint.h>

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

#include "hal/adc_ll.h"
#include "soc/mcpwm_struct.h"
#include "soc/sens_reg.h"

namespace motor
{
    namespace pwm
    {
        /**
         * @brief 强制关闭所有 PWM 输出
         */
        void IRAM_ATTR force_off();
    }

    namespace detail
    {
        constexpr uint32_t CPU_MHZ = 160;
        constexpr uint32_t PWM_PERIOD = 8000;
        constexpr uint32_t PWM_ZERO = 2000;
        constexpr uint32_t BASELINE_SAMPLES = 512;
        constexpr uint32_t POLE_PAIRS = 7;

        constexpr uint32_t BUS_STALE_US = 5000;
        constexpr uint32_t SAMPLE_STALE_US = 1000;
        constexpr uint32_t ENCODER_STALE_US = 1500;
        constexpr uint32_t TASK_STALE_US = 3000;

        constexpr uint32_t ALIGN_LOCK_US = 200000;
        constexpr uint32_t ALIGN_LOCK_TIMEOUT_US = 600000;
        constexpr uint32_t ALIGN_SWEEP_US = 1200000;
        constexpr uint32_t ALIGN_FINAL_US = 400000;
        constexpr uint32_t ALIGN_FINAL_TIMEOUT_US = 700000;
        constexpr uint32_t ALIGN_SETTLE_US = 100000;

        constexpr float TWO_PI = 6.28318530718f;
        constexpr float INV_SQRT3 = 0.57735026919f;

        constexpr float ALIGN_VOLTAGE_V = 0.06f;
        constexpr float CURRENT_KP = 0.060318579f;
        constexpr float CURRENT_KI = 226.194672f;
        constexpr float VOLTAGE_LIMIT_V = 0.12f;
        constexpr float BUS_VOLTAGE_RATIO_LIMIT = 0.015f;

        constexpr float MAX_TARGET_CURRENT_A = 0.7f;
        constexpr float HARD_CURRENT_LIMIT_A = 2.0f;
        constexpr float MAX_MOTOR_SPEED_RAD_S = 40.0f;
        constexpr float ALIGN_MAX_SPEED_RAD_S = 0.5f;
        constexpr float ALIGN_MIN_DELTA_RAD = 0.25f;
        constexpr float ALIGN_MAX_DELTA_RAD = 1.5f;

        struct sample
        {
            uint32_t us;
            uint32_t cycle;
            uint32_t seq;

            uint32_t a;
            uint32_t b;
            uint32_t bus;

            uint32_t ca;
            uint32_t cb;
            uint32_t cc;

            uint32_t a0;
            uint32_t a1;
            uint32_t b0;
            uint32_t b1;

            uint32_t flags;
        };

        struct sample_box
        {
            uint32_t version = 0;
            sample value = {};
        };

        struct command
        {
            uint32_t tag = 0;
            uint32_t seq = 0;
            uint32_t sample_us = 0;

            uint32_t ca = PWM_ZERO;
            uint32_t cb = PWM_ZERO;
            uint32_t cc = PWM_ZERO;
        };

        struct command_box
        {
            uint32_t version = 0;
            command value = {};
        };

        portMUX_TYPE info_lock = portMUX_INITIALIZER_UNLOCKED;

        foc_info latest_info{};
        target_current targets[2]{};

        sample_box feedback[2]{};
        command_box commands[2]{};

        uint32_t requested = 0;
        uint32_t running = 0;
        uint32_t enable_pending = 0;
        uint32_t enable_epoch = 0;
        uint32_t epoch = 0;
        uint32_t active_mask = 0;
        uint32_t run_tag = 1;

        uint32_t heartbeat_us = 0;
        uint32_t bus_raw = 0;
        uint32_t bus_timestamp_us = 0;
        uint32_t last_valid_sample_us[2] = {};

        uint32_t fault_value = 0;
        uint32_t fault_time_low = 0;
        uint32_t fault_time_high = 0;
        uint32_t fault_motor = 0xFF;
        uint32_t fault_raw_a = 0;
        uint32_t fault_raw_b = 0;

        uint32_t task_ready = 0;
        uint32_t sensor_init_ok = 0;
        uint32_t module_initialized = 0;
        uint32_t ready_flag = 0;
        bool init_started = false;

        float direction[2] = {};
        float zero_offset[2] = {};
        bool zero_valid[2] = {};

        /**
         * @brief 获取 32 位微秒计数
         *
         * @return 当前微秒计数，允许自然溢出
         */
        uint32_t IRAM_ATTR now_us32()
        {
            return static_cast<uint32_t>(esp_timer_get_time());
        }

        /**
         * @brief 将角度归一化至 [0, 2π)
         *
         * @param[in] angle 输入角度，单位 rad
         *
         * @return 归一化后的角度
         */
        float wrap_angle(float angle)
        {
            angle = fmodf(angle, TWO_PI);
            return angle < 0.0f ? angle + TWO_PI : angle;
        }

        /**
         * @brief 发布电流环采样结果
         *
         * @param[in] index 电机索引
         * @param[in] value 电流与角度快照
         */
        void publish_current(uint32_t index, const current_info &value)
        {
            portENTER_CRITICAL(&info_lock);
            latest_info.current[index] = value;
            portEXIT_CRITICAL(&info_lock);
        }

        /**
         * @brief 获取电机当前目标电流
         *
         * @param[in] index 电机索引
         *
         * @return 目标 d/q 轴电流
         */
        target_current get_target(uint32_t index)
        {
            portENTER_CRITICAL(&info_lock);
            const target_current value = targets[index];
            portEXIT_CRITICAL(&info_lock);

            return value;
        }

        /**
         * @brief 记录首个故障并立即关闭输出
         *
         * @param[in] code 故障代码
         * @param[in] index 电机索引；0xFF 表示非单电机故障
         * @param[in] raw_a A 相电流原始 ADC 值
         * @param[in] raw_b B 相电流原始 ADC 值
         */
        void IRAM_ATTR latch_fault(
            fault_code code,
            uint8_t index = 0xFF,
            uint32_t raw_a = 0,
            uint32_t raw_b = 0)
        {
            uint32_t expected = 0;

            if(__atomic_compare_exchange_n(
                &fault_value,
                &expected,
                static_cast<uint32_t>(code),
                false,
                __ATOMIC_ACQ_REL,
                __ATOMIC_ACQUIRE))
            {
                const uint64_t timestamp =
                    static_cast<uint64_t>(esp_timer_get_time());

                __atomic_store_n(&fault_motor, index, __ATOMIC_RELAXED);
                __atomic_store_n(&fault_raw_a, raw_a, __ATOMIC_RELAXED);
                __atomic_store_n(&fault_raw_b, raw_b, __ATOMIC_RELAXED);

                __atomic_store_n(
                    &fault_time_low,
                    static_cast<uint32_t>(timestamp),
                    __ATOMIC_RELAXED);

                __atomic_store_n(
                    &fault_time_high,
                    static_cast<uint32_t>(timestamp >> 32),
                    __ATOMIC_RELEASE);
            }

            __atomic_store_n(&requested, 0U, __ATOMIC_RELEASE);
            pwm::force_off();
        }
    }

    namespace adc
    {
        namespace
        {
            constexpr adc_channel_t CHANNELS[5] =
            {
                ADC_CHANNEL_1,
                ADC_CHANNEL_2,
                ADC_CHANNEL_3,
                ADC_CHANNEL_4,
                ADC_CHANNEL_0
            };

            constexpr float SENSE_GAIN[2] = {-4.0f, -4.0f};

            adc_oneshot_unit_handle_t handle = nullptr;

            uint16_t voltage_mv[5][4096] = {};
            float offset_V[4] = {};

            uint16_t cut_low[4] = {};
            uint16_t cut_high[4] = {};

            uint16_t bus_low = 0;
            uint16_t bus_high = 4095;
        }

        /**
         * @brief 初始化 ADC 通道、芯片校准表与电流零偏
         */
        void init()
        {
            adc_oneshot_unit_init_cfg_t unit{};
            unit.unit_id = ADC_UNIT_1;

            ESP_ERROR_CHECK(
                adc_oneshot_new_unit(&unit, &handle));

            adc_oneshot_chan_cfg_t channel{};
            channel.atten = ADC_ATTEN_DB_12;
            channel.bitwidth = ADC_BITWIDTH_12;

            for(uint32_t i = 0; i < 5; ++i)
            {
                ESP_ERROR_CHECK(
                    adc_oneshot_config_channel(
                        handle, CHANNELS[i], &channel));

                adc_cali_curve_fitting_config_t config{};
                config.unit_id = ADC_UNIT_1;
                config.chan = CHANNELS[i];
                config.atten = ADC_ATTEN_DB_12;
                config.bitwidth = ADC_BITWIDTH_12;

                adc_cali_handle_t calibration = nullptr;

                ESP_ERROR_CHECK(
                    adc_cali_create_scheme_curve_fitting(
                        &config, &calibration));

                for(uint32_t raw = 0; raw < 4096; ++raw)
                {
                    int mv = 0;

                    ESP_ERROR_CHECK(
                        adc_cali_raw_to_voltage(
                            calibration,
                            static_cast<int>(raw),
                            &mv));

                    voltage_mv[i][raw] =
                        static_cast<uint16_t>(mv < 0 ? 0 : mv);
                }

                ESP_ERROR_CHECK(
                    adc_cali_delete_scheme_curve_fitting(calibration));
            }

            for(uint32_t n = 0; n < detail::BASELINE_SAMPLES; ++n)
            {
                for(uint32_t ch = 0; ch < 4; ++ch)
                {
                    int raw = 0;

                    ESP_ERROR_CHECK(
                        adc_oneshot_read(
                            handle, CHANNELS[ch], &raw));

                    offset_V[ch] +=
                        voltage_mv[ch][raw] * 0.001f /
                        static_cast<float>(detail::BASELINE_SAMPLES);
                }
            }

            for(uint32_t ch = 0; ch < 4; ++ch)
            {
                cut_low[ch] = 0;
                cut_high[ch] = 4095;

                for(uint32_t raw = 0; raw < 4096; ++raw)
                {
                    const float current =
                        (voltage_mv[ch][raw] * 0.001f -
                         offset_V[ch]) * 4.0f;

                    if(current < -detail::HARD_CURRENT_LIMIT_A)
                    {
                        cut_low[ch] = static_cast<uint16_t>(raw);
                    }

                    if(current <= detail::HARD_CURRENT_LIMIT_A)
                    {
                        cut_high[ch] = static_cast<uint16_t>(raw);
                    }
                }
            }

            for(uint32_t raw = 0; raw < 4096; ++raw)
            {
                const float voltage = voltage_mv[4][raw] * 0.003f;

                if(voltage < 5.0f)
                {
                    bus_low = static_cast<uint16_t>(raw);
                }

                if(voltage <= 13.0f)
                {
                    bus_high = static_cast<uint16_t>(raw);
                }
            }
        }

        /**
         * @brief 在中断上下文中读取一次 ADC 原始值
         *
         * @param[in] channel ADC1 通道
         * @param[out] raw 12 位原始采样值
         *
         * @return true 采样完成
         * @return false ADC 转换超时
         */
        bool IRAM_ATTR read_raw(adc_channel_t channel, uint16_t &raw)
        {
            const uint32_t idle_start = esp_cpu_get_cycle_count();

            while(SENS.sar_slave_addr1.meas_status != 0)
            {
                if(esp_cpu_get_cycle_count() - idle_start >=
                   detail::CPU_MHZ * 20)
                {
                    return false;
                }
            }

            adc_oneshot_ll_set_channel(ADC_UNIT_1, channel);
            adc_oneshot_ll_start(ADC_UNIT_1);

            const uint32_t start = esp_cpu_get_cycle_count();

            while(!adc_oneshot_ll_get_event(
                ADC_LL_EVENT_ADC1_ONESHOT_DONE))
            {
                if(esp_cpu_get_cycle_count() - start >=
                   detail::CPU_MHZ * 20)
                {
                    if(!adc_oneshot_ll_get_event(
                        ADC_LL_EVENT_ADC1_ONESHOT_DONE))
                    {
                        return false;
                    }
                }
            }

            raw = static_cast<uint16_t>(
                adc_oneshot_ll_get_raw_result(ADC_UNIT_1));

            return true;
        }

        /**
         * @brief 将电流采样通道的 ADC 原始值换算为安培
         *
         * @param[in] channel 电流采样通道索引，范围 0–3
         * @param[in] raw ADC 原始值
         * @param[in] index 电机索引
         *
         * @return 换算后的电流，单位 A
         */
        float current(uint32_t channel, uint16_t raw, uint32_t index)
        {
            return (voltage_mv[channel][raw] * 0.001f -
                    offset_V[channel]) * SENSE_GAIN[index];
        }

        /**
         * @brief 将母线 ADC 原始值换算为实际母线电压
         *
         * @param[in] raw ADC 原始值
         *
         * @return 母线电压，单位 V
         */
        float bus_voltage(uint32_t raw)
        {
            return voltage_mv[4][raw] * 0.003f;
        }

        /**
         * @brief 判断母线 ADC 原始值是否处于允许范围
         *
         * @param[in] raw ADC 原始值
         *
         * @return true 母线电压处于允许范围
         */
        bool bus_in_range(uint32_t raw)
        {
            return raw >= bus_low && raw <= bus_high;
        }

        /**
         * @brief 判断电流采样值是否超出硬件保护阈值
         *
         * @param[in] channel 电流采样通道索引
         * @param[in] raw ADC 原始值
         *
         * @return true 采样值超出允许范围
         */
        bool overcurrent(uint32_t channel, uint16_t raw)
        {
            return raw < cut_low[channel] ||
                   raw > cut_high[channel];
        }

        /**
         * @brief 获取指定电机的两路电流 ADC 通道
         *
         * @param[in] index 电机索引
         *
         * @return A 相与 B 相 ADC 通道
         */
        void get_channels(uint32_t index, adc_channel_t (&channels)[2])
        {
            channels[0] = CHANNELS[2U * index];
            channels[1] = CHANNELS[2U * index + 1U];
        }
    }

    namespace pwm
    {
        namespace
        {
            constexpr int PINS[6] = {35, 34, 33, 12, 11, 10};

            mcpwm_timer_handle_t timer = nullptr;
            mcpwm_timer_handle_t sample_timer = nullptr;
            mcpwm_sync_handle_t sync_source = nullptr;

            mcpwm_oper_handle_t operators[3] = {};
            mcpwm_cmpr_handle_t comparators[6] = {};
            mcpwm_gen_handle_t generators[6] = {};

            uint32_t queued[6] =
            {
                detail::PWM_ZERO, detail::PWM_ZERO, detail::PWM_ZERO,
                detail::PWM_ZERO, detail::PWM_ZERO, detail::PWM_ZERO
            };

            uint32_t applied[6] =
            {
                detail::PWM_ZERO, detail::PWM_ZERO, detail::PWM_ZERO,
                detail::PWM_ZERO, detail::PWM_ZERO, detail::PWM_ZERO
            };

            /**
             * @brief 获取主 PWM 计数器当前相位
             *
             * @return 当前计数位置，范围 [0, PWM_PERIOD)
             */
            uint32_t IRAM_ATTR timer_phase()
            {
                mcpwm_timer_status_reg_t status{};
                status.val = MCPWM0.timer[0].timer_status.val;

                const uint32_t phase = status.timer_direction
                    ? detail::PWM_PERIOD - status.timer_value
                    : status.timer_value;

                return phase % detail::PWM_PERIOD;
            }

            /**
             * @brief 发布 ADC 中断采样结果
             *
             * @param[in] index 电机索引
             * @param[in] value 采样数据
             */
            void publish_sample(
                uint32_t index,
                const detail::sample &value)
            {
                detail::sample_box &box = detail::feedback[index];

                __atomic_add_fetch(
                    &box.version, 1U, __ATOMIC_ACQ_REL);

                uint32_t *dst =
                    reinterpret_cast<uint32_t *>(&box.value);

                const uint32_t *src =
                    reinterpret_cast<const uint32_t *>(&value);

                for(uint32_t i = 0;
                    i < sizeof(value) / sizeof(uint32_t);
                    ++i)
                {
                    __atomic_store_n(
                        dst + i, src[i], __ATOMIC_RELAXED);
                }

                __atomic_add_fetch(
                    &box.version, 1U, __ATOMIC_RELEASE);
            }

            /**
             * @brief 读取指定电机最新的一致性采样快照
             *
             * @param[in] index 电机索引
             * @param[out] out 采样快照
             *
             * @return true 成功取得一致性快照
             * @return false 连续读取均遇到写入竞争
             */
            bool read_sample(uint32_t index, detail::sample &out)
            {
                detail::sample_box &box = detail::feedback[index];

                for(uint32_t attempt = 0; attempt < 3; ++attempt)
                {
                    const uint32_t version =
                        __atomic_load_n(
                            &box.version, __ATOMIC_ACQUIRE);

                    if(version & 1U)
                    {
                        continue;
                    }

                    uint32_t *dst =
                        reinterpret_cast<uint32_t *>(&out);

                    const uint32_t *src =
                        reinterpret_cast<const uint32_t *>(&box.value);

                    for(uint32_t i = 0;
                        i < sizeof(out) / sizeof(uint32_t);
                        ++i)
                    {
                        dst[i] = __atomic_load_n(
                            src + i, __ATOMIC_RELAXED);
                    }

                    __atomic_thread_fence(__ATOMIC_ACQUIRE);

                    if(version ==
                       __atomic_load_n(
                           &box.version, __ATOMIC_ACQUIRE))
                    {
                        return true;
                    }
                }

                return false;
            }

            /**
             * @brief 发布待应用的 PWM 比较值
             *
             * @param[in] index 电机索引
             * @param[in] value 三相 PWM 命令
             */
            void publish_command(
                uint32_t index,
                const detail::command &value)
            {
                detail::command_box &box = detail::commands[index];

                __atomic_add_fetch(
                    &box.version, 1U, __ATOMIC_ACQ_REL);

                uint32_t *dst =
                    reinterpret_cast<uint32_t *>(&box.value);

                const uint32_t *src =
                    reinterpret_cast<const uint32_t *>(&value);

                for(uint32_t i = 0;
                    i < sizeof(value) / sizeof(uint32_t);
                    ++i)
                {
                    __atomic_store_n(
                        dst + i, src[i], __ATOMIC_RELAXED);
                }

                __atomic_add_fetch(
                    &box.version, 1U, __ATOMIC_RELEASE);
            }

            /**
             * @brief 读取待应用的 PWM 命令
             *
             * @param[in] index 电机索引
             * @param[out] out PWM 命令
             *
             * @return true 成功取得一致性快照
             * @return false 连续读取均遇到写入竞争
             */
            bool IRAM_ATTR read_command(
                uint32_t index,
                detail::command &out)
            {
                detail::command_box &box = detail::commands[index];

                for(uint32_t attempt = 0; attempt < 3; ++attempt)
                {
                    const uint32_t version =
                        __atomic_load_n(
                            &box.version, __ATOMIC_ACQUIRE);

                    if(version & 1U)
                    {
                        continue;
                    }

                    uint32_t *dst =
                        reinterpret_cast<uint32_t *>(&out);

                    const uint32_t *src =
                        reinterpret_cast<const uint32_t *>(&box.value);

                    for(uint32_t i = 0;
                        i < sizeof(out) / sizeof(uint32_t);
                        ++i)
                    {
                        dst[i] = __atomic_load_n(
                            src + i, __ATOMIC_RELAXED);
                    }

                    __atomic_thread_fence(__ATOMIC_ACQUIRE);

                    if(version ==
                       __atomic_load_n(
                           &box.version, __ATOMIC_ACQUIRE))
                    {
                        return true;
                    }
                }

                return false;
            }

            /**
             * @brief 在 PWM 周期边界提交母线电压并检查实时状态
             *
             * @param[in] timer_handle 触发回调的定时器
             * @param[in] event_data 定时器事件数据
             * @param[in] user_data 用户数据
             *
             * @return false 不要求唤醒更高优先级任务
             */
            bool IRAM_ATTR on_empty(
                mcpwm_timer_handle_t,
                const mcpwm_timer_event_data_t *,
                void *)
            {
                const uint32_t now = detail::now_us32();

                ++detail::epoch;

                for(uint32_t i = 0; i < 6; ++i)
                {
                    applied[i] = queued[i];
                }

                if(detail::enable_pending &&
                   static_cast<int32_t>(
                       detail::epoch - detail::enable_epoch) >= 0)
                {
                    __atomic_store_n(
                        &detail::running, 1U, __ATOMIC_RELEASE);

                    __atomic_store_n(
                        &detail::enable_pending, 0U, __ATOMIC_RELEASE);
                }

                if(!__atomic_load_n(
                       &detail::requested, __ATOMIC_ACQUIRE) &&
                   (detail::running || detail::enable_pending))
                {
                    force_off();
                }

                if(__atomic_load_n(
                    &detail::requested, __ATOMIC_ACQUIRE))
                {
                    if(now - __atomic_load_n(
                        &detail::heartbeat_us, __ATOMIC_ACQUIRE) >
                        detail::TASK_STALE_US)
                    {
                        detail::latch_fault(fault_code::TASK_TIMEOUT);
                    }
                    else if(now - __atomic_load_n(
                        &detail::bus_timestamp_us, __ATOMIC_ACQUIRE) >
                        detail::BUS_STALE_US)
                    {
                        detail::latch_fault(fault_code::BUS_STALE);
                    }
                    else if(detail::running)
                    {
                        const uint32_t mask =
                            __atomic_load_n(
                                &detail::active_mask, __ATOMIC_ACQUIRE);

                        for(uint32_t m = 0; m < 2; ++m)
                        {
                            if((mask & (1U << m)) &&
                               now - __atomic_load_n(
                                   &detail::last_valid_sample_us[m],
                                   __ATOMIC_ACQUIRE) >
                                   detail::SAMPLE_STALE_US)
                            {
                                detail::latch_fault(
                                    fault_code::SAMPLE_TIMEOUT,
                                    static_cast<uint8_t>(m));
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

                    if(adc::read_raw(ADC_CHANNEL_0, raw))
                    {
                        __atomic_store_n(
                            &detail::bus_raw,
                            static_cast<uint32_t>(raw),
                            __ATOMIC_RELEASE);

                        __atomic_store_n(
                            &detail::bus_timestamp_us,
                            detail::now_us32(),
                            __ATOMIC_RELEASE);

                        if(detail::running && !adc::bus_in_range(raw))
                        {
                            detail::latch_fault(
                                fault_code::BUS_VOLTAGE);
                        }
                    }
                    else if(detail::running)
                    {
                        detail::latch_fault(fault_code::ADC_READ);
                    }
                }

                return false;
            }

            /**
             * @brief 在 PWM 峰值处按需使能电机输出
             *
             * @param[in] timer_handle 触发回调的定时器
             * @param[in] event_data 定时器事件数据
             * @param[in] user_data 用户数据
             *
             * @return false 不要求唤醒更高优先级任务
             */
            bool IRAM_ATTR on_peak(
                mcpwm_timer_handle_t,
                const mcpwm_timer_event_data_t *,
                void *)
            {
                if(!detail::running &&
                   !detail::enable_pending &&
                   __atomic_load_n(
                       &detail::requested, __ATOMIC_ACQUIRE) &&
                   !__atomic_load_n(
                       &detail::fault_value, __ATOMIC_ACQUIRE))
                {
                    const uint32_t phase = timer_phase();

                    if(phase >= 4000 && phase < 6400)
                    {
                        const uint32_t mask =
                            __atomic_load_n(
                                &detail::active_mask, __ATOMIC_ACQUIRE);

                        for(uint32_t op = 0; op < 3; ++op)
                        {
                            mcpwm_gen_force_reg_t force{};
                            force.val =
                                MCPWM0.operators[op].gen_force.val;

                            force.gen_cntuforce_upmethod = 1;

                            force.gen_a_cntuforce_mode =
                                (mask & (1U << ((2U * op) / 3U)))
                                    ? 0 : 1;

                            force.gen_b_cntuforce_mode =
                                (mask & (1U << ((2U * op + 1U) / 3U)))
                                    ? 0 : 1;

                            MCPWM0.operators[op].gen_force.val =
                                force.val;
                        }

                        detail::enable_epoch = detail::epoch + 1U;

                        __atomic_store_n(
                            &detail::enable_pending,
                            1U,
                            __ATOMIC_RELEASE);
                    }
                }

                return false;
            }

            /**
             * @brief 在安全更新窗口内提交两台电机的三相 PWM
             *
             * @param[in] timer_handle 触发回调的定时器
             * @param[in] event_data 定时器事件数据
             * @param[in] user_data 用户数据
             *
             * @return false 不要求唤醒更高优先级任务
             */
            bool IRAM_ATTR on_commit(
                mcpwm_timer_handle_t,
                const mcpwm_timer_event_data_t *,
                void *)
            {
                detail::command value[2]{};

                if(!read_command(0, value[0]) ||
                   !read_command(1, value[1]))
                {
                    return false;
                }

                const uint32_t now = detail::now_us32();
                const uint32_t mask =
                    __atomic_load_n(
                        &detail::active_mask, __ATOMIC_ACQUIRE);

                const uint32_t tag =
                    __atomic_load_n(
                        &detail::run_tag, __ATOMIC_ACQUIRE);

                const bool requested =
                    __atomic_load_n(
                        &detail::requested, __ATOMIC_ACQUIRE) &&
                    !__atomic_load_n(
                        &detail::fault_value, __ATOMIC_ACQUIRE);

                for(uint32_t m = 0; m < 2; ++m)
                {
                    if(!requested ||
                       !(mask & (1U << m)) ||
                       value[m].tag != tag)
                    {
                        value[m].ca = detail::PWM_ZERO;
                        value[m].cb = detail::PWM_ZERO;
                        value[m].cc = detail::PWM_ZERO;
                        value[m].sample_us = 0;
                    }
                    else if(value[m].sample_us == 0 ||
                            now - value[m].sample_us >
                                detail::SAMPLE_STALE_US)
                    {
                        detail::latch_fault(
                            fault_code::SAMPLE_TIMEOUT,
                            static_cast<uint8_t>(m));
                        return false;
                    }
                }

                const uint32_t phase = timer_phase();

                if(phase < 6400 || phase > 7440)
                {
                    return false;
                }

                const uint32_t levels[6] =
                {
                    value[0].ca, value[0].cb, value[0].cc,
                    value[1].ca, value[1].cb, value[1].cc
                };

                for(uint32_t i = 0; i < 6; ++i)
                {
                    if(levels[i] != queued[i] &&
                       mcpwm_comparator_set_compare_value(
                           comparators[i], levels[i]) != ESP_OK)
                    {
                        detail::latch_fault(fault_code::PWM_UPDATE);
                        return false;
                    }
                }

                const uint32_t end_phase = timer_phase();

                if(end_phase < phase || end_phase > 7840)
                {
                    detail::latch_fault(fault_code::PWM_UPDATE);
                    return false;
                }

                for(uint32_t i = 0; i < 6; ++i)
                {
                    queued[i] = levels[i];
                }

                return false;
            }

            /**
             * @brief 交替采集左右电机的两相电流
             *
             * @param[in] timer_handle 触发回调的定时器
             * @param[in] event_data 定时器事件数据
             * @param[in] user_data 用户数据
             *
             * @return false 不要求唤醒更高优先级任务
             */
            bool IRAM_ATTR on_sample(
                mcpwm_timer_handle_t,
                const mcpwm_timer_event_data_t *,
                void *)
            {
                static uint32_t turn = 0;
                static uint32_t sequence[2] = {};

                const uint32_t index = (turn++) & 1U;

                detail::sample out{};
                out.us = detail::now_us32();
                out.cycle = detail::epoch;
                out.seq = ++sequence[index];

                out.ca = applied[3U * index];
                out.cb = applied[3U * index + 1U];
                out.cc = applied[3U * index + 2U];

                const bool enabled =
                    __atomic_load_n(
                        &detail::running, __ATOMIC_ACQUIRE) &&
                    (__atomic_load_n(
                        &detail::active_mask, __ATOMIC_ACQUIRE) &
                     (1U << index));

                out.a0 = timer_phase();

                adc_channel_t channels[2]{};
                adc::get_channels(index, channels);

                const uint32_t start = esp_cpu_get_cycle_count();

                uint16_t raw_a = 0;
                uint16_t raw_b = 0;

                const bool ok_a = adc::read_raw(channels[0], raw_a);

                out.a = raw_a;
                out.a1 = out.a0 +
                    (esp_cpu_get_cycle_count() - start + 1U) / 2U;

                out.b0 = out.a1;

                const bool ok_b = adc::read_raw(channels[1], raw_b);

                out.b = raw_b;
                out.b1 = out.a0 +
                    (esp_cpu_get_cycle_count() - start + 1U) / 2U;

                out.bus = __atomic_load_n(
                    &detail::bus_raw, __ATOMIC_ACQUIRE);

                const bool window_a =
                    !enabled ||
                    (out.a0 >= out.ca + 160U &&
                     out.a1 + 160U <= detail::PWM_PERIOD - out.ca);

                const bool window_b =
                    !enabled ||
                    (out.b0 >= out.cb + 160U &&
                     out.b1 + 160U <= detail::PWM_PERIOD - out.cb);

                const bool raw_ok =
                    ok_a && ok_b &&
                    raw_a > 0 && raw_b > 0 &&
                    raw_a < 4095 && raw_b < 4095;

                const bool same_cycle =
                    out.cycle == detail::epoch &&
                    out.b1 < detail::PWM_PERIOD;

                out.flags =
                    (enabled ? 1U : 0U) |
                    (ok_a ? 2U : 0U) |
                    (ok_b ? 4U : 0U) |
                    (window_a ? 8U : 0U) |
                    (window_b ? 16U : 0U) |
                    (same_cycle ? 32U : 0U);

                const bool overcurrent =
                    adc::overcurrent(2U * index, raw_a) ||
                    adc::overcurrent(2U * index + 1U, raw_b);

                if(enabled && (!raw_ok || overcurrent))
                {
                    detail::latch_fault(
                        raw_ok ? fault_code::OVERCURRENT
                               : fault_code::ADC_READ,
                        static_cast<uint8_t>(index),
                        raw_a,
                        raw_b);
                }

                if(enabled && window_a && window_b &&
                   raw_ok && same_cycle)
                {
                    __atomic_store_n(
                        &detail::last_valid_sample_us[index],
                        out.us,
                        __ATOMIC_RELEASE);
                }

                publish_sample(index, out);

                return false;
            }
        }

        /**
         * @brief 强制关闭所有 PWM 输出并清除运行标志
         */
        void IRAM_ATTR force_off()
        {
            __atomic_store_n(
                &detail::requested, 0U, __ATOMIC_RELEASE);

            for(uint32_t i = 0; i < 6; ++i)
            {
                if(generators[i] != nullptr)
                {
                    mcpwm_generator_set_force_level(
                        generators[i], 0, true);
                }
            }

            __atomic_store_n(
                &detail::running, 0U, __ATOMIC_RELEASE);

            __atomic_store_n(
                &detail::enable_pending, 0U, __ATOMIC_RELEASE);

            __atomic_store_n(
                &detail::ready_flag, 0U, __ATOMIC_RELEASE);
        }

        /**
         * @brief 初始化六路互补 PWM、采样定时器与中断回调
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
            config.period_ticks = detail::PWM_PERIOD;
            config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
            config.intr_priority = 3;

            ESP_ERROR_CHECK(mcpwm_new_timer(&config, &timer));

            for(uint32_t op = 0; op < 3; ++op)
            {
                mcpwm_operator_config_t operator_config{};
                operator_config.group_id = 0;
                operator_config.intr_priority = 3;

                ESP_ERROR_CHECK(
                    mcpwm_new_operator(
                        &operator_config, &operators[op]));

                ESP_ERROR_CHECK(
                    mcpwm_operator_connect_timer(
                        operators[op], timer));

                for(uint32_t j = 0; j < 2; ++j)
                {
                    const uint32_t i = 2U * op + j;

                    mcpwm_comparator_config_t comparator_config{};
                    comparator_config.flags.update_cmp_on_tez = true;

                    ESP_ERROR_CHECK(
                        mcpwm_new_comparator(
                            operators[op],
                            &comparator_config,
                            &comparators[i]));

                    ESP_ERROR_CHECK(
                        mcpwm_comparator_set_compare_value(
                            comparators[i], detail::PWM_ZERO));

                    mcpwm_generator_config_t generator_config{};
                    generator_config.gen_gpio_num = PINS[i];

                    ESP_ERROR_CHECK(
                        mcpwm_new_generator(
                            operators[op],
                            &generator_config,
                            &generators[i]));

                    ESP_ERROR_CHECK(
                        mcpwm_generator_set_force_level(
                            generators[i], 0, true));

                    ESP_ERROR_CHECK(
                        mcpwm_generator_set_action_on_timer_event(
                            generators[i],
                            MCPWM_GEN_TIMER_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP,
                                MCPWM_TIMER_EVENT_EMPTY,
                                MCPWM_GEN_ACTION_HIGH)));

                    ESP_ERROR_CHECK(
                        mcpwm_generator_set_action_on_compare_event(
                            generators[i],
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP,
                                comparators[i],
                                MCPWM_GEN_ACTION_LOW)));

                    ESP_ERROR_CHECK(
                        mcpwm_generator_set_action_on_compare_event(
                            generators[i],
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_DOWN,
                                comparators[i],
                                MCPWM_GEN_ACTION_HIGH)));
                }
            }

            ESP_ERROR_CHECK(mcpwm_new_timer(&config, &sample_timer));

            mcpwm_timer_sync_src_config_t sync_config{};
            sync_config.timer_event = MCPWM_TIMER_EVENT_EMPTY;

            ESP_ERROR_CHECK(
                mcpwm_new_timer_sync_src(
                    timer, &sync_config, &sync_source));

            mcpwm_timer_sync_phase_config_t phase_config{};
            phase_config.sync_src = sync_source;
            phase_config.count_value = 1600;
            phase_config.direction = MCPWM_TIMER_DIRECTION_UP;

            ESP_ERROR_CHECK(
                mcpwm_timer_set_phase_on_sync(
                    sample_timer, &phase_config));

            mcpwm_timer_event_callbacks_t sample_callbacks{};
            sample_callbacks.on_full = on_sample;
            sample_callbacks.on_empty = on_commit;

            ESP_ERROR_CHECK(
                mcpwm_timer_register_event_callbacks(
                    sample_timer, &sample_callbacks, nullptr));

            mcpwm_timer_event_callbacks_t timer_callbacks{};
            timer_callbacks.on_empty = on_empty;
            timer_callbacks.on_full = on_peak;

            ESP_ERROR_CHECK(
                mcpwm_timer_register_event_callbacks(
                    timer, &timer_callbacks, nullptr));

            ESP_ERROR_CHECK(mcpwm_timer_enable(sample_timer));
            ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        }

        /**
         * @brief 启动 PWM 主定时器与采样定时器
         */
        void start()
        {
            ESP_ERROR_CHECK(
                mcpwm_timer_start_stop(
                    sample_timer, MCPWM_TIMER_START_NO_STOP));

            ESP_ERROR_CHECK(
                mcpwm_timer_start_stop(
                    timer, MCPWM_TIMER_START_NO_STOP));
        }

        /**
         * @brief 发布新的三相 PWM 命令
         *
         * @param[in] index 电机索引
         * @param[in] value 待提交命令
         */
        void set_command(
            uint32_t index,
            const detail::command &value)
        {
            publish_command(index, value);
        }

        /**
         * @brief 获取最新电流采样
         *
         * @param[in] index 电机索引
         * @param[out] out 采样数据
         *
         * @return true 获取成功
         * @return false 暂时无法取得一致性快照
         */
        bool get_sample(uint32_t index, detail::sample &out)
        {
            return read_sample(index, out);
        }
    }

    namespace foc
    {
        /**
         * @brief 执行逆 Park 变换、SVPWM 与占空比限幅
         *
         * @param[in] vd d 轴电压，单位 V
         * @param[in] vq q 轴电压，单位 V
         * @param[in] theta 电角度，单位 rad
         * @param[in] bus 母线电压，单位 V
         *
         * @return 三相 PWM 比较值
         */
        detail::command modulation(
            float vd,
            float vq,
            float theta,
            float bus)
        {
            const float cs = cosf(theta);
            const float sn = sinf(theta);

            const float alpha = vd * cs - vq * sn;
            const float beta = vd * sn + vq * cs;

            const float phase[3] =
            {
                alpha,
                -0.5f * alpha + 0.8660254f * beta,
                -0.5f * alpha - 0.8660254f * beta
            };

            const float maximum =
                fmaxf(phase[0], fmaxf(phase[1], phase[2]));

            const float minimum =
                fminf(phase[0], fminf(phase[1], phase[2]));

            const float common = -0.5f * (maximum + minimum);

            detail::command out{};

            uint32_t *compare[3] = {&out.ca, &out.cb, &out.cc};

            for(uint32_t i = 0; i < 3; ++i)
            {
                const float duty =
                    (0.5f + (phase[i] + common) / bus) *
                    4000.0f + 0.5f;

                const uint32_t value =
                    static_cast<uint32_t>(
                        duty < 0.0f ? 0.0f : duty);

                *compare[i] =
                    value < 320U ? 320U :
                    value > 3680U ? 3680U :
                    value;
            }

            return out;
        }
    }

    namespace control
    {
        namespace
        {
            enum class state : uint8_t
            {
                WAITING,
                ALIGN_LOCK,
                ALIGN_SWEEP,
                ALIGN_FINAL,
                ALIGN_STOP,
                RUN
            };

            /**
             * @brief 在后台任务中执行传感器轮询、校准与电流环
             *
             * @param[in] arg 未使用
             *
             * @note sensor::init() 与 sensor::poll() 在同一任务内执行，
             *       符合当前 sensor 模块的任务所有权约束。
             */
            void task(void *)
            {
                if(!sensor::init())
                {
                    detail::latch_fault(fault_code::SENSOR_INIT);

                    __atomic_store_n(
                        &detail::sensor_init_ok, 0U, __ATOMIC_RELEASE);

                    __atomic_store_n(
                        &detail::task_ready, 1U, __ATOMIC_RELEASE);

                    vTaskDelete(nullptr);
                    return;
                }

                __atomic_store_n(
                    &detail::sensor_init_ok, 1U, __ATOMIC_RELEASE);

                __atomic_store_n(
                    &detail::task_ready, 1U, __ATOMIC_RELEASE);

                state current_state = state::WAITING;

                uint32_t boot_us = detail::now_us32();
                uint32_t state_start_us = boot_us;

                uint32_t seen_sequence[2] = {};
                uint64_t seen_encoder_timestamp[2] = {};

                uint32_t previous_sample_us[2] = {};

                float integral_d[2] = {};
                float integral_q[2] = {};

                uint32_t motor_index = 0;
                uint32_t stable_count = 0;
                uint32_t angle_count = 0;

                float sweep_start_angle = 0.0f;
                double angle_sum = 0.0;

                while(true)
                {
                    uint32_t notifications = 0;

                    xTaskNotifyWait(
                        0, UINT32_MAX, &notifications, 0);

                    const bool allow_recovery =
                        !__atomic_load_n(
                            &detail::requested, __ATOMIC_ACQUIRE);

                    sensor::poll(notifications, allow_recovery);

                    const uint32_t now = detail::now_us32();

                    const uint64_t now64 =
                        static_cast<uint64_t>(esp_timer_get_time());

                    __atomic_store_n(
                        &detail::heartbeat_us, now, __ATOMIC_RELEASE);

                    sensor::package sensors{};
                    sensor::get_package(sensors);

                    const sensor::encoder_data encoders[2] =
                    {
                        sensors.left_encoder,
                        sensors.right_encoder
                    };

                    bool fresh_encoder[2] = {};

                    for(uint32_t m = 0; m < 2; ++m)
                    {
                        fresh_encoder[m] =
                            encoders[m].timestamp_us != 0 &&
                            encoders[m].timestamp_us !=
                                seen_encoder_timestamp[m];

                        if(fresh_encoder[m])
                        {
                            seen_encoder_timestamp[m] =
                                encoders[m].timestamp_us;
                        }
                    }

                    if(__atomic_load_n(
                        &detail::fault_value, __ATOMIC_ACQUIRE))
                    {
                        __atomic_store_n(
                            &detail::ready_flag, 0U, __ATOMIC_RELEASE);

                        taskYIELD();
                        continue;
                    }

                    const uint32_t raw_bus =
                        __atomic_load_n(
                            &detail::bus_raw, __ATOMIC_ACQUIRE);

                    const uint32_t bus_stamp =
                        __atomic_load_n(
                            &detail::bus_timestamp_us, __ATOMIC_ACQUIRE);

                    const float bus = adc::bus_voltage(raw_bus);

                    const bool bus_fresh =
                        bus_stamp != 0 &&
                        now - bus_stamp <= detail::BUS_STALE_US;

                    if(current_state == state::WAITING)
                    {
                        bool encoders_ready = true;

                        for(uint32_t m = 0; m < 2; ++m)
                        {
                            encoders_ready &=
                                encoders[m].timestamp_us != 0 &&
                                now64 >= encoders[m].timestamp_us &&
                                now64 - encoders[m].timestamp_us <=
                                    detail::ENCODER_STALE_US;
                        }

                        if(bus_fresh &&
                           bus >= 5.0f && bus <= 13.0f &&
                           encoders_ready)
                        {
                            motor_index = 0;
                            stable_count = 0;

                            __atomic_store_n(
                                &detail::active_mask, 1U, __ATOMIC_RELEASE);

                            __atomic_add_fetch(
                                &detail::run_tag, 1U, __ATOMIC_RELEASE);

                            __atomic_store_n(
                                &detail::last_valid_sample_us[0],
                                now,
                                __ATOMIC_RELEASE);

                            current_state = state::ALIGN_LOCK;
                            state_start_us = now;

                            __atomic_store_n(
                                &detail::requested, 1U, __ATOMIC_RELEASE);
                        }
                        else if(now - boot_us > 3000000)
                        {
                            detail::latch_fault(
                                bus_fresh ? fault_code::SENSOR_TIMEOUT
                                          : fault_code::BUS_VOLTAGE);
                        }
                    }

                    if(current_state == state::ALIGN_LOCK)
                    {
                        const uint32_t elapsed = now - state_start_us;

                        if(fresh_encoder[motor_index] &&
                           elapsed >= detail::ALIGN_SETTLE_US)
                        {
                            if(fabsf(
                                encoders[motor_index].speed_rad_s) <
                                detail::ALIGN_MAX_SPEED_RAD_S)
                            {
                                ++stable_count;
                            }
                            else
                            {
                                stable_count = 0;
                            }
                        }

                        if(elapsed >= detail::ALIGN_LOCK_US &&
                           stable_count >= 10)
                        {
                            sweep_start_angle =
                                encoders[motor_index].angle_rad;

                            current_state = state::ALIGN_SWEEP;
                            state_start_us = now;

                            __atomic_add_fetch(
                                &detail::run_tag, 1U, __ATOMIC_RELEASE);
                        }
                        else if(elapsed >
                                detail::ALIGN_LOCK_TIMEOUT_US)
                        {
                            detail::latch_fault(
                                fault_code::ALIGNMENT,
                                static_cast<uint8_t>(motor_index));
                        }
                    }
                    else if(current_state == state::ALIGN_SWEEP)
                    {
                        if(now - state_start_us >= detail::ALIGN_SWEEP_US)
                        {
                            const float delta =
                                encoders[motor_index].angle_rad -
                                sweep_start_angle;

                            if(fabsf(delta) < detail::ALIGN_MIN_DELTA_RAD ||
                               fabsf(delta) > detail::ALIGN_MAX_DELTA_RAD)
                            {
                                detail::latch_fault(
                                    fault_code::ALIGNMENT,
                                    static_cast<uint8_t>(motor_index));
                            }
                            else
                            {
                                detail::direction[motor_index] =
                                    delta > 0.0f ? 1.0f : -1.0f;

                                current_state = state::ALIGN_FINAL;
                                state_start_us = now;

                                stable_count = 0;
                                angle_count = 0;
                                angle_sum = 0.0;

                                __atomic_add_fetch(
                                    &detail::run_tag, 1U, __ATOMIC_RELEASE);
                            }
                        }
                    }
                    else if(current_state == state::ALIGN_FINAL)
                    {
                        const uint32_t elapsed = now - state_start_us;

                        if(fresh_encoder[motor_index] &&
                           elapsed >= detail::ALIGN_SETTLE_US)
                        {
                            if(fabsf(
                                encoders[motor_index].speed_rad_s) <
                                detail::ALIGN_MAX_SPEED_RAD_S)
                            {
                                angle_sum +=
                                    encoders[motor_index].angle_rad;

                                ++angle_count;
                                ++stable_count;
                            }
                            else
                            {
                                angle_sum = 0.0;
                                angle_count = 0;
                                stable_count = 0;
                            }
                        }

                        if(elapsed >= detail::ALIGN_FINAL_US &&
                           angle_count >= 10)
                        {
                            const float aligned_angle =
                                static_cast<float>(
                                    angle_sum /
                                    static_cast<double>(angle_count));

                            detail::zero_offset[motor_index] =
                                detail::wrap_angle(
                                    detail::direction[motor_index] *
                                    static_cast<float>(detail::POLE_PAIRS) *
                                    aligned_angle);

                            detail::zero_valid[motor_index] = true;

                            __atomic_store_n(
                                &detail::requested, 0U, __ATOMIC_RELEASE);

                            current_state = state::ALIGN_STOP;
                        }
                        else if(elapsed >
                                detail::ALIGN_FINAL_TIMEOUT_US)
                        {
                            detail::latch_fault(
                                fault_code::ALIGNMENT,
                                static_cast<uint8_t>(motor_index));
                        }
                    }

                    if(current_state == state::ALIGN_STOP &&
                       !__atomic_load_n(
                           &detail::running, __ATOMIC_ACQUIRE) &&
                       !__atomic_load_n(
                           &detail::enable_pending, __ATOMIC_ACQUIRE))
                    {
                        if(motor_index == 0)
                        {
                            motor_index = 1;
                            stable_count = 0;

                            __atomic_store_n(
                                &detail::active_mask, 2U, __ATOMIC_RELEASE);

                            __atomic_add_fetch(
                                &detail::run_tag, 1U, __ATOMIC_RELEASE);

                            __atomic_store_n(
                                &detail::last_valid_sample_us[1],
                                now,
                                __ATOMIC_RELEASE);

                            current_state = state::ALIGN_LOCK;
                            state_start_us = now;

                            __atomic_store_n(
                                &detail::requested, 1U, __ATOMIC_RELEASE);
                        }
                        else
                        {
                            integral_d[0] = integral_d[1] = 0.0f;
                            integral_q[0] = integral_q[1] = 0.0f;

                            previous_sample_us[0] = 0;
                            previous_sample_us[1] = 0;

                            __atomic_store_n(
                                &detail::active_mask, 3U, __ATOMIC_RELEASE);

                            __atomic_add_fetch(
                                &detail::run_tag, 1U, __ATOMIC_RELEASE);

                            current_state = state::RUN;
                            state_start_us = now;

                            __atomic_store_n(
                                &detail::last_valid_sample_us[0],
                                now,
                                __ATOMIC_RELEASE);

                            __atomic_store_n(
                                &detail::last_valid_sample_us[1],
                                now,
                                __ATOMIC_RELEASE);

                            __atomic_store_n(
                                &detail::requested, 1U, __ATOMIC_RELEASE);
                        }
                    }

                    if(current_state == state::RUN &&
                       __atomic_load_n(
                           &detail::running, __ATOMIC_ACQUIRE) &&
                       detail::zero_valid[0] &&
                       detail::zero_valid[1])
                    {
                        __atomic_store_n(
                            &detail::ready_flag, 1U, __ATOMIC_RELEASE);
                    }

                    for(uint32_t m = 0; m < 2; ++m)
                    {
                        detail::sample s{};

                        if(!pwm::get_sample(m, s) ||
                           s.seq == seen_sequence[m])
                        {
                            continue;
                        }

                        seen_sequence[m] = s.seq;

                        const bool calibrating =
                            (current_state == state::ALIGN_LOCK ||
                             current_state == state::ALIGN_SWEEP ||
                             current_state == state::ALIGN_FINAL) &&
                            m == motor_index;

                        const bool controlling =
                            current_state == state::RUN &&
                            __atomic_load_n(
                                &detail::running, __ATOMIC_ACQUIRE) &&
                            (__atomic_load_n(
                                &detail::active_mask, __ATOMIC_ACQUIRE) &
                             (1U << m));

                        if(!calibrating && !controlling)
                        {
                            continue;
                        }

                        if((s.flags & 63U) != 63U)
                        {
                            continue;
                        }

                        const uint32_t sampled_bus_stamp =
                            __atomic_load_n(
                                &detail::bus_timestamp_us,
                                __ATOMIC_ACQUIRE);

                        if(!sampled_bus_stamp ||
                           detail::now_us32() - sampled_bus_stamp >
                               detail::BUS_STALE_US)
                        {
                            detail::latch_fault(fault_code::BUS_STALE);
                            break;
                        }

                        const float measured_bus =
                            adc::bus_voltage(s.bus);

                        if(!isfinite(measured_bus) ||
                           measured_bus < 5.0f ||
                           measured_bus > 13.0f)
                        {
                            detail::latch_fault(fault_code::BUS_VOLTAGE);
                            break;
                        }

                        const uint32_t sample_age =
                            detail::now_us32() - s.us;

                        const sensor::encoder_data &encoder = encoders[m];

                        if(sample_age > detail::SAMPLE_STALE_US ||
                           !encoder.timestamp_us ||
                           now64 < encoder.timestamp_us ||
                           now64 - encoder.timestamp_us >
                               detail::ENCODER_STALE_US)
                        {
                            detail::latch_fault(
                                fault_code::SENSOR_TIMEOUT,
                                static_cast<uint8_t>(m));
                            break;
                        }

                        if(fabsf(encoder.speed_rad_s) >
                           detail::MAX_MOTOR_SPEED_RAD_S)
                        {
                            detail::latch_fault(
                                fault_code::SENSOR_TIMEOUT,
                                static_cast<uint8_t>(m));
                            break;
                        }

                        const float ia = adc::current(m * 2U, 
                            static_cast<uint16_t>(s.a), m);

                        const float ib = adc::current(m * 2U + 1U,
                            static_cast<uint16_t>(s.b), m);

                        const float ic = -ia - ib;
                        const float beta = (ia + 2.0f * ib) *
                                           detail::INV_SQRT3;

                        float theta = 0.0f;
                        float vd = 0.0f;
                        float vq = 0.0f;

                        if(calibrating)
                        {
                            const uint32_t elapsed =
                                s.us - state_start_us;

                            if(current_state == state::ALIGN_SWEEP)
                            {
                                const float progress = fminf(
                                    static_cast<float>(elapsed) /
                                        static_cast<float>(
                                            detail::ALIGN_SWEEP_US),
                                    1.0f);

                                theta = detail::TWO_PI * progress;
                                vd = detail::ALIGN_VOLTAGE_V;
                            }
                            else
                            {
                                theta = 0.0f;

                                vd = detail::ALIGN_VOLTAGE_V *
                                    fminf(
                                        static_cast<float>(elapsed) /
                                            50000.0f,
                                        1.0f);
                            }
                        }
                        else
                        {
                            const int32_t sensor_age_us =
                                static_cast<int32_t>(
                                    s.us -
                                    static_cast<uint32_t>(
                                        encoder.timestamp_us));

                            const float predicted_angle =
                                encoder.angle_rad +
                                encoder.speed_rad_s *
                                (static_cast<float>(sensor_age_us) *
                                 1.0e-6f);

                            theta = detail::wrap_angle(
                                detail::direction[m] *
                                static_cast<float>(detail::POLE_PAIRS) *
                                predicted_angle -
                                detail::zero_offset[m]);
                        }

                        const float cs = cosf(theta);
                        const float sn = sinf(theta);

                        const float id = ia * cs + beta * sn;
                        const float iq = beta * cs - ia * sn;

                        current_info current{};
                        current.timestamp_us =
                            now64 - sample_age;

                        current.electrical_angle_rad = theta;
                        current.mechanical_angle_rad =
                            encoder.angle_rad;

                        current.mechanical_speed_rad_s =
                            encoder.speed_rad_s;

                        current.phase_A[0] = ia;
                        current.phase_A[1] = ib;
                        current.phase_A[2] = ic;

                        current.id_A = id;
                        current.iq_A = iq;
                        current.valid = true;

                        detail::publish_current(m, current);

                        if(!calibrating)
                        {
                            const uint32_t dt_us =
                                previous_sample_us[m]
                                    ? s.us - previous_sample_us[m]
                                    : 200U;

                            previous_sample_us[m] = s.us;

                            if(dt_us == 0 ||
                               dt_us > detail::SAMPLE_STALE_US)
                            {
                                detail::latch_fault(
                                    fault_code::SAMPLE_TIMEOUT,
                                    static_cast<uint8_t>(m));
                                break;
                            }

                            const float dt =
                                static_cast<float>(dt_us) * 1.0e-6f;

                            const target_current target =
                                detail::get_target(m);

                            const float error_d = target.id_A - id;
                            const float error_q = target.iq_A - iq;

                            const float candidate_d =
                                integral_d[m] +
                                detail::CURRENT_KI * dt * error_d;

                            const float candidate_q =
                                integral_q[m] +
                                detail::CURRENT_KI * dt * error_q;

                            vd = detail::CURRENT_KP * error_d +
                                 candidate_d;

                            vq = detail::CURRENT_KP * error_q +
                                 candidate_q;

                            const float voltage_limit = fminf(
                                detail::VOLTAGE_LIMIT_V,
                                detail::BUS_VOLTAGE_RATIO_LIMIT *
                                    measured_bus);

                            const float magnitude =
                                sqrtf(vd * vd + vq * vq);

                            if(magnitude > voltage_limit)
                            {
                                const float scale =
                                    voltage_limit / magnitude;

                                vd *= scale;
                                vq *= scale;

                                integral_d[m] =
                                    vd - detail::CURRENT_KP * error_d;

                                integral_q[m] =
                                    vq - detail::CURRENT_KP * error_q;
                            }
                            else
                            {
                                integral_d[m] = candidate_d;
                                integral_q[m] = candidate_q;
                            }
                        }

                        detail::command output = foc::modulation(
                            vd, vq, theta, measured_bus);

                        output.tag = __atomic_load_n(
                            &detail::run_tag, __ATOMIC_ACQUIRE);

                        output.seq = s.seq;
                        output.sample_us = s.us;

                        pwm::set_command(m, output);
                    }

                    taskYIELD();
                }
            }
        }
    }

    /**
     * @brief 初始化整个电机模块
     *
     * @return true 初始化成功
     * @return false 任务创建或传感器初始化失败
     */
    bool init()
    {
        if(detail::init_started)
        {
            return __atomic_load_n(
                &detail::module_initialized, __ATOMIC_ACQUIRE) != 0;
        }

        detail::init_started = true;

        adc::init();
        pwm::init();

        if(xTaskCreatePinnedToCore(
            control::task,
            "motor_control",
            8192,
            nullptr,
            5,
            nullptr,
            1) != pdPASS)
        {
            return false;
        }

        while(!__atomic_load_n(
            &detail::task_ready, __ATOMIC_ACQUIRE))
        {
            vTaskDelay(1);
        }

        if(!__atomic_load_n(
            &detail::sensor_init_ok, __ATOMIC_ACQUIRE))
        {
            return false;
        }

        pwm::start();

        portENTER_CRITICAL(&detail::info_lock);
        detail::latest_info.initialized = true;
        portEXIT_CRITICAL(&detail::info_lock);

        __atomic_store_n(
            &detail::module_initialized, 1U, __ATOMIC_RELEASE);

        return true;
    }

    /**
     * @brief 设置指定电机的目标 d/q 轴电流
     *
     * @param[in] id 电机编号
     * @param[in] target 目标电流，单位 A
     *
     * @return true 参数有效且已接受
     * @return false 参数无效、模块未初始化或存在故障
     */
    bool set_target_current(
        motor_id id,
        const target_current &target)
    {
        const uint32_t index = static_cast<uint32_t>(id);

        if(index > 1 ||
           !isfinite(target.id_A) ||
           !isfinite(target.iq_A) ||
           target.id_A * target.id_A +
               target.iq_A * target.iq_A >
               detail::MAX_TARGET_CURRENT_A *
               detail::MAX_TARGET_CURRENT_A)
        {
            return false;
        }

        if(!__atomic_load_n(
            &detail::module_initialized, __ATOMIC_ACQUIRE) ||
           __atomic_load_n(
            &detail::fault_value, __ATOMIC_ACQUIRE))
        {
            return false;
        }

        portENTER_CRITICAL(&detail::info_lock);

        detail::targets[index] = target;
        detail::latest_info.target[index] = target;

        portEXIT_CRITICAL(&detail::info_lock);

        return true;
    }

    /**
     * @brief 获取电流环状态快照
     *
     * @param[out] snapshot 当前目标、采样数据、母线电压及故障信息
     *
     * @return true 已取得状态快照
     * @return false 模块尚未初始化
     */
    bool get_info(foc_info &snapshot)
    {
        if(!__atomic_load_n(
            &detail::module_initialized, __ATOMIC_ACQUIRE))
        {
            snapshot = foc_info{};
            return false;
        }

        portENTER_CRITICAL(&detail::info_lock);
        snapshot = detail::latest_info;
        portEXIT_CRITICAL(&detail::info_lock);

        const uint32_t code =
            __atomic_load_n(
                &detail::fault_value, __ATOMIC_ACQUIRE);

        snapshot.initialized = true;

        snapshot.ready =
            __atomic_load_n(
                &detail::ready_flag, __ATOMIC_ACQUIRE) != 0 &&
            code == 0;

        snapshot.fault.active = code != 0;
        snapshot.fault.code = static_cast<fault_code>(code);

        snapshot.fault.motor_index = static_cast<uint8_t>(
            __atomic_load_n(
                &detail::fault_motor, __ATOMIC_RELAXED));

        snapshot.fault.raw_a = static_cast<uint16_t>(
            __atomic_load_n(
                &detail::fault_raw_a, __ATOMIC_RELAXED));

        snapshot.fault.raw_b = static_cast<uint16_t>(
            __atomic_load_n(
                &detail::fault_raw_b, __ATOMIC_ACQUIRE));

        const uint32_t low =
            __atomic_load_n(
                &detail::fault_time_low, __ATOMIC_RELAXED);

        const uint32_t high =
            __atomic_load_n(
                &detail::fault_time_high, __ATOMIC_ACQUIRE);

        snapshot.fault.timestamp_us =
            (static_cast<uint64_t>(high) << 32) | low;

        const uint32_t raw =
            __atomic_load_n(
                &detail::bus_raw, __ATOMIC_ACQUIRE);

        const uint32_t bus_stamp =
            __atomic_load_n(
                &detail::bus_timestamp_us, __ATOMIC_ACQUIRE);

        const uint32_t now = detail::now_us32();

        if(bus_stamp != 0 &&
           now - bus_stamp <= detail::BUS_STALE_US)
        {
            snapshot.bus_voltage_V = adc::bus_voltage(raw);

            snapshot.bus_timestamp_us =
                static_cast<uint64_t>(esp_timer_get_time()) -
                static_cast<uint32_t>(now - bus_stamp);
        }
        else
        {
            snapshot.bus_voltage_V = 0.0f;
            snapshot.bus_timestamp_us = 0;
        }

        return true;
    }
}
