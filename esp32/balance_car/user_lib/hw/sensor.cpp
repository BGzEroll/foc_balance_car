#include "sensor.h"

#include "sys_time.h"
#include <math.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace i2c
{
    constexpr uint32_t LEFT_DONE = (1U << 0);
    constexpr uint32_t RIGHT_DONE = (1U << 1);

    namespace
    {
        constexpr i2c_port_num_t LEFT_PORT = 0;
        constexpr i2c_port_num_t RIGHT_PORT = 1;

        constexpr gpio_num_t LEFT_SDA = GPIO_NUM_37;
        constexpr gpio_num_t LEFT_SCL = GPIO_NUM_36;

        constexpr gpio_num_t RIGHT_SDA = GPIO_NUM_8;
        constexpr gpio_num_t RIGHT_SCL = GPIO_NUM_9;

        constexpr uint32_t LEFT_FREQ_HZ = 400000;
        constexpr uint32_t RIGHT_FREQ_HZ = 400000;

        constexpr uint16_t DEFAULT_ADDRESS = 0x36;

        struct context
        {
            i2c_master_bus_handle_t bus = nullptr;
            i2c_master_dev_handle_t dev = nullptr;

            uint32_t notify_bit = 0;
            volatile uint64_t completion_time_us = 0;
            uint64_t submitted_us = 0;
            bool pending = false;
        };

        context left;
        context right;

        TaskHandle_t task_handle = nullptr;

        /**
         * @brief I2C 异步事务完成回调
         *
         * @param[in] arg I2C 上下文
         *
         * @return true 已唤醒高优先级任务
         * @return false 未唤醒高优先级任务
         */
        bool done_callback(
            i2c_master_dev_handle_t,
            const i2c_master_event_data_t *,
            void *arg)
        {
            auto *ctx = static_cast<context *>(arg);

            ctx->completion_time_us = sys_time::get_us_tick();

            BaseType_t task_woken = pdFALSE;
            xTaskNotifyFromISR(
                task_handle,
                ctx->notify_bit,
                eSetBits,
                &task_woken);

            if(task_woken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }

            return task_woken == pdTRUE;
        }

        /**
         * @brief 初始化 I2C 总线
         *
         * @param[in, out] ctx I2C 上下文
         * @param[in] port I2C 控制器
         * @param[in] sda SDA 引脚
         * @param[in] scl SCL 引脚
         * @param[in] frequency 总线频率
         * @param[in] notify_bit 任务通知位
         */
        void init_bus(
            context &ctx,
            i2c_port_num_t port,
            gpio_num_t sda,
            gpio_num_t scl,
            uint32_t frequency,
            uint32_t notify_bit)
        {
            ctx.notify_bit = notify_bit;

            i2c_master_bus_config_t bus_config{};
            bus_config.i2c_port = port;
            bus_config.sda_io_num = sda;
            bus_config.scl_io_num = scl;
            bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            bus_config.glitch_ignore_cnt = 7;
            bus_config.trans_queue_depth = 1;

            ESP_ERROR_CHECK(
                i2c_new_master_bus(
                    &bus_config,
                    &ctx.bus));

            i2c_device_config_t dev_config{};
            dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_config.device_address = DEFAULT_ADDRESS;
            dev_config.scl_speed_hz = frequency;

            ESP_ERROR_CHECK(
                i2c_master_bus_add_device(
                    ctx.bus,
                    &dev_config,
                    &ctx.dev));

            i2c_master_event_callbacks_t callbacks{};
            callbacks.on_trans_done = done_callback;

            ESP_ERROR_CHECK(
                i2c_master_register_event_callbacks(
                    ctx.dev,
                    &callbacks,
                    &ctx));
        }
    }

    /**
     * @brief 初始化传感器 I2C
     *
     * @param[in] sensor_task Sensor 任务句柄
     */
    void init(TaskHandle_t sensor_task)
    {
        task_handle = sensor_task;

        init_bus(
            left,
            LEFT_PORT,
            LEFT_SDA,
            LEFT_SCL,
            LEFT_FREQ_HZ,
            LEFT_DONE);

        init_bus(
            right,
            RIGHT_PORT,
            RIGHT_SDA,
            RIGHT_SCL,
            RIGHT_FREQ_HZ,
            RIGHT_DONE);
    }

    /**
     * @brief 重建 I2C 总线及设备
     *
     * @param[in, out] ctx I2C 上下文
     *
     * @note 仅允许在所有者任务中调用
     */
    void rebuild(context &ctx)
    {
        ESP_ERROR_CHECK(i2c_master_bus_reset(ctx.bus));
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(ctx.dev));
        ESP_ERROR_CHECK(i2c_del_master_bus(ctx.bus));

        ctx.bus = nullptr;
        ctx.dev = nullptr;
        ctx.pending = false;
        ctx.completion_time_us = 0;
        ctx.submitted_us = 0;

        ulTaskNotifyValueClear(nullptr, ctx.notify_bit);

        if(&ctx == &left)
        {
            init_bus(
                ctx,
                LEFT_PORT,
                LEFT_SDA,
                LEFT_SCL,
                LEFT_FREQ_HZ,
                LEFT_DONE);
        }
        else
        {
            init_bus(
                ctx,
                RIGHT_PORT,
                RIGHT_SDA,
                RIGHT_SCL,
                RIGHT_FREQ_HZ,
                RIGHT_DONE);
        }
    }

    /**
     * @brief 修改右侧 I2C 设备地址
     *
     * @param[in] address 设备地址
     */
    void set_right_address(uint16_t address)
    {
        ESP_ERROR_CHECK(
            i2c_master_device_change_address(
                right.dev,
                address,
                -1));
    }

    /**
     * @brief 发起传感器 I2C 异步读取
     *
     * @param[in, out] ctx 左侧或右侧总线上下文
     * @param[in] reg 寄存器地址
     * @param[out] data 接收缓冲区
     * @param[in] size 读取长度
     */
    void read(context &ctx, const uint8_t *reg, uint8_t *data, size_t size)
    {

        ctx.submitted_us = sys_time::get_us_tick();
        ctx.pending = true;

        // 超时由 sensor::poll() 统一检测与恢复
        // 不能在这里等待事务完成
        i2c_master_transmit_receive(
            ctx.dev,
            reg,
            1,
            data,
            size,
            0);
    }

    /**
     * @brief 同步写入右侧 I2C 单寄存器
     *
     * @param[in] reg 寄存器地址
     * @param[in] value 寄存器值
     *
     * @note 仅用于 IMU 初始化
     */
    void write_right_sync(uint8_t reg, uint8_t value)
    {
        uint8_t data[2] = {reg, value};

        for(uint8_t attempt = 0; attempt < 3; ++attempt)
        {
            ESP_ERROR_CHECK(
                i2c_master_transmit(
                    right.dev,
                    data,
                    sizeof(data),
                    20));

            uint32_t notification = 0;

            if(xTaskNotifyWait(
                    0,
                    RIGHT_DONE,
                    &notification,
                    pdMS_TO_TICKS(20)) == pdTRUE &&
                (notification & RIGHT_DONE))
            {
                return;
            }

            rebuild(right);
            set_right_address(0x68);
        }

        ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
    }
}

namespace as5600
{
    constexpr uint16_t ADDRESS = 0x36;

    namespace
    {
        constexpr uint8_t REG_RAW_ANGLE = 0x0C;

        constexpr int32_t RESOLUTION = 4096;
        constexpr int32_t HALF_RESOLUTION = RESOLUTION / 2;

        constexpr float TWO_PI = 6.28318530717958647692f;
        constexpr float COUNT_TO_RAD = TWO_PI / static_cast<float>(RESOLUTION);
        constexpr float OBSERVER_TAU_S = 0.004f;

        struct state
        {
            uint8_t reg = REG_RAW_ANGLE;
            uint8_t raw[2] = {};

            bool first_sample = true;

            uint16_t last_raw = 0;
            uint64_t last_time_us = 0;

            int32_t turns = 0;
            float speed_rad_s = 0.0f;
            float observer_error_rad = 0.0f;
        };

        state left;
        state right;

        /**
         * @brief 处理一次 AS5600 采样
         *
         * @param[in, out] encoder 编码器状态
         * @param[in] now_us 采样完成时间
         *
         * @return 编码器数据
         */
        sensor::encoder_data process(state &encoder, uint64_t now_us)
        {
            const uint16_t raw =
                static_cast<uint16_t>(
                    ((static_cast<uint16_t>(encoder.raw[0]) & 0x0F) << 8) |
                    encoder.raw[1]);

            if(encoder.first_sample)
            {
                encoder.first_sample = false;

                encoder.last_raw = raw;
                encoder.last_time_us = now_us;

                encoder.turns = 0;
                encoder.speed_rad_s = 0.0f;
            }
            else
            {
                int32_t delta =
                    static_cast<int32_t>(raw) -
                    static_cast<int32_t>(encoder.last_raw);

                if(delta > HALF_RESOLUTION)
                {
                    delta -= RESOLUTION;
                    --encoder.turns;
                }
                else if(delta < -HALF_RESOLUTION)
                {
                    delta += RESOLUTION;
                    ++encoder.turns;
                }

                const uint64_t dt_us = now_us - encoder.last_time_us;

                if(dt_us != 0)
                {
                    const float dt = static_cast<float>(dt_us) * 1.0e-6f;
                    const float delta_rad = static_cast<float>(delta) * COUNT_TO_RAD;

                    const float pole = OBSERVER_TAU_S /
                        (OBSERVER_TAU_S + 2.0f * dt);

                    const float beta = (1.0f - pole) * (1.0f - pole);

                    const float prediction =
                        encoder.observer_error_rad + dt * encoder.speed_rad_s;

                    const float residual = delta_rad - prediction;

                    encoder.observer_error_rad = -pole * pole * residual;
                    encoder.speed_rad_s += beta / dt * residual;
                }

                encoder.last_raw = raw;
                encoder.last_time_us = now_us;
            }

            sensor::encoder_data data;
            data.timestamp_us = now_us;
            data.single_turn_rad = static_cast<float>(raw) * COUNT_TO_RAD;
            data.angle_rad = static_cast<float>(encoder.turns) * TWO_PI +
                             data.single_turn_rad;
            data.speed_rad_s = encoder.speed_rad_s;

            return data;
        }
    }

    /**
     * @brief 发起左编码器读取
     */
    void start_left_read()
    {
        i2c::read(
            i2c::left,
            &left.reg,
            left.raw,
            sizeof(left.raw));
    }

    /**
     * @brief 发起右编码器读取
     */
    void start_right_read()
    {
        i2c::read(
            i2c::right,
            &right.reg,
            right.raw,
            sizeof(right.raw));
    }
}

namespace mpu6050
{
    constexpr uint16_t ADDRESS = 0x68;

    namespace
    {
        constexpr uint8_t REG_SMPLRT_DIV = 0x19;
        constexpr uint8_t REG_CONFIG = 0x1A;
        constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
        constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
        constexpr uint8_t REG_DATA = 0x3B;
        constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;

        constexpr uint32_t CALIBRATION_SAMPLES = 1000;

        constexpr float DEG2RAD = 0.01745329251994329577f;
        constexpr float WHEEL_RADIUS_M = 0.0325f;
        constexpr float WHEEL_FACTOR = WHEEL_RADIUS_M / 0.1325364f;

        uint8_t reg = REG_DATA;
        uint8_t raw[14] = {};

        int64_t gyro_offset_sum[3] = {};
        float gyro_offset[3] = {};

        uint32_t calibration_count = 0;

        /**
         * @brief 读取大端有符号 16 位数据
         *
         * @param[in] data 原始数据
         *
         * @return 转换后的数值
         */
        int16_t read_i16(const uint8_t *data)
        {
            return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                static_cast<uint16_t>(data[1]));
        }

        /**
         * @brief 处理陀螺仪零偏校准
         *
         * @param[in] raw_gyro 原始陀螺仪数据
         *
         * @return true 校准完成
         * @return false 校准进行中
         */
        bool process_calibration(const int16_t raw_gyro[3])
        {
            if(calibration_count >= CALIBRATION_SAMPLES)
            {
                return true;
            }

            for(uint8_t i = 0; i < 3; i++)
            {
                gyro_offset_sum[i] += raw_gyro[i];
            }

            calibration_count++;

            if(calibration_count < CALIBRATION_SAMPLES)
            {
                return false;
            }

            for(uint8_t i = 0; i < 3; i++)
            {
                const float raw_offset =
                    static_cast<float>(gyro_offset_sum[i]) /
                    static_cast<float>(CALIBRATION_SAMPLES);

                gyro_offset[i] =
                    raw_offset /
                    65.5f *
                    DEG2RAD;
            }

            return false;
        }

        // 误差状态为世界坐标姿态误差 XYZ 和机体系陀螺仪零偏 XYZ
        struct attitude_filter
        {
            float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
            float bias[3] = {};
            float covariance[6][6] = {};

            float gravity_norm = 1.0f;
            bool initialized = false;

            /**
             * @brief 计算 Hamilton 四元数乘积
             *
             * @param[in] a 左侧四元数
             * @param[in] b 右侧四元数
             * @param[out] out 四元数乘积，允许与输入共用缓冲区
             */
            static void multiply(const float a[4], const float b[4], float out[4])
            {
                const float result[4] =
                {
                    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
                };

                for(uint8_t i = 0; i < 4; ++i)
                {
                    out[i] = result[i];
                }
            }

            /**
             * @brief 将旋转向量转换为单位四元数
             *
             * @param[in] vector 旋转向量，单位 rad
             * @param[out] out 对应的四元数
             */
            static void rotation(const float vector[3], float out[4])
            {
                const float norm = sqrtf(
                    vector[0] * vector[0] +
                    vector[1] * vector[1] +
                    vector[2] * vector[2]);

                const float scale = norm > 1.0e-7f ?
                    sinf(0.5f * norm) / norm : 0.5f;

                out[0] = cosf(0.5f * norm);

                for(uint8_t i = 0; i < 3; ++i)
                {
                    out[i + 1] = scale * vector[i];
                }
            }

            /**
             * @brief 计算 3×3 矩阵逆矩阵
             *
             * @param[in] a 待求逆矩阵
             * @param[out] out 逆矩阵
             *
             * @return true 求逆成功
             * @return false 矩阵行列式非正
             */
            static bool invert(const float a[3][3], float out[3][3])
            {
                const float det =
                    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                    a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                    a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

                if(!(det > 0.0f))
                {
                    return false;
                }

                out[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det;
                out[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det;
                out[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;

                out[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det;
                out[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
                out[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det;

                out[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det;
                out[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det;
                out[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det;

                return true;
            }

            /**
             * @brief 将当前姿态四元数归一化
             */
            void normalize()
            {
                float norm = 0.0f;

                for(float value : q)
                {
                    norm += value * value;
                }

                const float scale = 1.0f / sqrtf(norm);

                for(float &value : q)
                {
                    value *= scale;
                }
            }

            /**
             * @brief 将当前姿态四元数转换为 XYZ 欧拉角
             *
             * @param[out] out 绕 X、Y、Z 轴的欧拉角，单位 rad
             */
            void euler(float out[3]) const
            {
                out[0] = atan2f(
                    2.0f * (q[0] * q[1] + q[2] * q[3]),
                    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));

                const float sine = fmaxf(-1.0f, fminf(1.0f,
                    2.0f * (q[0] * q[2] - q[3] * q[1])));

                out[1] = asinf(sine);

                out[2] = atan2f(
                    2.0f * (q[0] * q[3] + q[1] * q[2]),
                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
            }

            /**
             * @brief 获取机体系下的单位重力方向
             *
             * @param[out] out 机体系重力方向向量
             */
            void gravity(float out[3]) const
            {
                out[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
                out[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
                out[2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
            }

            /**
             * @brief 计算从机体系到世界系的旋转矩阵
             *
             * @param[out] r 3×3 旋转矩阵
             */
            void matrix(float r[3][3]) const
            {
                const float w = q[0];
                const float x = q[1];
                const float y = q[2];
                const float z = q[3];

                r[0][0] = 1.0f - 2.0f * (y * y + z * z);
                r[0][1] = 2.0f * (x * y - w * z);
                r[0][2] = 2.0f * (x * z + w * y);

                r[1][0] = 2.0f * (x * y + w * z);
                r[1][1] = 1.0f - 2.0f * (x * x + z * z);
                r[1][2] = 2.0f * (y * z - w * x);

                r[2][0] = 2.0f * (x * z - w * y);
                r[2][1] = 2.0f * (y * z + w * x);
                r[2][2] = 1.0f - 2.0f * (x * x + y * y);
            }

            /**
             * @brief 根据重力方向初始化姿态与协方差
             *
             * @param[in] acc 三轴加速度，单位 g
             *
             * @return true 初始化成功
             * @return false 重力数据无效
             */
            bool initialize(const float acc[3])
            {
                gravity_norm = sqrtf(
                    acc[0] * acc[0] +
                    acc[1] * acc[1] +
                    acc[2] * acc[2]);

                if(!(gravity_norm > 0.1f) || !isfinite(gravity_norm))
                {
                    return false;
                }

                const float roll = atan2f(acc[1], acc[2]);
                const float pitch = atan2f(-acc[0], hypotf(acc[1], acc[2]));

                const float cx = cosf(roll * 0.5f);
                const float sx = sinf(roll * 0.5f);
                const float cy = cosf(pitch * 0.5f);
                const float sy = sinf(pitch * 0.5f);

                q[0] = cy * cx;
                q[1] = cy * sx;
                q[2] = sy * cx;
                q[3] = -sy * sx;

                for(uint8_t i = 0; i < 6; ++i)
                {
                    covariance[i][i] = i < 3 ? 0.0009f : 7.615435e-7f;
                }

                initialized = true;
                return true;
            }

            /**
             * @brief 按状态变换矩阵传播误差协方差
             *
             * @param[in] f 六状态变换矩阵
             */
            void transform_covariance(const float f[6][6])
            {
                float temp[6][6] = {};
                float next[6][6] = {};

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        for(uint8_t k = 0; k < 6; ++k)
                        {
                            temp[i][j] += f[i][k] * covariance[k][j];
                        }
                    }
                }

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        for(uint8_t k = 0; k < 6; ++k)
                        {
                            next[i][j] += temp[i][k] * f[j][k];
                        }
                    }
                }

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        covariance[i][j] = next[i][j];
                    }
                }
            }

            /**
             * @brief 根据重力方向观测执行 ESKF 卡尔曼修正
             *
             * @param[in] h 三维观测雅可比矩阵
             * @param[in] residual 三维观测残差
             * @param[in] variance 观测噪声方差
             *
             * @return true 修正成功
             * @return false 创新协方差无效或超出 NIS 门限
             */
            bool correct(
                const float h[3][6],
                const float residual[3],
                float variance)
            {
                float ph[6][3] = {};
                float s[3][3] = {};
                float inverse[3][3];
                float gain[6][3] = {};

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 3; ++j)
                    {
                        for(uint8_t k = 0; k < 6; ++k)
                        {
                            ph[i][j] += covariance[i][k] * h[j][k];
                        }
                    }
                }

                for(uint8_t i = 0; i < 3; ++i)
                {
                    for(uint8_t j = 0; j < 3; ++j)
                    {
                        for(uint8_t k = 0; k < 6; ++k)
                        {
                            s[i][j] += h[i][k] * ph[k][j];
                        }

                        if(i == j)
                        {
                            s[i][j] += variance;
                        }
                    }
                }

                if(!invert(s, inverse))
                {
                    return false;
                }

                float nis = 0.0f;

                for(uint8_t i = 0; i < 3; ++i)
                {
                    for(uint8_t j = 0; j < 3; ++j)
                    {
                        nis += residual[i] * inverse[i][j] * residual[j];
                    }
                }

                if(!(nis <= 9.21f))
                {
                    return false;
                }

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 3; ++j)
                    {
                        for(uint8_t k = 0; k < 3; ++k)
                        {
                            gain[i][j] += ph[i][k] * inverse[k][j];
                        }
                    }
                }

                // 重力仅约束倾斜方向，不直接修正世界坐标系 Yaw
                float g[3];
                gravity(g);

                for(uint8_t j = 0; j < 3; ++j)
                {
                    gain[2][j] = 0.0f;
                    float along = 0.0f;

                    for(uint8_t i = 0; i < 3; ++i)
                    {
                        along += g[i] * gain[i + 3][j];
                    }

                    for(uint8_t i = 0; i < 3; ++i)
                    {
                        gain[i + 3][j] -= g[i] * along;
                    }
                }

                float dx[6] = {};

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 3; ++j)
                    {
                        dx[i] += gain[i][j] * residual[j];
                    }
                }

                // Joseph 协方差修正
                float a[6][6] = {};
                float ap[6][6] = {};
                float next[6][6] = {};

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        a[i][j] = i == j ? 1.0f : 0.0f;

                        for(uint8_t k = 0; k < 3; ++k)
                        {
                            a[i][j] -= gain[i][k] * h[k][j];
                        }
                    }
                }

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        for(uint8_t k = 0; k < 6; ++k)
                        {
                            ap[i][j] += a[i][k] * covariance[k][j];
                        }
                    }
                }

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        for(uint8_t k = 0; k < 6; ++k)
                        {
                            next[i][j] += ap[i][k] * a[j][k];
                        }

                        for(uint8_t k = 0; k < 3; ++k)
                        {
                            next[i][j] += variance * gain[i][k] * gain[j][k];
                        }
                    }
                }

                for(uint8_t i = 0; i < 6; ++i)
                {
                    for(uint8_t j = 0; j < 6; ++j)
                    {
                        covariance[i][j] = 0.5f * (next[i][j] + next[j][i]);
                    }
                }

                float dq[4];
                rotation(dx, dq);
                multiply(dq, q, q);
                normalize();

                for(uint8_t i = 0; i < 3; ++i)
                {
                    bias[i] += dx[i + 3];
                }

                // 修正后的误差坐标重置
                float j[6][6] = {};

                for(uint8_t i = 0; i < 6; ++i)
                {
                    j[i][i] = 1.0f;
                }

                j[0][1] -= 0.5f * dx[2];
                j[0][2] += 0.5f * dx[1];
                j[1][0] += 0.5f * dx[2];
                j[1][2] -= 0.5f * dx[0];
                j[2][0] -= 0.5f * dx[1];
                j[2][1] += 0.5f * dx[0];

                transform_covariance(j);
                return true;
            }

            /**
             * @brief 预测姿态并利用重力观测修正误差状态
             *
             * @param[in] acc 三轴加速度，单位 g
             * @param[in] gyro 三轴角速度，单位 rad/s
             * @param[in] dt 采样时间间隔，单位 s
             *
             * @return true 更新成功
             * @return false 重力数据初始化失败
             */
            bool update(
                const float acc[3],
                const float gyro[3],
                float dt)
            {
                if(!initialized)
                {
                    return initialize(acc);
                }

                float increment[3];

                for(uint8_t i = 0; i < 3; ++i)
                {
                    increment[i] = (gyro[i] - bias[i]) * dt;
                }

                float dq[4];
                rotation(increment, dq);
                multiply(q, dq, q);
                normalize();

                float f[6][6] = {};
                float r[3][3];
                matrix(r);

                for(uint8_t i = 0; i < 6; ++i)
                {
                    f[i][i] = 1.0f;
                }

                for(uint8_t i = 0; i < 3; ++i)
                {
                    for(uint8_t j = 0; j < 3; ++j)
                    {
                        f[i][j + 3] = -r[i][j] * dt;
                    }
                }

                transform_covariance(f);

                for(uint8_t i = 0; i < 3; ++i)
                {
                    covariance[i][i] += 0.00070f * 0.00070f * dt * dt;
                    covariance[i + 3][i + 3] += 5.0e-6f * 5.0e-6f * dt;
                }

                const float norm = sqrtf(
                    acc[0] * acc[0] +
                    acc[1] * acc[1] +
                    acc[2] * acc[2]);

                if(norm > 0.0f &&
                   fabsf(norm / gravity_norm - 1.0f) < 0.08f)
                {
                    float g[3];
                    float h[3][6] = {};
                    float residual[3];
                    gravity(g);

                    for(uint8_t i = 0; i < 3; ++i)
                    {
                        residual[i] = acc[i] / norm - g[i];
                        h[i][0] = r[1][i];
                        h[i][1] = -r[0][i];
                    }

                    correct(h, residual, 0.003f * 0.003f);
                }

                return true;
            }
        };

        struct yaw_observer
        {
            float bias = 0.0f;

            float left = 0.0f;
            float right = 0.0f;
            float base_left = 0.0f;
            float base_right = 0.0f;
            float base_yaw = 0.0f;
            float encoder_yaw = 0.0f;

            float seconds = 0.0f;
            float gyro_integral = 0.0f;
            float travel = 0.0f;
            float window_angle = 0.0f;
            float minimum = 0.0f;
            float maximum = 0.0f;

            bool initialized = false;
            bool candidate = true;

            /**
             * @brief 开始新的编码器 Yaw 零偏辨识窗口
             */
            void begin_window()
            {
                seconds = 0.0f;
                gyro_integral = 0.0f;
                travel = 0.0f;
                window_angle = encoder_yaw;
                minimum = encoder_yaw;
                maximum = encoder_yaw;
                candidate = true;
            }

            /**
             * @brief 根据轮运动约束及陀螺仪角速度学习 Yaw 零偏
             *
             * @param[in] yaw 当前车体相对 Yaw 角，单位 rad
             * @param[in] rate 当前车体 Yaw 角速度，单位 rad/s
             * @param[in] wheel_left 左轮正方向累计角度，单位 rad
             * @param[in] wheel_right 右轮正方向累计角度，单位 rad
             * @param[in] dt 采样时间间隔，单位 s
             * @param[in] allow_learning 当前是否允许学习新的零偏
             */
            void update(
                float yaw,
                float rate,
                float wheel_left,
                float wheel_right,
                float dt,
                bool allow_learning)
            {
                if(!initialized)
                {
                    initialized = true;
                    base_yaw = yaw;
                    encoder_yaw = yaw;
                    base_left = wheel_left;
                    base_right = wheel_right;
                    left = wheel_left;
                    right = wheel_right;
                    begin_window();
                    return;
                }

                const float dl = wheel_left - left;
                const float dr = wheel_right - right;
                left = wheel_left;
                right = wheel_right;

                encoder_yaw = base_yaw + WHEEL_FACTOR *
                    ((wheel_right - base_right) - (wheel_left - base_left));

                seconds += dt;
                gyro_integral += rate * dt;
                travel += WHEEL_RADIUS_M * (fabsf(dl) + fabsf(dr)) * 0.5f;
                minimum = fminf(minimum, encoder_yaw);
                maximum = fmaxf(maximum, encoder_yaw);
                candidate = candidate && allow_learning && fabsf(rate - bias) < 0.02f;

                if(seconds >= 1.0f)
                {
                    const float mean = gyro_integral / seconds;

                    if(candidate &&
                       fabsf(encoder_yaw - window_angle) < 0.00004712389f &&
                       maximum - minimum < 0.000872664626f &&
                       travel < 0.005f &&
                       fabsf(mean - bias) < 0.001745329252f)
                    {
                        bias += (1.0f - expf(-seconds / 15.0f)) * (mean - bias);
                    }

                    begin_window();
                }
            }
        };

        attitude_filter estimator;
        yaw_observer heading;
        uint64_t previous_attitude_us = 0;
    }

    /**
     * @brief 初始化 MPU6050
     */
    void init()
    {
        i2c::set_right_address(ADDRESS);

        // PLL X gyro clock
        i2c::write_right_sync(REG_PWR_MGMT_1, 0x01);

        sys_time::delay_ms(10);

        // DLPF_CFG = 3，带宽约 44 Hz
        i2c::write_right_sync(REG_CONFIG, 0x03);

        // 1000 / (1 + 0) = 1000 Hz
        i2c::write_right_sync(REG_SMPLRT_DIV, 0x00);

        // ±500 deg/s，65.5 LSB/(deg/s)
        i2c::write_right_sync(REG_GYRO_CONFIG, 0x08);

        // ±2 g，16384 LSB/g
        i2c::write_right_sync(REG_ACCEL_CONFIG, 0x00);

        i2c::set_right_address(as5600::ADDRESS);
    }

    /**
     * @brief 发起 MPU6050 数据读取
     */
    void start_read()
    {
        i2c::read(
            i2c::right,
            &reg,
            raw,
            sizeof(raw));
    }

    /**
     * @brief 处理一次 MPU6050 采样
     *
     * @param[in] now_us 采样完成时间
     * @param[out] data IMU 数据
     *
     * @return true 数据有效
     * @return false 陀螺仪仍在校准
     */
    bool process(uint64_t now_us, sensor::imu_data &data)
    {
        int16_t raw_acc[3];
        int16_t raw_gyro[3];

        raw_acc[0] = read_i16(&raw[0]);
        raw_acc[1] = read_i16(&raw[2]);
        raw_acc[2] = read_i16(&raw[4]);

        const int16_t raw_temperature = read_i16(&raw[6]);

        raw_gyro[0] = read_i16(&raw[8]);
        raw_gyro[1] = read_i16(&raw[10]);
        raw_gyro[2] = read_i16(&raw[12]);

        // 坐标系调整为 X 向右，Y 向前，Z 向上
        raw_acc[0] = -raw_acc[0];
        raw_acc[1] = -raw_acc[1];
        raw_gyro[0] = -raw_gyro[0];
        raw_gyro[1] = -raw_gyro[1];

        if(!process_calibration(raw_gyro))
        {
            return false;
        }

        data.timestamp_us = now_us;

        data.temperature =
            static_cast<float>(raw_temperature) /
            340.0f +
            36.53f;

        for(uint8_t i = 0; i < 3; i++)
        {
            data.acc[i] =
                static_cast<float>(raw_acc[i]) /
                16384.0f;

            data.gyro[i] =
                static_cast<float>(raw_gyro[i]) /
                65.5f *
                DEG2RAD -
                gyro_offset[i];
        }

        return true;
    }

    /**
     * @brief 重新初始化姿态，保留已估计的陀螺仪零偏
     *
     * @note 仅允许姿态控制任务调用
     */
    void reset_attitude()
    {
        const float bias[3] =
        {
            estimator.bias[0],
            estimator.bias[1],
            estimator.bias[2]
        };

        estimator = attitude_filter{};

        for(uint8_t i = 0; i < 3; ++i)
        {
            estimator.bias[i] = bias[i];
        }

        heading.initialized = false;
        previous_attitude_us = 0;
    }

    /**
     * @brief 根据 IMU 与编码器观测更新姿态及陀螺仪零偏
     *
     * @param[in, out] snapshot 传感器快照，输出姿态角和修正角速度
     * @param[in] wheel_angle_rad 左右轮正方向累计角度，单位 rad
     * @param[in] grounded 轮地运动信息是否有效
     * @param[in] allow_yaw_learning 是否允许学习 Yaw 零偏
     *
     * @return true 姿态更新成功
     * @return false IMU 数据无效、重复或姿态初始化失败
     *
     * @note 仅允许姿态控制任务调用
     */
    bool update_attitude(
        sensor::package &snapshot,
        const float wheel_angle_rad[2],
        bool grounded,
        bool allow_yaw_learning)
    {
        sensor::imu_data &imu = snapshot.imu;

        if(imu.timestamp_us == 0 ||
           imu.timestamp_us <= previous_attitude_us)
        {
            return false;
        }

        for(uint8_t i = 0; i < 3; ++i)
        {
            if(!isfinite(imu.acc[i]) || !isfinite(imu.gyro[i]))
            {
                return false;
            }
        }

        if(grounded &&
           (!isfinite(wheel_angle_rad[0]) || !isfinite(wheel_angle_rad[1])))
        {
            return false;
        }

        const float dt = previous_attitude_us ?
            static_cast<float>(imu.timestamp_us - previous_attitude_us) * 1.0e-6f :
            0.005f;

        if(!(dt > 0.0f) || !isfinite(dt))
        {
            return false;
        }

        previous_attitude_us = imu.timestamp_us;

        const bool first_sample = !estimator.initialized;

        if(first_sample && !estimator.initialize(imu.acc))
        {
            return false;
        }

        float angle[3];
        float gravity[3];
        float corrected_gyro[3];

        estimator.euler(angle);
        estimator.gravity(gravity);

        for(uint8_t i = 0; i < 3; ++i)
        {
            corrected_gyro[i] = imu.gyro[i] - estimator.bias[i];
        }

        if(grounded)
        {
            const float sx = sinf(angle[0]);
            const float cx = cosf(angle[0]);
            const float sy = sinf(angle[1]);
            const float cy = cosf(angle[1]);

            const float roll = atan2f(sy, cy * cx);
            const float pitch = atan2f(-cy * sx, hypotf(cx, sy * sx));
            const float yaw = angle[2] + atan2f(-sy * sx, cx);

            const float rate =
                (-sinf(roll) * corrected_gyro[0] +
                  cosf(roll) * corrected_gyro[2]) /
                fmaxf(0.2f, cosf(pitch));

            heading.update(
                yaw,
                rate,
                wheel_angle_rad[0],
                wheel_angle_rad[1],
                dt,
                allow_yaw_learning);
        }
        else
        {
            heading.initialized = false;
        }

        const float correction = grounded ? heading.bias : 0.0f;
        float gyro_input[3];

        for(uint8_t i = 0; i < 3; ++i)
        {
            gyro_input[i] = imu.gyro[i] - correction * gravity[i];
        }

        if(!first_sample)
        {
            estimator.update(imu.acc, gyro_input, dt);
        }

        estimator.euler(imu.angle);
        estimator.gravity(gravity);

        for(uint8_t i = 0; i < 3; ++i)
        {
            imu.gyro[i] -= estimator.bias[i] + correction * gravity[i];
        }

        return true;
    }
}

namespace sensor
{
    namespace
    {
        constexpr uint64_t MPU_PERIOD_US = 5000;
        constexpr uint64_t I2C_TIMEOUT_US = 2000;

        portMUX_TYPE package_lock = portMUX_INITIALIZER_UNLOCKED;

        package latest_package{};

        TaskHandle_t owner_task = nullptr;
        bool initialized = false;
        TaskHandle_t control_task = nullptr;

        bool right_reading_imu = false;
        uint64_t next_imu_time_us = 0;

        /**
         * @brief 发布左编码器数据
         *
         * @param[in] data 左编码器数据
         */
        void publish_left(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.left_encoder = data;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 发布右编码器数据
         *
         * @param[in] data 右编码器数据
         */
        void publish_right(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.right_encoder = data;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 发布 IMU 数据并通知控制任务
         *
         * @param[in] data IMU 数据
         */
        void publish_imu(const imu_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.imu = data;
            portEXIT_CRITICAL(&package_lock);

            TaskHandle_t task =
                __atomic_load_n(&control_task, __ATOMIC_ACQUIRE);

            if(task != nullptr)
            {
                xTaskNotifyGive(task);
            }
        }

        /**
         * @brief 处理左编码器事务完成
         */
        void handle_left_encoder()
        {
            i2c::left.pending = false;
            publish_left(as5600::process(as5600::left, i2c::left.completion_time_us));
            as5600::start_left_read();
        }

        /**
         * @brief 处理右编码器事务完成
         */
        void handle_right_encoder()
        {
            i2c::right.pending = false;

            const uint64_t now_us = i2c::right.completion_time_us;
            publish_right(as5600::process(as5600::right, now_us));

            // 到达 IMU 采样时刻时，暂时切换到 MPU6050
            if(now_us >= next_imu_time_us)
            {
                do
                {
                    next_imu_time_us += MPU_PERIOD_US;
                }
                while(now_us >= next_imu_time_us);

                right_reading_imu = true;
                i2c::set_right_address(mpu6050::ADDRESS);
                mpu6050::start_read();

                return;
            }

            as5600::start_right_read();
        }

        /**
         * @brief 处理 MPU6050 事务完成
         */
        void handle_imu()
        {
            i2c::right.pending = false;

            imu_data data{};
            if(mpu6050::process(i2c::right.completion_time_us, data))
            {
                publish_imu(data);
            }

            // MPU 完成后立即切回 AS5600
            i2c::set_right_address(as5600::ADDRESS);
            right_reading_imu = false;
            as5600::start_right_read();
        }

        /**
         * @brief 检查异步 I2C 事务超时
         *
         * @param[in] allow_recovery 是否允许执行总线恢复
         */
        void recover_stalled_transactions(bool allow_recovery)
        {
            if(!allow_recovery)
            {
                return;
            }

            const uint64_t now_us = sys_time::get_us_tick();

            if(i2c::left.pending &&
               now_us - i2c::left.submitted_us > I2C_TIMEOUT_US)
            {
                i2c::rebuild(i2c::left);
                as5600::start_left_read();
            }

            if(i2c::right.pending &&
               now_us - i2c::right.submitted_us > I2C_TIMEOUT_US)
            {
                i2c::rebuild(i2c::right);

                right_reading_imu = false;
                next_imu_time_us = now_us + MPU_PERIOD_US;

                as5600::start_right_read();
            }
        }
    }

    /**
     * @brief 初始化传感器模块
     *
     * @return true 初始化成功
     * @return false 当前任务不符合所有权要求
     *
     * @note 不创建任务。初始化与后续 poll() 必须由同一任务执行
     */
    bool init()
    {
        const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

        if(initialized)
        {
            return owner_task == current_task;
        }

        owner_task = current_task;

        i2c::init(owner_task);
        mpu6050::init();

        next_imu_time_us =
            sys_time::get_us_tick() + MPU_PERIOD_US;

        as5600::start_left_read();
        as5600::start_right_read();

        initialized = true;

        return true;
    }

    /**
     * @brief 处理传感器事件并推进异步采样
     *
     * @param[in] notifications 调用方收到的任务通知位
     * @param[in] allow_recovery 是否允许恢复 I2C 总线
     *
     * @note 不等待任务通知，不创建任务
     *       总线恢复只在 allow_recovery 为 true 时执行
     */
    void poll(uint32_t notifications, bool allow_recovery)
    {
        if(!initialized ||
           owner_task != xTaskGetCurrentTaskHandle())
        {
            return;
        }

        if(notifications & i2c::LEFT_DONE)
        {
            handle_left_encoder();
        }

        if(notifications & i2c::RIGHT_DONE)
        {
            if(right_reading_imu)
            {
                handle_imu();
            }
            else
            {
                handle_right_encoder();
            }
        }

        recover_stalled_transactions(allow_recovery);
    }

    /**
     * @brief 阻塞等待新的 IMU 数据
     *
     * @return true 收到 IMU 更新通知
     * @return false 等待超时
     *
     * @note 仅供一个控制任务等待
     */
    bool wait_for_imu()
    {
        __atomic_store_n(
            &control_task,
            xTaskGetCurrentTaskHandle(),
            __ATOMIC_RELEASE);

        return ulTaskNotifyTake(
            pdTRUE,
            pdMS_TO_TICKS(10)) != 0;
    }

    /**
     * @brief 获取最新传感器数据
     *
     * @param[out] snapshot 传感器数据快照
     *
     * @return true IMU 已发布有效数据
     * @return false IMU 尚未完成校准
     */
    bool get_package(package &snapshot)
    {
        portENTER_CRITICAL(&package_lock);
        snapshot = latest_package;
        portEXIT_CRITICAL(&package_lock);

        return snapshot.imu.timestamp_us != 0;
    }

    /**
     * @brief 更新姿态与陀螺仪零偏估计
     *
     * @param[in, out] snapshot 传感器快照，输出姿态角和修正角速度
     * @param[in] wheel_angle_rad 左右轮正方向累计角度，单位 rad
     * @param[in] grounded 轮地运动信息是否有效
     * @param[in] allow_yaw_learning 是否允许学习 Yaw 零偏
     *
     * @return true 姿态更新成功
     * @return false IMU 数据无效、重复或姿态初始化失败
     *
     * @note 转发到 MPU6050，须由姿态控制任务调用
     */
    bool update_attitude(
        package &snapshot,
        const float wheel_angle_rad[2],
        bool grounded,
        bool allow_yaw_learning)
    {
        return mpu6050::update_attitude(
            snapshot,
            wheel_angle_rad,
            grounded,
            allow_yaw_learning);
    }

    /**
     * @brief 重新初始化姿态估计
     *
     * @note 转发到 MPU6050，须由姿态控制任务调用
     */
    void reset_attitude()
    {
        mpu6050::reset_attitude();
    }
}
