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
            uint32_t updates = 0, recoveries = 0;
            uint32_t submit_errors = 0, rejected_events = 0;
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
            const i2c_master_event_data_t *event,
            void *arg)
        {
            auto *ctx = static_cast<context *>(arg);

            if(!event || event->event != I2C_EVENT_DONE)
            {
                __atomic_add_fetch(&ctx->rejected_events,1,__ATOMIC_RELAXED);
                return false;
            }

            ctx->completion_time_us = sys_time::get_us_tick();

            BaseType_t task_woken = pdFALSE;
            xTaskNotifyFromISR(
                task_handle,
                ctx->notify_bit,
                eSetBits,
                &task_woken);

            // The ESP-IDF driver performs the yield after this callback returns.
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

    // All transaction queues belong to the old bus. Recreate them as well as
    // resetting hardware, so an abandoned asynchronous transaction cannot
    // leave a queued retry waiting forever. Called only from the owning task.
    void rebuild(context &ctx)
    {
        ESP_ERROR_CHECK(i2c_master_bus_reset(ctx.bus));
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(ctx.dev));
        ESP_ERROR_CHECK(i2c_del_master_bus(ctx.bus));
        ctx.bus=nullptr;ctx.dev=nullptr;ctx.pending=false;
        ctx.completion_time_us=0;
        ulTaskNotifyValueClear(nullptr,ctx.notify_bit);
        if(&ctx==&left)
            init_bus(ctx,LEFT_PORT,LEFT_SDA,LEFT_SCL,LEFT_FREQ_HZ,LEFT_DONE);
        else
            init_bus(ctx,RIGHT_PORT,RIGHT_SDA,RIGHT_SCL,RIGHT_FREQ_HZ,RIGHT_DONE);
        __atomic_add_fetch(&ctx.recoveries,1,__ATOMIC_RELAXED);
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
        ctx.submitted_us=sys_time::get_us_tick();
        ctx.pending=true;
        const esp_err_t result=i2c_master_transmit_receive(
                ctx.dev,
                reg,
                1,
                data,
                size,
                0);
        if(result!=ESP_OK)
            __atomic_add_fetch(&ctx.submit_errors,1,__ATOMIC_RELAXED);
        // A successful submission is not a successful measurement. Only the
        // DONE callback can publish data; the stopped-state watchdog retries.
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

        for(unsigned attempt=0;attempt<3;++attempt)
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
        constexpr float SPEED_FILTER_S = 0.003f;        // 速度低通时间常数：3 ms

        struct state
        {
            uint8_t reg = REG_RAW_ANGLE;
            uint8_t raw[2] = {};

            bool first_sample = true;

            uint16_t last_raw = 0;
            uint64_t last_time_us = 0;

            int32_t turns = 0;
            float speed_rad_s = 0.0f;
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

                const uint64_t dt_us =
                    now_us -
                    encoder.last_time_us;

                if(dt_us != 0)
                {
                    const float dt = static_cast<float>(dt_us) * 1.0e-6f;
                    const float delta_rad = static_cast<float>(delta) * COUNT_TO_RAD;

                    encoder.speed_rad_s =
                        (encoder.speed_rad_s * SPEED_FILTER_S + delta_rad) /
                        (SPEED_FILTER_S + dt);
                }

                encoder.last_raw = raw;
                encoder.last_time_us = now_us;
            }

            sensor::encoder_data data;
            data.timestamp_us = now_us;
            data.angle_rad = static_cast<float>(encoder.turns) * TWO_PI +
                             static_cast<float>(raw) * COUNT_TO_RAD;
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

        constexpr float PI = 3.14159265358979323846f;
        constexpr float DEG2RAD = PI / 180.0f;
        constexpr float ACC_COEF = 0.02f;

        uint8_t reg = REG_DATA;
        uint8_t raw[14] = {};

        int64_t gyro_offset_sum[3] = {};
        float gyro_offset[3] = {};

        uint32_t calibration_count = 0;
        uint64_t last_time_us = 0;

        float pitch_rad = 0.0f;     // 俯仰角，绕 X 轴，车头抬起为正，单位 rad
        float roll_rad = 0.0f;      // 横滚角，绕 Y 轴，车身右侧下沉为正，单位 rad
        float yaw_rad = 0.0f;       // 相对偏航角，绕 Z 轴，左转为正，单位 rad

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

            if(calibration_count < CALIBRATION_SAMPLES){return false;}

            for(uint8_t i = 0; i < 3; i++)
            {
                const float raw_offset = static_cast<float>(gyro_offset_sum[i]) /
                    static_cast<float>(CALIBRATION_SAMPLES);

                gyro_offset[i] =
                    raw_offset /
                    65.5f *
                    DEG2RAD;
            }

            last_time_us = 0;

            return false;
        }
    }

    /**
     * @brief 初始化 MPU6050
     */
    void init()
    {
        i2c::set_right_address(ADDRESS);

        /*
         * PLL X gyro clock
         */
        i2c::write_right_sync(REG_PWR_MGMT_1, 0x01);

        sys_time::delay_ms(10);

        /*
         * DLPF_CFG = 3
         *
         * Gyro/Accel bandwidth ≈ 44 Hz
         * Gyro internal output rate = 1 kHz
         */
        i2c::write_right_sync(REG_CONFIG, 0x03);

        /*
         * 1000 / (1 + 4)
         * = 200 Hz
         */
        i2c::write_right_sync(REG_SMPLRT_DIV, 0x04);

        /*
         * ±500 deg/s
         * 65.5 LSB/(deg/s)
         */
        i2c::write_right_sync(REG_GYRO_CONFIG, 0x08);

        /*
         * ±2 g
         * 16384 LSB/g
         */
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

        // 利用重力方向计算绕 X 轴的俯仰角
        const float acc_pitch_rad = atan2f(data.acc[1], data.acc[2]);

        // 利用重力方向计算绕 Y 轴的横滚角
        const float acc_roll_rad =
            atan2f(
                -data.acc[0],
                sqrtf(
                    data.acc[1] * data.acc[1] +
                    data.acc[2] * data.acc[2]));

        if(last_time_us == 0)
        {
            pitch_rad = acc_pitch_rad;
            roll_rad = acc_roll_rad;
            yaw_rad = 0.0f;
        }
        else
        {
            const float dt = static_cast<float>(now_us - last_time_us) * 1.0e-6f;

            // 俯仰使用 X 轴角速度进行互补滤波
            pitch_rad =
                (1.0f - ACC_COEF) *
                (
                    pitch_rad +
                    data.gyro[0] * dt
                ) +
                ACC_COEF *
                acc_pitch_rad;

            // 横滚使用 Y 轴角速度进行互补滤波
            roll_rad =
                (1.0f - ACC_COEF) *
                (
                    roll_rad +
                    data.gyro[1] * dt
                ) +
                ACC_COEF *
                acc_roll_rad;

            // 偏航仅积分 Z 轴角速度，保持原有正负 pi 范围
            yaw_rad += data.gyro[2] * dt;

            if(yaw_rad > PI)
            {
                yaw_rad -= 2.0f * PI;
            }
            else if(yaw_rad < -PI)
            {
                yaw_rad += 2.0f * PI;
            }
        }

        last_time_us = now_us;

        // 对外数组仍按 X、Y、Z 排列，单位为 rad
        data.angle[0] = pitch_rad;
        data.angle[1] = roll_rad;
        data.angle[2] = yaw_rad;

        return true;
    }
}

namespace sensor
{
    namespace
    {
        constexpr uint64_t MPU_PERIOD_US = 5000;

        portMUX_TYPE package_lock = portMUX_INITIALIZER_UNLOCKED;

        package latest_package;
        bool right_reading_imu = false;
        uint64_t next_imu_time_us = 0;
        bool started = false;
        bool (*recovery_permission)() = nullptr;
        unsigned event_ready = 0;
        unsigned test_nack_pending = 0;

        /**
         * @brief 发布左编码器数据
         *
         * @param[in] data 左编码器数据
         */
        void publish_left(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.left_encoder = data;
            latest_package.left_encoder.angle_rad = -latest_package.left_encoder.angle_rad;     // 根据实际安装修正方向
            latest_package.left_encoder.speed_rad_s = -latest_package.left_encoder.speed_rad_s;     // 根据实际安装修正方向
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
         * @brief 发布 IMU 数据
         *
         * @param[in] data IMU 数据
         */
        void publish_imu(const imu_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.imu = data;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 处理左编码器事务完成
         */
        void handle_left_encoder(bool allow_test = false)
        {
            i2c::left.pending=false;
            __atomic_add_fetch(&i2c::left.updates,1,__ATOMIC_RELAXED);
            publish_left(as5600::process(as5600::left, i2c::left.completion_time_us));
            if(allow_test&&__atomic_exchange_n(&test_nack_pending,0,__ATOMIC_ACQ_REL))
                ESP_ERROR_CHECK(i2c_master_device_change_address(i2c::left.dev,0x37,0));
            as5600::start_left_read();
        }

        /**
         * @brief 处理右编码器事务完成
         */
        void handle_right_encoder()
        {
            i2c::right.pending=false;
            __atomic_add_fetch(&i2c::right.updates,1,__ATOMIC_RELAXED);
            const uint64_t now_us = i2c::right.completion_time_us;
            publish_right(as5600::process(as5600::right, now_us));

            // MPU6050 到时间后插队一次
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
            i2c::right.pending=false;
            imu_data data;
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
         * @brief Sensor 任务入口
         */
        void task(void *)
        {
            begin_inline();
            __atomic_store_n(&event_ready,1,__ATOMIC_RELEASE);
            while(true){poll_inline(true,1);}
        }
    }

    // Inline mode is mutually exclusive with the standalone sensor task.
    void begin_inline(){
        i2c::init(xTaskGetCurrentTaskHandle());
        mpu6050::init();
        next_imu_time_us=sys_time::get_us_tick()+MPU_PERIOD_US;
        as5600::start_left_read();as5600::start_right_read();
        started=true;
    }
    void poll_inline(bool allow_recovery, uint32_t wait_ticks){
        uint32_t notification=0;
        xTaskNotifyWait(0,UINT32_MAX,&notification,wait_ticks);
        if(recovery_permission){allow_recovery=allow_recovery&&recovery_permission();}
        if(notification&i2c::LEFT_DONE)handle_left_encoder(allow_recovery);
        if(notification&i2c::RIGHT_DONE){if(right_reading_imu)handle_imu();else handle_right_encoder();}
        if(!allow_recovery)return;
        const uint64_t now=sys_time::get_us_tick();
        if(i2c::left.pending&&now-i2c::left.submitted_us>2000){
            i2c::rebuild(i2c::left);as5600::start_left_read();
        }
        if(i2c::right.pending&&now-i2c::right.submitted_us>2000){
            i2c::rebuild(i2c::right);right_reading_imu=false;
            next_imu_time_us=now+MPU_PERIOD_US;as5600::start_right_read();
        }
    }
    bool start_event_task(bool (*can_recover)()){
        if(started){return false;}
        recovery_permission=can_recover;
        if(!init()){return false;}
        for(unsigned i=0;i<2000;++i){
            if(__atomic_load_n(&event_ready,__ATOMIC_ACQUIRE)){return true;}
            vTaskDelay(1);
        }
        return false;
    }
    health_data get_health(){
        health_data h{};
        h.left_updates=__atomic_load_n(&i2c::left.updates,__ATOMIC_RELAXED);
        h.right_updates=__atomic_load_n(&i2c::right.updates,__ATOMIC_RELAXED);
        h.left_recoveries=__atomic_load_n(&i2c::left.recoveries,__ATOMIC_RELAXED);
        h.right_recoveries=__atomic_load_n(&i2c::right.recoveries,__ATOMIC_RELAXED);
        h.left_submit_errors=__atomic_load_n(&i2c::left.submit_errors,__ATOMIC_RELAXED);
        h.right_submit_errors=__atomic_load_n(&i2c::right.submit_errors,__ATOMIC_RELAXED);
        h.left_rejected_events=__atomic_load_n(&i2c::left.rejected_events,__ATOMIC_RELAXED);
        h.right_rejected_events=__atomic_load_n(&i2c::right.rejected_events,__ATOMIC_RELAXED);
        return h;
    }
    void request_test_nack(){__atomic_store_n(&test_nack_pending,1,__ATOMIC_RELEASE);}
    /**
     * @brief 初始化传感器模块
     *
     * @return true 初始化成功
     * @return false Sensor 任务创建失败
     */
    bool init()
    {
        if(started){return true;}

        if(xTaskCreatePinnedToCore(
            task,
            "sensor",
            4096,
            nullptr,
            5,
            nullptr,
            1) != pdPASS)
        {
            return false;
        }

        started = true;
        return true;
    }

    /**
     * @brief 获取最新传感器数据
     *
     * @param[out] snapshot 传感器数据快照
     *
     * @return true 数据有效
     * @return false IMU 尚未完成校准
     */
    bool get_package(package &snapshot)
    {
        portENTER_CRITICAL(&package_lock);
        snapshot = latest_package;
        portEXIT_CRITICAL(&package_lock);
        return snapshot.imu.timestamp_us != 0;
    }
}
