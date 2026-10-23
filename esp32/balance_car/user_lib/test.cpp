#include "test.h"

#include "balance.h"
#include "wifi_debug.h"
#include "hw/motor.h"
#include "hw/sensor.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdarg.h>
#include "esp_timer.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace test
{
    namespace
    {
        constexpr float FORWARD_SIGN[2] = {-1.0f, 1.0f};
        constexpr float MAX_CURRENT_A = 0.7f;
        constexpr float MAX_WHEEL_SPEED_RAD_S = 12.0f;
        constexpr uint32_t ROW_COUNT = 400;

        enum class mode : uint32_t
        {
            IDLE,
            SHADOW,
            BALANCE,
            PROBE
        };

        struct run
        {
            mode type = mode::IDLE;
            uint32_t duration_ms = 0;
            uint32_t motor = 0;
            float current_A = 0.0f;
            balance::gains tuning{};
        };

        struct row
        {
            uint32_t time_us;
            uint32_t dt_us;
            float pitch, roll, pitch_rate, yaw, yaw_rate;
            float speed, distance, target_speed, target_pitch;
            float command[2], iq[2], id[2], raw_speed[2], forward_speed[2];
            float angle[2];
            uint32_t current_age_us[2];
            uint32_t saturated, imu_age_us, encoder_age_us, calculate_us;
        };

        run pending{};
        balance::gains tuning{0.5f, 0.2f, 0.05f};
        portMUX_TYPE request_lock = portMUX_INITIALIZER_UNLOCKED;
        uint32_t requested = 0;
        uint32_t active = 0;
        uint32_t cancel = 0;
        uint32_t ended = 0;
        uint32_t count = 0;
        uint32_t result = 0;
        uint32_t write_index = 0;
        portMUX_TYPE log_lock = portMUX_INITIALIZER_UNLOCKED;
        uint32_t scheduler_updates = 0, scheduler_max_dt_us = 0;
        uint32_t updates = 0;
        uint32_t maximum_dt_us = 0;
        uint32_t maximum_calculate_us = 0;
        row rows[ROW_COUNT] = {};
        row snapshot[ROW_COUNT] = {};

        struct network_request { char line[96]; };
        struct network_reply { char text[768]; };
        QueueHandle_t command_queue = nullptr, reply_queue = nullptr;
        char *reply_text = nullptr;
        size_t reply_used = 0;
        // Only the console task executes commands, for both UART and TCP.
        void respond(const char *format, ...)
        {
            va_list args;
            va_start(args, format);
            if(reply_text)
            {
                const size_t remaining = sizeof(network_reply::text) - reply_used;
                int n = vsnprintf(reply_text + reply_used, remaining, format, args);
                if(n > 0) reply_used += static_cast<size_t>(n) < remaining ? n : remaining - 1;
            }
            else vprintf(format, args);
            va_end(args);
        }

        /**
         * @brief 输出当前硬件状态，角度和速度均采用车身前进坐标
         */
        void status()
        {
            sensor::package sensors{};
            sensor::get_package(sensors);
            motor::foc_info foc{};
            motor::get_info(foc);
            respond("STATUS calibrated=%u running=%u ready=%u fault=%lu bus=%.4f "
                   "imu_us=%llu pitch=%.6f roll=%.6f yaw=%.6f "
                   "raw_a=%.6f raw_b=%.6f dir_a=%.0f dir_b=%.0f active=%lu\n",
                   foc.calibrated, foc.running, foc.ready,
                   static_cast<unsigned long>(foc.fault.code), foc.bus_voltage_V,
                   static_cast<unsigned long long>(sensors.imu.timestamp_us),
                   -sensors.imu.angle[0], sensors.imu.angle[1], sensors.imu.angle[2],
                   sensors.left_encoder.speed_rad_s, sensors.right_encoder.speed_rad_s,
                   foc.current[0].encoder_direction, foc.current[1].encoder_direction,
                   static_cast<unsigned long>(__atomic_load_n(&active, __ATOMIC_ACQUIRE)));
            fflush(stdout);
        }

        /**
         * @brief 按 200 Hz 运行控制器，结束或保护触发后立即停止输出
         */
        void control_task(void *)
        {
            TickType_t wake = xTaskGetTickCount();
            run current{};
            balance::state controller{};
            uint64_t began_us = 0;
            uint64_t previous_us = 0;
            float yaw_zero = 0.0f;
            uint32_t log_stride = 1;
            uint64_t previous_wake_us = 0;

            while(true)
            {
                vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
                sensor::package sensors{};
                sensor::get_package(sensors);
                motor::foc_info foc{};
                motor::get_info(foc);
                const uint64_t now = esp_timer_get_time();
                if(previous_wake_us) {
                    const uint32_t dt = now - previous_wake_us;
                    if(dt > scheduler_max_dt_us) __atomic_store_n(&scheduler_max_dt_us, dt, __ATOMIC_RELAXED);
                }
                previous_wake_us = now;
                __atomic_add_fetch(&scheduler_updates, 1U, __ATOMIC_RELAXED);
                const bool fresh = sensors.imu.timestamp_us != 0 &&
                    now - sensors.imu.timestamp_us <= 20000 &&
                    sensors.left_encoder.timestamp_us != 0 &&
                    now - sensors.left_encoder.timestamp_us <= 1500 &&
                    sensors.right_encoder.timestamp_us != 0 &&
                    now - sensors.right_encoder.timestamp_us <= 1500;

                if(__atomic_exchange_n(&requested, 0U, __ATOMIC_ACQ_REL))
                {
                    portENTER_CRITICAL(&request_lock);
                    current = pending;
                    portEXIT_CRITICAL(&request_lock);
                    controller = balance::state{};
                    began_us = now;
                    previous_us = now;
                    const float stopped_speed[2] = {};
                    yaw_zero = balance::observe(sensors.imu.angle, sensors.imu.gyro, stopped_speed).yaw_rad;
                    count = 0;
                    write_index = 0;
                    result = 0;
                    updates = 0;
                    maximum_dt_us = 0;
                    maximum_calculate_us = 0;
                    log_stride = current.duration_ms == 0 ? 5U : (current.duration_ms + 1999U) / 2000U;
                    __atomic_store_n(&cancel, 0U, __ATOMIC_RELEASE);
                    __atomic_store_n(&active, static_cast<uint32_t>(current.type), __ATOMIC_RELEASE);

                    if(!fresh || !foc.calibrated || foc.fault.active ||
                       (current.type == mode::BALANCE &&
                        (fabsf(sensors.imu.angle[0]) > 0.13962634f ||
                         fabsf(sensors.imu.angle[1]) > 0.13962634f)))
                    {
                        result = 1;
                    }
                    else if(current.type != mode::SHADOW &&
                            !motor::start(current.duration_ms,
                                current.type == mode::PROBE ? 1U << current.motor : 3U))
                    {
                        result = 2;
                    }
                    continue;
                }

                if(current.type == mode::IDLE)
                {
                    continue;
                }

                const uint64_t elapsed = now - began_us;

                if(result != 0)
                {
                    motor::stop();
                    current.type = mode::IDLE;
                    __atomic_store_n(&active, 0U, __ATOMIC_RELEASE);
                    __atomic_store_n(&ended, 1U, __ATOMIC_RELEASE);
                    continue;
                }

                if(__atomic_exchange_n(&cancel, 0U, __ATOMIC_ACQ_REL))
                {
                    result = 3;
                }
                else if(!fresh || foc.fault.active)
                {
                    result = 4;
                }
                else if(fabsf(sensors.imu.angle[0]) > 0.26179939f ||
                        fabsf(sensors.imu.angle[1]) > 0.26179939f ||
                        fabsf(sensors.left_encoder.speed_rad_s) > MAX_WHEEL_SPEED_RAD_S ||
                        fabsf(sensors.right_encoder.speed_rad_s) > MAX_WHEEL_SPEED_RAD_S)
                {
                    result = 5;
                }
                else if(now - previous_us > 10000)
                {
                    result = 6;
                }
                else if(current.type != mode::SHADOW && !foc.ready &&
                        (current.duration_ms == 0 || elapsed < current.duration_ms * 1000ULL))
                {
                    result = 7;
                }

                if(result != 0 || (current.duration_ms != 0 && elapsed >= current.duration_ms * 1000ULL))
                {
                    motor::stop();
                    current.type = mode::IDLE;
                    __atomic_store_n(&active, 0U, __ATOMIC_RELEASE);
                    __atomic_store_n(&ended, 1U, __ATOMIC_RELEASE);
                    continue;
                }

                if(current.duration_ms == 0)
                {
                    motor::keep_alive();
                }

                portENTER_CRITICAL(&request_lock);
                current.tuning = tuning;
                portEXIT_CRITICAL(&request_lock);
                const uint64_t calculate_began_us = esp_timer_get_time();
                const float roll = sensors.imu.angle[1];
                const float forward_speed[2] =
                {
                    FORWARD_SIGN[0] * foc.current[0].encoder_direction *
                        sensors.left_encoder.speed_rad_s,
                    FORWARD_SIGN[1] * foc.current[1].encoder_direction *
                        sensors.right_encoder.speed_rad_s
                };
                balance::observation measured = balance::observe(
                    sensors.imu.angle, sensors.imu.gyro, forward_speed);
                measured.yaw_rad -= yaw_zero;
                balance::limits limit{};
                limit.current_A = fabsf(current.current_A);
                balance::output output = balance::update(
                    measured, balance::reference{}, limit, controller, current.tuning);

                if(current.type == mode::PROBE)
                {
                    output.current_A[0] = 0.0f;
                    output.current_A[1] = 0.0f;
                    output.current_A[current.motor] = current.current_A;
                    output.saturated = false;
                }

                bool accepted = true;

                for(uint32_t i = 0; i < 2; ++i)
                {
                    if(!isfinite(output.current_A[i]))
                    {
                        accepted = false;
                    }
                    else if(current.type != mode::SHADOW)
                    {
                        motor::target_current target{};
                        target.iq_A = FORWARD_SIGN[i] * output.current_A[i];
                        accepted = motor::set_target_current(static_cast<motor::motor_id>(i), target) && accepted;
                    }
                }

                if(!accepted)
                {
                    result = 8;
                    motor::stop();
                }

                const uint32_t dt_us = now - previous_us;
                const uint32_t calculate_us = esp_timer_get_time() - calculate_began_us;
                previous_us = now;
                ++updates;

                if(dt_us > maximum_dt_us)
                {
                    maximum_dt_us = dt_us;
                }

                if(calculate_us > maximum_calculate_us)
                {
                    maximum_calculate_us = calculate_us;
                }

                if((updates - 1U) % log_stride != 0)
                {
                    continue;
                }

                row entry{};
                entry.time_us = elapsed;
                entry.dt_us = dt_us;
                entry.pitch = measured.pitch_rad;
                entry.roll = roll;
                entry.pitch_rate = measured.pitch_rate_rad_s;
                entry.yaw = measured.yaw_rad;
                entry.yaw_rate = measured.yaw_rate_rad_s;
                entry.speed = measured.speed_m_s;
                entry.distance = controller.distance_m;
                entry.target_speed = output.target_speed_m_s;
                entry.target_pitch = output.target_pitch_rad;

                for(uint32_t i = 0; i < 2; ++i)
                {
                    entry.command[i] = output.current_A[i];
                    entry.iq[i] = foc.current[i].iq_A;
                    entry.id[i] = foc.current[i].id_A;
                    entry.raw_speed[i] = i == 0 ? sensors.left_encoder.speed_rad_s : sensors.right_encoder.speed_rad_s;
                    entry.forward_speed[i] = forward_speed[i];
                    entry.angle[i] = i == 0 ? sensors.left_encoder.angle_rad : sensors.right_encoder.angle_rad;
                    entry.current_age_us[i] = now - foc.current[i].timestamp_us;
                }

                entry.saturated = output.saturated;
                entry.imu_age_us = now - sensors.imu.timestamp_us;
                entry.encoder_age_us = now -
                    (sensors.left_encoder.timestamp_us < sensors.right_encoder.timestamp_us ?
                     sensors.left_encoder.timestamp_us : sensors.right_encoder.timestamp_us);
                entry.calculate_us = calculate_us;
                portENTER_CRITICAL(&log_lock);
                rows[write_index] = entry;
                write_index = (write_index + 1U) % ROW_COUNT;

                if(count < ROW_COUNT)
                {
                    ++count;
                }

                portEXIT_CRITICAL(&log_lock);
            }
        }

        /**
         * @brief 在停机后导出 RAM 中的完整控制轨迹
         */
        void dump()
        {
            portENTER_CRITICAL(&log_lock);
            memcpy(snapshot, rows, sizeof(rows));
            const uint32_t size = count;
            const uint32_t head = (write_index + ROW_COUNT - size) % ROW_COUNT;
            const uint32_t captured_updates = updates;
            const uint32_t captured_dt = maximum_dt_us;
            const uint32_t captured_calculate = maximum_calculate_us;
            portEXIT_CRITICAL(&log_lock);
            printf("CSV_BEGIN result=%lu rows=%lu updates=%lu max_dt_us=%lu max_calculate_us=%lu\n",
                   static_cast<unsigned long>(result), static_cast<unsigned long>(size),
                   static_cast<unsigned long>(captured_updates), static_cast<unsigned long>(captured_dt),
                   static_cast<unsigned long>(captured_calculate));
            printf("t_us,dt_us,pitch,roll,pitch_rate,yaw,yaw_rate,speed,distance,target_speed,target_pitch,cmd_a,cmd_b,iq_a,iq_b,id_a,id_b,raw_a,raw_b,forward_a,forward_b,saturated,imu_age_us,encoder_age_us,calculate_us,angle_a,angle_b,current_age_a_us,current_age_b_us\n");

            for(uint32_t i = 0; i < size; ++i)
            {
                const row &r = snapshot[(head + i) % ROW_COUNT];
                printf("%lu,%lu,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%lu,%lu,%lu,%lu,%.8g,%.8g,%lu,%lu\n",
                       static_cast<unsigned long>(r.time_us), static_cast<unsigned long>(r.dt_us),
                       r.pitch, r.roll, r.pitch_rate, r.yaw, r.yaw_rate, r.speed, r.distance,
                       r.target_speed, r.target_pitch, r.command[0], r.command[1], r.iq[0], r.iq[1],
                       r.id[0], r.id[1], r.raw_speed[0], r.raw_speed[1], r.forward_speed[0], r.forward_speed[1],
                       static_cast<unsigned long>(r.saturated), static_cast<unsigned long>(r.imu_age_us),
                       static_cast<unsigned long>(r.encoder_age_us), static_cast<unsigned long>(r.calculate_us),
                       r.angle[0], r.angle[1], static_cast<unsigned long>(r.current_age_us[0]),
                       static_cast<unsigned long>(r.current_age_us[1]));
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            printf("CSV_END\n");
        }

        void dispatch(const char *line)
        {
            if(strcmp(line, "SCHEDRESET") == 0)
            {
                __atomic_store_n(&scheduler_max_dt_us, 0U, __ATOMIC_RELAXED);
                respond("SCHEDRESET OK\n");
            }
            else if(strcmp(line, "STOP") == 0)
            {
                __atomic_store_n(&requested, 0U, __ATOMIC_RELEASE);
                motor::stop();
                __atomic_store_n(&cancel, 1U, __ATOMIC_RELEASE);
                respond("STOPPED\n");
            }
            else if(strcmp(line, "STATUS") == 0)
            {
                status();
            }
            else if(strcmp(line, "DUMP") == 0)
            {
                if(reply_text) respond("REJECT use STREAM or serial DUMP\n");
                else dump();
            }
            else if(strncmp(line, "TUNE ", 5) == 0)
            {
                balance::gains next{};

                if(sscanf(line, "TUNE %f %f %f", &next.velocity_kp,
                          &next.velocity_ki, &next.pitch_kd) != 3 ||
                   !isfinite(next.velocity_kp) || !isfinite(next.velocity_ki) ||
                   !isfinite(next.pitch_kd) || next.velocity_kp < 0.0f ||
                   next.velocity_kp > 2.0f || next.velocity_ki < 0.0f ||
                   next.velocity_ki > 2.0f || next.pitch_kd < 0.005f || next.pitch_kd > 0.1f)
                {
                    respond("REJECT tuning\n");
                }
                else
                {
                    portENTER_CRITICAL(&request_lock);
                    tuning = next;
                    portEXIT_CRITICAL(&request_lock);
                    respond("TUNING velocity_kp=%.4f velocity_ki=%.4f pitch_kd=%.4f\n",
                           tuning.velocity_kp, tuning.velocity_ki, tuning.pitch_kd);
                }
            }
            else if(__atomic_load_n(&active, __ATOMIC_ACQUIRE) ||
                    __atomic_load_n(&requested, __ATOMIC_ACQUIRE))
            {
                respond("BUSY\n");
            }
            else if(strcmp(line, "CAL") == 0)
            {
                sensor::package sensors{};
                sensor::get_package(sensors);
                const bool imu_ready = sensors.imu.timestamp_us != 0 &&
                    esp_timer_get_time() - sensors.imu.timestamp_us <= 20000;
                respond("CAL accepted=%u\n", imu_ready && motor::calibrate());
            }
            else
            {
                char command[16] = {};
                unsigned duration = 0;
                unsigned index = 0;
                float cap = 0.0f;
                run next{};

                if(sscanf(line, "PROBE %u %f %u", &index, &cap, &duration) == 3 && index < 2)
                {
                    next.type = mode::PROBE;
                    next.motor = index;
                }
                else if(sscanf(line, "%15s %u %f", command, &duration, &cap) == 3)
                {
                    if(strcmp(command, "SHADOW") == 0)
                    {
                        next.type = mode::SHADOW;
                    }
                    else if(strcmp(command, "BALANCE") == 0)
                    {
                        next.type = mode::BALANCE;
                    }
                }

                if(next.type == mode::IDLE || (duration == 0 && next.type != mode::BALANCE) ||
                   (duration != 0 && duration < 50) || duration > 10000 ||
                   !isfinite(cap) || fabsf(cap) > MAX_CURRENT_A || fabsf(cap) < 0.001f)
                {
                    respond("REJECT command\n");
                    return;
                }

                next.duration_ms = duration;
                next.current_A = cap;
                next.tuning = tuning;
                portENTER_CRITICAL(&request_lock);
                pending = next;
                portEXIT_CRITICAL(&request_lock);
                __atomic_store_n(&requested, 1U, __ATOMIC_RELEASE);
                respond("RUN accepted=1 mode=%lu duration_ms=%u cap=%.4f tuning=%.4f,%.4f,%.4f\n",
                       static_cast<unsigned long>(next.type), duration, cap,
                       tuning.velocity_kp, tuning.velocity_ki, tuning.pitch_kd);
            }
        }

        /**
         * @brief 接收串口命令，所有带输出试验均设置独立 PWM 截止时间
         */
        void console_task(void *)
        {
            uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
            char line[96] = {};
            uint32_t used = 0;
            respond("UPPER_BENCH_READY 200Hz default=STOP forward_sign=-1,+1\n");

            while(true)
            {
                if(__atomic_exchange_n(&ended, 0U, __ATOMIC_ACQ_REL))
                {
                    respond("DONE result=%lu rows=%lu updates=%lu max_dt_us=%lu max_calculate_us=%lu\n",
                           static_cast<unsigned long>(result), static_cast<unsigned long>(count),
                           static_cast<unsigned long>(updates), static_cast<unsigned long>(maximum_dt_us),
                           static_cast<unsigned long>(maximum_calculate_us));
                    status();
                }

                network_request request{};
                if(xQueueReceive(command_queue, &request, 0) == pdTRUE)
                {
                    network_reply reply{};
                    reply_text = reply.text;
                    reply_used = 0;
                    dispatch(request.line);
                    reply_text = nullptr;
                    xQueueSend(reply_queue, &reply, portMAX_DELAY);
                }

                uint8_t ch = 0;

                if(uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(10)) != 1)
                {
                    continue;
                }

                if(ch != '\n' && ch != '\r')
                {
                    if(used < sizeof(line) - 1)
                    {
                        line[used++] = ch;
                    }
                    continue;
                }

                line[used] = '\0';
                used = 0;

                dispatch(line);
                fflush(stdout);
            }
        }
    }

    bool network_command(const char *line, char *response, size_t capacity)
    {
        if(strlen(line) >= sizeof(network_request::line)) return false;
        network_request request{};
        snprintf(request.line, sizeof(request.line), "%s", line);
        if(xQueueSend(command_queue, &request, 0) != pdTRUE) return false;
        network_reply reply{};
        // Commands execute on console task; CAL starts async motor calibration.
        // Wait for this reply before submitting another request.
        if(xQueueReceive(reply_queue, &reply, portMAX_DELAY) != pdTRUE) return false;
        snprintf(response, capacity, "%s", reply.text);
        return true;
    }

    void network_stats(uint32_t &u, uint32_t &dt, uint32_t &calc, uint32_t &mode)
    {
        // Diagnostic counters only; no network operation runs in control task.
        u = __atomic_load_n(&scheduler_updates, __ATOMIC_RELAXED);
        dt = __atomic_load_n(&scheduler_max_dt_us, __ATOMIC_RELAXED);
        calc = __atomic_load_n(&maximum_calculate_us, __ATOMIC_RELAXED);
        mode = __atomic_load_n(&active, __ATOMIC_ACQUIRE);
    }

    /**
     * @brief 启动架空验证控制任务及串口命令入口，上电保持停机
     */
    bool init()
    {
        command_queue = xQueueCreate(1, sizeof(network_request));
        reply_queue = xQueueCreate(1, sizeof(network_reply));
        if(!command_queue || !reply_queue) return false;
        return xTaskCreatePinnedToCore(control_task, "upper_control", 6144,
                   nullptr, 2, nullptr, 0) == pdPASS &&
               xTaskCreatePinnedToCore(console_task, "upper_console", 6144,
                   nullptr, 1, nullptr, 0) == pdPASS && wifi_debug::init();
    }
}
