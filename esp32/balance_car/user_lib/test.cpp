#include "test.h"
#include "realtime_config.h"

#include "balance.h"
#include "pid_config.h"
#include "motion_test.h"
#include "motion_integral.h"
#include "wifi_debug.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "hw/attitude_eskf.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_rom_crc.h"
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
        constexpr float MAX_CURRENT_A = 6.0f;
        constexpr float MAX_TILT_RAD = 0.7853981634f;
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
        balance::gains tuning{};
        uint32_t parameter_revision = pid_config::REVISION;
        telemetry latest_telemetry{};
        portMUX_TYPE telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
        portMUX_TYPE request_lock = portMUX_INITIALIZER_UNLOCKED;
        uint32_t requested = 0;
        uint32_t active = 0;
        uint32_t cancel = 0;
        uint32_t dynamic_request=0;
        float dynamic_amplitude=.05f;
        motion_test::state dynamic_status{};
        float dynamic_integral_limit=0;
        uint32_t ended = 0;
        uint32_t count = 0;
        uint32_t result = 0;
        uint32_t write_index = 0;
        portMUX_TYPE log_lock = portMUX_INITIALIZER_UNLOCKED;
        uint32_t scheduler_updates = 0, scheduler_max_dt_us = 0;
        uint32_t monitor_enabled = 1;
        uint32_t updates = 0;
        uint32_t maximum_dt_us = 0;
        uint32_t maximum_calculate_us = 0;
        row rows[ROW_COUNT] = {};
        row snapshot[ROW_COUNT] = {};

        struct network_request { char line[96]; };
        struct network_reply { char text[768]; };
        QueueHandle_t command_queue = nullptr, reply_queue = nullptr;
        SemaphoreHandle_t uart_tx_lock = nullptr;
        TaskHandle_t uart_stream_handle = nullptr;
        uint32_t uart_stream_enabled = 0;
        uint32_t imu_stream_enabled = 0;
        uint32_t attitude_stream_enabled=0,attitude_selected=1,attitude_confirmed_rest=0;
        struct __attribute__((packed)) attitude_frame {
            uint32_t magic,sequence;uint64_t timestamp_us;
            float q[4],bias[3],angle[3],legacy_angle[3],gyro[3],covariance_diag[6];
            uint32_t compute_us,gravity_updates,gravity_rejected,rest_updates,selected,confirmed_rest,stack_high_water;
            float nis;uint32_t crc;
        };
        static_assert(sizeof(attitude_frame)==140);
        attitude_frame latest_attitude{};
        portMUX_TYPE attitude_lock=portMUX_INITIALIZER_UNLOCKED;
        void get_attitude(attitude_frame &a){portENTER_CRITICAL(&attitude_lock);a=latest_attitude;portEXIT_CRITICAL(&attitude_lock);}
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
            else {
                if(uart_tx_lock) xSemaphoreTake(uart_tx_lock, portMAX_DELAY);
                vprintf(format, args);fflush(stdout);
                if(uart_tx_lock) xSemaphoreGive(uart_tx_lock);
            }
            va_end(args);
        }

        void gain_values(const balance::gains &g,float v[10])
        {
            const float values[10]={g.velocity_kp,g.velocity_ki,g.pitch_kd,g.pitch_kp,g.pitch_ki,g.yaw_kp,g.yaw_ki,g.yaw_kd,g.position_kp,g.pitch_trim_rad};
            memcpy(v,values,sizeof(values));
        }
        void report_gains()
        {
            balance::gains g;uint32_t revision;
            portENTER_CRITICAL(&request_lock);g=tuning;revision=parameter_revision;portEXIT_CRITICAL(&request_lock);
            float v[10];gain_values(g,v);
            respond("PID schema=pid_current_v1 revision=%lu design=%s Ts_us=5000 actual_dt=1 max_current_A=6 telemetry=BCU3 gains=%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                (unsigned long)revision,pid_config::DESIGN_SHA256,v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9]);
        }
        void publish(const sensor::package &s,const motor::foc_info &m,uint64_t now,uint32_t dt,
                     const balance::state &state,const balance::output &out,uint32_t calc,uint32_t current_mode,const balance::gains *applied=nullptr,uint32_t applied_revision=0,float current_cap=0,float yaw_reference=0)
        {
            telemetry t{}; t.sample_us=now;t.imu_us=s.imu.timestamp_us;
            t.cycle=scheduler_updates;t.dt_us=dt;t.mode=current_mode;t.result=result;
            t.fault=static_cast<uint32_t>(m.fault.code);
            t.flags=(m.calibrated?1U:0U)|(m.running?2U:0U)|(m.ready?4U:0U);
            t.imu_age_us=now-s.imu.timestamp_us;
            t.encoder_age_us=now-(s.left_encoder.timestamp_us<s.right_encoder.timestamp_us?s.left_encoder.timestamp_us:s.right_encoder.timestamp_us);
            t.current_age_a_us=now-m.current[0].timestamp_us;t.current_age_b_us=now-m.current[1].timestamp_us;
            t.raw_a=s.left_encoder.speed_rad_s;t.raw_b=s.right_encoder.speed_rad_s;
            const bool encoder_ab=sensor::encoder_observer_us()!=0;
            const float forward[]={FORWARD_SIGN[0]*m.current[0].encoder_direction*(encoder_ab?s.left_encoder.observer_speed_rad_s:t.raw_a),
                                   FORWARD_SIGN[1]*m.current[1].encoder_direction*(encoder_ab?s.right_encoder.observer_speed_rad_s:t.raw_b)};
            const auto observed=balance::observe(s.imu.angle,s.imu.gyro,forward);
            t.pitch=observed.pitch_rad;t.roll=s.imu.angle[1];t.pitch_rate=observed.pitch_rate_rad_s;
            t.yaw=observed.yaw_rad-yaw_reference;t.yaw_rate=observed.yaw_rate_rad_s;t.speed=observed.speed_m_s;
            t.distance=state.distance_m;t.target_speed=out.target_speed_m_s;t.target_pitch=out.target_pitch_rad;
            t.command_a=out.current_A[0];t.command_b=out.current_A[1];t.saturated=out.saturated;
            t.iq_a=m.current[0].iq_A;t.iq_b=m.current[1].iq_A;t.id_a=m.current[0].id_A;t.id_b=m.current[1].id_A;
            t.angle_a=s.left_encoder.angle_rad;t.angle_b=s.right_encoder.angle_rad;t.bus=m.bus_voltage_V;t.calculate_us=calc;t.current_limit_A=fabsf(current_cap);
            t.velocity_integral=state.velocity_integral;t.pitch_integral=state.pitch_integral;t.yaw_integral=state.yaw_integral;
            float values[10];
            if(applied) { gain_values(*applied,values); t.revision=applied_revision; }
            else {
                portENTER_CRITICAL(&request_lock);
                gain_values(tuning,values);t.revision=parameter_revision;
                portEXIT_CRITICAL(&request_lock);
            }
            memcpy(t.gains,values,sizeof(values));
            portENTER_CRITICAL(&telemetry_lock); latest_telemetry=t;portEXIT_CRITICAL(&telemetry_lock);
            if(uart_stream_handle && (__atomic_load_n(&uart_stream_enabled,__ATOMIC_ACQUIRE) ||
                __atomic_load_n(&imu_stream_enabled,__ATOMIC_ACQUIRE) || __atomic_load_n(&attitude_stream_enabled,__ATOMIC_ACQUIRE)))
                xTaskNotifyGive(uart_stream_handle);
        }

        void uart_stream_task(void *)
        {
            struct __attribute__((packed)) frame {
                uint32_t magic,sequence;uint64_t sent_us;telemetry data;uint32_t crc;
            };
            static_assert(sizeof(frame)==224);
            uint32_t sequence=0,last_cycle=UINT32_MAX;
            uint32_t imu_sequence=0;uint64_t last_imu_us=0;
            uint64_t last_attitude_us=0;
            while(true) {
                ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
                if(__atomic_load_n(&imu_stream_enabled,__ATOMIC_ACQUIRE)) {
                    sensor::package s{};sensor::get_package(s);
                    if(s.imu.timestamp_us && s.imu.timestamp_us!=last_imu_us) {
                        struct __attribute__((packed)) imu_frame {
                            uint32_t magic,sequence;uint64_t timestamp_us,read_began_us;
                            uint32_t sample_sequence;
                            int16_t raw[7];uint16_t reserved;
                            float gyro[3],angle[3];uint32_t crc;
                        } f{};
                        static_assert(sizeof(f)==72);
                        last_imu_us=s.imu.timestamp_us;f.magic=0x32554d49;f.sequence=imu_sequence++;
                        f.timestamp_us=s.imu.timestamp_us;f.read_began_us=s.imu.read_began_us;
                        f.sample_sequence=s.imu.sequence;memcpy(f.raw,s.imu.raw,sizeof(f.raw));
                        memcpy(f.gyro,s.imu.gyro,sizeof(f.gyro));memcpy(f.angle,s.imu.angle,sizeof(f.angle));
                        f.crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&f),sizeof(f)-4);
                        xSemaphoreTake(uart_tx_lock,portMAX_DELAY);
                        uart_write_bytes(UART_NUM_0,&f,sizeof(f));xSemaphoreGive(uart_tx_lock);
                    }
                }
                if(__atomic_load_n(&attitude_stream_enabled,__ATOMIC_ACQUIRE)){
                    attitude_frame a{};get_attitude(a);
                    if(a.timestamp_us && a.timestamp_us!=last_attitude_us){
                        last_attitude_us=a.timestamp_us;
                        a.crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&a),sizeof(a)-4);
                        xSemaphoreTake(uart_tx_lock,portMAX_DELAY);uart_write_bytes(UART_NUM_0,&a,sizeof(a));xSemaphoreGive(uart_tx_lock);
                    }
                }
                if(!__atomic_load_n(&uart_stream_enabled,__ATOMIC_ACQUIRE))continue;
                frame f{};get_telemetry(f.data);
                if(f.data.cycle==last_cycle)continue;
                last_cycle=f.data.cycle;f.magic=0x33554342U;f.sequence=sequence++;
                f.sent_us=esp_timer_get_time();
                f.crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&f),sizeof(f)-4);
                xSemaphoreTake(uart_tx_lock,portMAX_DELAY);
                uart_write_bytes(UART_NUM_0,&f,sizeof(f));
                xSemaphoreGive(uart_tx_lock);
            }
        }

        /**
         * @brief 输出当前硬件状态，角度和速度均采用车身前进坐标
         */
        void status()
        {
            sensor::package sensors{};
            sensor::get_package(sensors);
            attitude_frame a{};get_attitude(a);
            if(a.selected && a.timestamp_us){sensors.imu.timestamp_us=a.timestamp_us;memcpy(sensors.imu.angle,a.angle,sizeof(a.angle));}
            motor::foc_info foc{};
            motor::get_info(foc);
            respond("STATUS calibrated=%u running=%u ready=%u fault=%lu bus=%.4f "
                   "imu_us=%llu pitch=%.6f roll=%.6f yaw=%.6f "
                   "raw_a=%.6f raw_b=%.6f dir_a=%.0f dir_b=%.0f active=%lu pwm_core=%lu controller=PID\n",
                   foc.calibrated, foc.running, foc.ready,
                   static_cast<unsigned long>(foc.fault.code), foc.bus_voltage_V,
                   static_cast<unsigned long long>(sensors.imu.timestamp_us),
                   -sensors.imu.angle[0], sensors.imu.angle[1], sensors.imu.angle[2],
                   sensors.left_encoder.speed_rad_s, sensors.right_encoder.speed_rad_s,
                   foc.current[0].encoder_direction, foc.current[1].encoder_direction,
                   static_cast<unsigned long>(__atomic_load_n(&active, __ATOMIC_ACQUIRE)),
                   static_cast<unsigned long>(foc.pwm_core));
            fflush(stdout);
        }

        /**
         * @brief 按 200 Hz 运行控制器，结束或保护触发后立即停止输出
         */
        void control_task(void *)
        {
            run current{};
            balance::state controller{};
            motion_test::state dynamic{};
            motion_integral::state dynamic_integral{};
            uint64_t began_us = 0;
            uint64_t previous_us = 0;
            float yaw_zero = 0.0f;
            uint32_t log_stride = 1;
            uint64_t previous_wake_us = 0;
            uint32_t applied_revision=1;
            attitude_eskf::filter estimator;
            uint64_t previous_attitude_us=0;uint32_t attitude_sequence=0;
            bool previous_calibrated=false;

            while(true)
            {
                if(!sensor::wait_for_imu())continue;
                if(!__atomic_load_n(&monitor_enabled,__ATOMIC_ACQUIRE)) continue;
                motor::foc_info foc{};
                motor::get_info(foc);
                sensor::package sensors{};
                sensor::get_package(sensors);
                uint64_t now = esp_timer_get_time();
                // Wi-Fi may preempt this core between copying the package and
                // checking its age. Re-read once before rejecting an old copy;
                // keep the existing freshness limits and lower FOC watchdogs.
                if(sensors.left_encoder.timestamp_us && sensors.right_encoder.timestamp_us &&
                   (now-sensors.left_encoder.timestamp_us>1500 ||
                    now-sensors.right_encoder.timestamp_us>1500))
                {
                    sensor::get_package(sensors);
                    now = esp_timer_get_time();
                }
                const uint32_t wake_dt=previous_wake_us ? now-previous_wake_us : 5000U;
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

                if(foc.calibrated && !previous_calibrated){
                    // Calibration may follow a boot on the side. Rebuild gravity
                    // magnitude and tilt in the held calibration pose, retaining gyro bias.
                    float bias[3];memcpy(bias,estimator.bias,sizeof(bias));
                    estimator=attitude_eskf::filter{};memcpy(estimator.bias,bias,sizeof(bias));
                    previous_attitude_us=0;
                }
                previous_calibrated=foc.calibrated;
                float imu_dt_s=.005f;
                if(sensors.imu.timestamp_us && sensors.imu.timestamp_us!=previous_attitude_us){
                    const uint64_t filter_began=esp_timer_get_time();
                    imu_dt_s=previous_attitude_us?(sensors.imu.timestamp_us-previous_attitude_us)*1e-6f:.005f;
                    previous_attitude_us=sensors.imu.timestamp_us;
                    const bool rest=__atomic_load_n(&attitude_confirmed_rest,__ATOMIC_ACQUIRE)!=0;
                    estimator.update(sensors.imu.acc,sensors.imu.gyro,imu_dt_s,rest);
                    attitude_frame a{};a.magic=0x31545441;a.sequence=attitude_sequence++;a.timestamp_us=sensors.imu.timestamp_us;
                    memcpy(a.q,estimator.q,sizeof(a.q));memcpy(a.bias,estimator.bias,sizeof(a.bias));
                    float angles[3];estimator.euler(angles);memcpy(a.angle,angles,sizeof(angles));
                    memcpy(a.legacy_angle,sensors.imu.angle,sizeof(a.legacy_angle));
                    for(int i=0;i<3;++i)a.gyro[i]=sensors.imu.gyro[i]-estimator.bias[i];
                    for(int i=0;i<6;++i)a.covariance_diag[i]=estimator.P[i][i];
                    a.compute_us=esp_timer_get_time()-filter_began;a.gravity_updates=estimator.gravity_updates;
                    a.gravity_rejected=estimator.gravity_rejected;a.rest_updates=estimator.rest_updates;
                    a.selected=__atomic_load_n(&attitude_selected,__ATOMIC_ACQUIRE);a.confirmed_rest=rest;a.nis=estimator.nis;
                    a.stack_high_water=uxTaskGetStackHighWaterMark(nullptr);
                    portENTER_CRITICAL(&attitude_lock);latest_attitude=a;portEXIT_CRITICAL(&attitude_lock);
                    if(a.selected){memcpy(sensors.imu.angle,a.angle,sizeof(a.angle));memcpy(sensors.imu.gyro,a.gyro,sizeof(a.gyro));}
                }

                if(motor::update_locked())
                {
                    __atomic_store_n(&requested, 0U, __ATOMIC_RELEASE);
                    __atomic_store_n(&active, 0U, __ATOMIC_RELEASE);
                    current.type = mode::IDLE;
                    controller = balance::state{};
                    publish(sensors,foc,now,wake_dt,controller,balance::output{},0,0);
                    continue;
                }

                if(__atomic_exchange_n(&requested, 0U, __ATOMIC_ACQ_REL))
                {
                    portENTER_CRITICAL(&request_lock);
                    current = pending;
                    portEXIT_CRITICAL(&request_lock);
                    controller = balance::state{};
                    began_us = now;
                    dynamic=motion_test::state{};
                    __atomic_store_n(&dynamic_request,0U,__ATOMIC_RELEASE);
                    portENTER_CRITICAL(&request_lock);
                    dynamic_status=dynamic;
                    portEXIT_CRITICAL(&request_lock);
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

                    if(current.type!=mode::SHADOW)__atomic_store_n(&attitude_confirmed_rest,0U,__ATOMIC_RELEASE);
                    if(!fresh || !foc.calibrated || foc.fault.active)
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
                    publish(sensors,foc,now,wake_dt,controller,balance::output{},0,0);
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
                else if(fabsf(sensors.imu.angle[0]) > MAX_TILT_RAD ||
                        fabsf(sensors.imu.angle[1]) > MAX_TILT_RAD)
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
                applied_revision=parameter_revision;
                portEXIT_CRITICAL(&request_lock);
                const uint64_t calculate_began_us = esp_timer_get_time();
                const float roll = sensors.imu.angle[1];
                const bool encoder_ab=sensor::encoder_observer_us()!=0;
                const float forward_speed[2] =
                {
                    FORWARD_SIGN[0] * foc.current[0].encoder_direction *
                        (encoder_ab?sensors.left_encoder.observer_speed_rad_s:sensors.left_encoder.speed_rad_s),
                    FORWARD_SIGN[1] * foc.current[1].encoder_direction *
                        (encoder_ab?sensors.right_encoder.observer_speed_rad_s:sensors.right_encoder.speed_rad_s)
                };
                balance::observation measured = balance::observe(
                    sensors.imu.angle, sensors.imu.gyro, forward_speed);
                measured.yaw_rad -= yaw_zero;
                balance::limits limit{};
                limit.current_A = fabsf(current.current_A);
                const uint32_t dynamic_action=__atomic_exchange_n(&dynamic_request,0U,__ATOMIC_ACQ_REL);
                if(dynamic_action==1)
                {
                    float amplitude;
                    portENTER_CRITICAL(&request_lock);amplitude=dynamic_amplitude;portEXIT_CRITICAL(&request_lock);
                    dynamic.start(now,controller.distance_m,amplitude);
                    controller.velocity_integral=dynamic_integral.begin(controller.velocity_integral);
                }
                else if(dynamic_action==2)
                {controller.velocity_integral=dynamic_integral.end(controller.velocity_integral);dynamic=motion_test::state{};}
                else if(dynamic_action==3){dynamic.aborted=true;}
                dynamic.advance(now,controller.distance_m);
                float integral_limit=0;
                portENTER_CRITICAL(&request_lock);
                dynamic_status=dynamic;
                integral_limit=dynamic_integral_limit;
                portEXIT_CRITICAL(&request_lock);
                if(dynamic.hard_stop)
                {
                    result=9;motor::stop();continue;
                }
                balance::reference target{};
                if(dynamic.armed){
                    target.position_m=dynamic.position;target.speed_m_s=dynamic.speed;
                    target.pitch_bias_rad=-current.tuning.velocity_kp*current.tuning.position_kp*dynamic.origin;
                }
                if(dynamic.armed)controller.velocity_integral=dynamic_integral.update(controller.velocity_integral,
                    (now-dynamic.began_us)*1e-6,integral_limit,imu_dt_s);
                balance::output output = balance::update(
                    measured, target, limit, controller, imu_dt_s,current.tuning);

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
                    if(!std::isfinite(output.current_A[i]))
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

                publish(sensors,foc,now,dt_us,controller,output,calculate_us,static_cast<uint32_t>(current.type),&current.tuning,applied_revision,current.current_A,yaw_zero);

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
            if(motor::update_locked() && strcmp(line,"STOP") && strcmp(line,"STATUS") &&
               strcmp(line,"GAINS") && strcmp(line,"SCHEDRESET"))
            {
                respond("REJECT OTA maintenance lock\n");
                return;
            }
            if(!strcmp(line,"WIFIOFF") || !strcmp(line,"WIFION") ||
               !strcmp(line,"MONITOROFF") || !strcmp(line,"MONITORON"))
            {
                motor::foc_info m{};motor::get_info(m);
                if(reply_text || m.running || __atomic_load_n(&active,__ATOMIC_ACQUIRE) ||
                   __atomic_load_n(&requested,__ATOMIC_ACQUIRE)) respond("REJECT serial idle diagnostic only\n");
                else if(!strncmp(line,"WIFI",4)) respond("%s ok=%u\n",line,wifi_debug::set_enabled(!strcmp(line,"WIFION")));
                else {
                    __atomic_store_n(&monitor_enabled,!strcmp(line,"MONITORON")?1U:0U,__ATOMIC_RELEASE);
                    respond("%s OK\n",line);
                }
            }
            else if(strcmp(line, "SCHEDRESET") == 0)
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
            else if(!strcmp(line,"ENCCAP"))
            {
                respond("ENCCAP accepted=%u\n",sensor::begin_encoder_capture());
            }
            else if(!strcmp(line,"ENCDUMP"))
            {
                sensor::encoder_capture_header header{};
                const auto *records=sensor::get_encoder_capture(header);
                if(!records){respond("ENCDUMP ready=0 count=%lu\n",(unsigned long)header.count);return;}
                static_assert(sizeof(sensor::encoder_record)==16);
                uint32_t crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&header),sizeof(header));
                xSemaphoreTake(uart_tx_lock,portMAX_DELAY);
                uart_write_bytes(UART_NUM_0,&header,sizeof(header));
                for(uint32_t i=0;i<header.count;i+=256){
                    const uint32_t bytes=(header.count-i<256?header.count-i:256)*sizeof(*records);
                    crc=esp_rom_crc32_le(crc,reinterpret_cast<const uint8_t*>(records+i),bytes);
                    uart_write_bytes(UART_NUM_0,records+i,bytes);
                }
                uart_write_bytes(UART_NUM_0,&crc,sizeof(crc));
                xSemaphoreGive(uart_tx_lock);
                respond("ENCDUMP ready=1 count=%lu\n",(unsigned long)header.count);
            }
            else if(!strcmp(line,"ATTITUDE") || !strcmp(line,"ATTITUDE 0") || !strcmp(line,"ATTITUDE 1") ||
                    !strcmp(line,"ATTREST 0") || !strcmp(line,"ATTREST 1"))
            {
                motor::foc_info m{};motor::get_info(m);
                if(strcmp(line,"ATTITUDE") && m.running){respond("REJECT attitude selection/rest calibration requires stopped motors\n");return;}
                if(!strcmp(line,"ATTITUDE 0") || !strcmp(line,"ATTITUDE 1"))__atomic_store_n(&attitude_selected,line[9]=='1'?1U:0U,__ATOMIC_RELEASE);
                if(!strcmp(line,"ATTREST 0") || !strcmp(line,"ATTREST 1"))__atomic_store_n(&attitude_confirmed_rest,line[8]=='1'?1U:0U,__ATOMIC_RELEASE);
                attitude_frame a{};get_attitude(a);
                respond("ATTITUDE selected=%lu confirmed_rest=%lu compute_us=%lu bias=%.9g,%.9g,%.9g\n",
                    (unsigned long)__atomic_load_n(&attitude_selected,__ATOMIC_ACQUIRE),(unsigned long)__atomic_load_n(&attitude_confirmed_rest,__ATOMIC_ACQUIRE),
                    (unsigned long)a.compute_us,a.bias[0],a.bias[1],a.bias[2]);
            }
            else if(!strcmp(line,"ATTSTREAM 200") || !strcmp(line,"ATTSTREAM 0"))
            {
                const bool enabled=!strcmp(line,"ATTSTREAM 200");__atomic_store_n(&attitude_stream_enabled,enabled,__ATOMIC_RELEASE);
                respond("ATTSTREAM hz=%u bytes=140\n",enabled?200:0);
            }
            else if(!strcmp(line,"IMUINFO"))
            {
                const auto c=sensor::get_imu_config();
                respond("IMUINFO who_am_i=0x%02x sample_divider=%u dlpf=%u gyro_range=0x%02x accel_range=0x%02x power=0x%02x host_period_us=5000\n",
                    c.who_am_i,c.sample_divider,c.dlpf,c.gyro_range,c.accel_range,c.power);
            }
            else if(!strcmp(line,"IMUSTREAM 200") || !strcmp(line,"IMUSTREAM 0"))
            {
                const bool enabled=!strcmp(line,"IMUSTREAM 200");
                __atomic_store_n(&imu_stream_enabled,enabled,__ATOMIC_RELEASE);
                respond("IMUSTREAM hz=%u\n",enabled?200:0);
            }
            else if(!strcmp(line,"ENCAB"))
            {
                respond("ENCAB selected_us=%lu\n",(unsigned long)sensor::encoder_observer_us());
            }
            else if(!strncmp(line,"ENCAB ",6))
            {
                unsigned us=0;char extra;
                if(sscanf(line+6,"%u %c",&us,&extra)!=1 || (us!=0&&(us<1000||us>10000)))
                {respond("REJECT ENCAB requires 0 (LPF) or 1000..10000 us\n");return;}
                sensor::set_encoder_observer(us);respond("ENCAB selected_us=%u\n",us);
            }
            else if(!strcmp(line,"UARTSTREAM 200") || !strcmp(line,"UARTSTREAM 0"))
            {
                const bool enabled=!strcmp(line,"UARTSTREAM 200");
                if(reply_text)respond("REJECT UART command only\n");
                else {
                    __atomic_store_n(&uart_stream_enabled,enabled?1U:0U,__ATOMIC_RELEASE);
                    respond("UARTSTREAM hz=%u baud=921600 bytes=224 crc=CRC32\n",enabled?200:0);
                }
            }
            else if(strcmp(line, "FAULT") == 0)
            {
                motor::foc_info m{}; motor::get_info(m);
                respond("FAULT code=%lu motor=%u time_us=%llu source=%lu age_or_dt_us=%lu raw_a=%u raw_b=%u adc_age_us=%lu heartbeat_age_us=%lu control_phase=%lu sample_flags=%lu sample_start_tick=%lu sample_end_tick=%lu duty_a=%lu duty_b=%lu\n",
                    (unsigned long)m.fault.code,m.fault.motor_index,(unsigned long long)m.fault.timestamp_us,
                    (unsigned long)(m.fault.detail>>24),(unsigned long)(m.fault.detail&0xffffffU),m.fault.raw_a,m.fault.raw_b,
                    (unsigned long)m.fault.adc_age_us,(unsigned long)m.fault.heartbeat_age_us,(unsigned long)m.fault.control_phase,
                    (unsigned long)m.fault.sample_flags,(unsigned long)m.fault.sample_start_tick,
                    (unsigned long)m.fault.sample_end_tick,(unsigned long)m.fault.sample_duty_a,
                    (unsigned long)m.fault.sample_duty_b);
            }
            else if(strcmp(line, "DUMP") == 0)
            {
                if(reply_text) respond("REJECT use STREAM or serial DUMP\n");
                else if(__atomic_load_n(&uart_stream_enabled,__ATOMIC_ACQUIRE)) respond("REJECT use active UARTSTREAM; DUMP would interrupt streaming\n");
                else dump();
            }
            else if(strcmp(line,"PID")==0 || strcmp(line,"GAINS")==0) report_gains();
            else if(strcmp(line,"DYNSTAT")==0)
            {
                motion_test::state d;
                portENTER_CRITICAL(&request_lock);d=dynamic_status;portEXIT_CRITICAL(&request_lock);
                float integral_limit;
                portENTER_CRITICAL(&request_lock);integral_limit=dynamic_integral_limit;portEXIT_CRITICAL(&request_lock);
                respond("DYNSTAT armed=%u aborted=%u hard_stop=%u began_us=%llu phase=%u origin=%.6f position=%.6f speed=%.6f peak=%.6f integral_limit=%.6f amplitude=%.6f\n",
                    d.armed,d.aborted,d.hard_stop,(unsigned long long)d.began_us,d.phase,d.origin,d.position,d.speed,d.peak,integral_limit,d.amplitude);
            }
            else if(strncmp(line,"DYNINT ",7)==0)
            {
                float limit=0;char extra;
                if(sscanf(line+7,"%f %c",&limit,&extra)!=1 || !std::isfinite(limit) || limit<0 || limit>.12f)
                {respond("REJECT DYNINT requires 0..0.12 m*s\n");return;}
                portENTER_CRITICAL(&request_lock);dynamic_integral_limit=limit;portEXIT_CRITICAL(&request_lock);
                respond("DYNINT accepted=1 limit=%.6f\n",limit);
            }
            else if(strcmp(line,"DYNTEST")==0 || strncmp(line,"DYNTEST ",8)==0)
            {
                float amplitude=.05f;char extra;
                if(line[7] && (sscanf(line+8,"%f %c",&amplitude,&extra)!=1 || !std::isfinite(amplitude) || amplitude<.05f || amplitude>.20f))
                {respond("REJECT DYNTEST requires 0.05..0.20 m\n");return;}
                motion_test::state d;
                portENTER_CRITICAL(&request_lock);d=dynamic_status;portEXIT_CRITICAL(&request_lock);
                const bool accepted=__atomic_load_n(&active,__ATOMIC_ACQUIRE)==2 && !d.armed;
                if(accepted){portENTER_CRITICAL(&request_lock);dynamic_amplitude=amplitude;portEXIT_CRITICAL(&request_lock);__atomic_store_n(&dynamic_request,1U,__ATOMIC_RELEASE);}
                respond("DYNTEST accepted=%u excursion_m=%.6f duration_s=16 soft_return_m=.23 hard_stop_m=.25\n",accepted,amplitude);
            }
            else if(strcmp(line,"DYNABORT")==0)
            {
                motion_test::state d;portENTER_CRITICAL(&request_lock);d=dynamic_status;portEXIT_CRITICAL(&request_lock);
                if(d.armed)__atomic_store_n(&dynamic_request,3U,__ATOMIC_RELEASE);
                respond("DYNABORT accepted=%u\n",d.armed);
            }
            else if(strcmp(line,"DYNEND")==0)
            {
                motion_test::state d;
                portENTER_CRITICAL(&request_lock);d=dynamic_status;portEXIT_CRITICAL(&request_lock);
                const bool accepted=d.armed && d.phase>=4;
                if(accepted)__atomic_store_n(&dynamic_request,2U,__ATOMIC_RELEASE);
                respond("DYNEND accepted=%u\n",accepted);
            }
            else if(strncmp(line,"PIDSET ",7)==0)
            {
                float v[10];char extra;
                if(sscanf(line+7,"%f %f %f %f %f %f %f %f %f %f %c",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9],&extra)!=10)
                {respond("REJECT PIDSET requires ten gains\n");return;}
                bool valid=true;for(int i=0;i<10;++i)valid=valid&&std::isfinite(v[i])&&(i==9?fabsf(v[i])<=.1f:v[i]>=0);
                if(!valid){respond("REJECT invalid PID gains\n");return;}
                balance::gains g;g.velocity_kp=v[0];g.velocity_ki=v[1];g.pitch_kd=v[2];g.pitch_kp=v[3];g.pitch_ki=v[4];
                g.yaw_kp=v[5];g.yaw_ki=v[6];g.yaw_kd=v[7];g.position_kp=v[8];g.pitch_trim_rad=v[9];
                portENTER_CRITICAL(&request_lock);tuning=g;const uint32_t revision=++parameter_revision;portEXIT_CRITICAL(&request_lock);
                respond("PIDSET accepted=1 revision=%lu\n",(unsigned long)revision);
            }
            else if(strncmp(line,"LQI",3)==0 || strncmp(line,"TUNE",4)==0 || strncmp(line,"SET ",4)==0 || strncmp(line,"GAINSSET",8)==0)
                respond("REJECT PID controller; use PIDSET\n");
            else if(__atomic_load_n(&active, __ATOMIC_ACQUIRE) ||
                    __atomic_load_n(&requested, __ATOMIC_ACQUIRE))
            {
                respond("BUSY\n");
            }
            else if(strcmp(line, "CAL") == 0)
            {
                __atomic_store_n(&attitude_confirmed_rest,0U,__ATOMIC_RELEASE);
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
                   !std::isfinite(cap) || fabsf(cap) > MAX_CURRENT_A || fabsf(cap) < 0.001f)
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
                respond("RUN accepted=1 mode=%lu duration_ms=%u cap=%.4f controller=PID velocity_kp_ki_pitch_kd=%.4f,%.4f,%.4f\n",
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
            uart_set_baudrate(UART_NUM_0,921600);
            uart_tx_lock=xSemaphoreCreateMutex();
            if(!uart_tx_lock || xTaskCreatePinnedToCore(uart_stream_task,"uart_telemetry",4096,
                nullptr,realtime_config::NETWORK_PRIORITY,&uart_stream_handle,0)!=pdPASS)
            {
                motor::stop();vTaskDelete(nullptr);return;
            }
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

    void get_telemetry(telemetry &out)
    {
        portENTER_CRITICAL(&telemetry_lock);out=latest_telemetry;portEXIT_CRITICAL(&telemetry_lock);
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
                   nullptr, realtime_config::UPPER_PRIORITY, nullptr,
                   realtime_config::NETWORK_CORE) == pdPASS &&
               xTaskCreatePinnedToCore(console_task, "upper_console", 6144,
                   nullptr, 1, nullptr, 0) == pdPASS &&
               (!realtime_config::WIRELESS_DEBUG_ENABLED || wifi_debug::init());
    }
}
