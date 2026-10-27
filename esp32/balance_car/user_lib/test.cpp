#include "test.h"
#include "controller/balance.h"
#include "hw/motor.h"
#include "hw/gamepad.h"
#include "hw/sensor.h"
#include "driver/uart.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace test
{
    namespace
    {
        struct __attribute__((packed)) frame
        {
            uint32_t magic, sequence;
            uint64_t sent_us, sample_us, imu_us;
            uint32_t values[13];
            float data[34];
            uint32_t crc;
        };
        static_assert(sizeof(frame) == 224);

        struct __attribute__((packed)) shadow_frame
        {
            uint32_t magic, sequence;
            uint64_t imu_us, shadow_us;
            int16_t raw[6];
            float data[19];
            uint32_t raw_flags, shadow_flags, condition, calculate_us;
            uint32_t crc;
        };
        static_assert(sizeof(shadow_frame) == 132);

        uint32_t age(uint64_t now, uint64_t stamp)
        {
            return stamp && now >= stamp ? static_cast<uint32_t>(now-stamp) : 0;
        }

        void respond(const char *line)
        {
            float position_m, speed_m_s;
            if(sscanf(line,"REF %f %f",&position_m,&speed_m_s)==2)
            {
                if(!balance::set_motion_reference(position_m,speed_m_s))
                {
                    constexpr char reply[]="REF accepted=0\n";
                    uart_write_bytes(UART_NUM_0,reply,sizeof(reply)-1);
                }
                return;
            }
            motor::foc_info m{};
            balance::status b{};
            motor::get_info(m);
            balance::get_status(b);
            char reply[256];
            int reply_length=0;
            if(!strcmp(line,"START"))
                reply_length=snprintf(reply,sizeof(reply),"START accepted=%u\n",!b.active && balance::start());
            else if(!strcmp(line,"STOP"))
            {
                motor::emergency_stop();
                reply_length=snprintf(reply,sizeof(reply),"STOP accepted=1\n");
            }
            else if(!strcmp(line,"FAULT"))
                reply_length=snprintf(reply,sizeof(reply),
                    "FAULTDIAG code=%lu motor=%u source=%lu detail_us=%lu adc_age_us=%lu heartbeat_age_us=%lu stamp_us=%llu raw_a=%u raw_b=%u\n",
                    (unsigned long)m.fault.code,m.fault.motor_index,
                    (unsigned long)m.fault.source,(unsigned long)m.fault.detail_us,
                    (unsigned long)m.fault.adc_age_us,(unsigned long)m.fault.heartbeat_age_us,
                    (unsigned long long)m.fault.timestamp_us,m.fault.raw_a,m.fault.raw_b);
            else if(!strcmp(line,"STATUS"))
            {
                gamepad::state pad{};
                gamepad::get_state(pad);
                reply_length=snprintf(reply,sizeof(reply),"STATUS ready=%u active=%u fault=%lu bus=%.3f pitch=%.6f dir_a=%.0f dir_b=%.0f pad=%u neutral=%u forward=%.3f turn=%.3f age_ms=%llu\n",
                    m.ready,b.active,(unsigned long)m.fault.code,m.bus_voltage_V,
                    b.measured.pitch_rad,m.current[0].encoder_direction,m.current[1].encoder_direction,
                    pad.connected,pad.neutral_seen,pad.forward,pad.turn,
                    (unsigned long long)(pad.received_us ? (esp_timer_get_time()-pad.received_us)/1000 : 0));
            }
            if(reply_length>0)uart_write_bytes(UART_NUM_0,reply,reply_length);
        }

        void task(void *)
        {
            uint32_t sequence=0;
            uint64_t previous_imu_us=0;
            uint32_t previous_fault=0;
            char line[32]{};
            uint32_t length=0;
            while(true)
            {
                char input[64];
                const int count=uart_read_bytes(UART_NUM_0,input,sizeof(input),0);
                for(int i=0;i<count;++i)
                {
                    if(input[i]=='\n') {line[length]=0;respond(line);length=0;}
                    else if(input[i]!='\r' && length<sizeof(line)-1) line[length++]=input[i];
                }
                sensor::package s{};
                motor::foc_info m{};
                balance::status b{};
                sensor::get_package(s);motor::get_info(m);balance::get_status(b);
                if(m.fault.active && uint32_t(m.fault.code)!=previous_fault)
                {
                    previous_fault=uint32_t(m.fault.code);
                    char info[192];
                    const int n=snprintf(info,sizeof(info),
                        "FAULTDIAG code=%lu motor=%u source=%lu detail_us=%lu adc_age_us=%lu heartbeat_age_us=%lu stamp_us=%llu\n",
                        (unsigned long)m.fault.code,m.fault.motor_index,
                        (unsigned long)m.fault.source,(unsigned long)m.fault.detail_us,
                        (unsigned long)m.fault.adc_age_us,(unsigned long)m.fault.heartbeat_age_us,
                        (unsigned long long)m.fault.timestamp_us);
                    uart_write_bytes(UART_NUM_0,info,n);
                }
                if(s.imu.timestamp_us && s.imu.timestamp_us!=previous_imu_us)
                {
                    previous_imu_us=s.imu.timestamp_us;
                    frame f{};f.magic=0x33554342U;f.sequence=sequence++;
                    f.sent_us=esp_timer_get_time();f.sample_us=b.sample_us;f.imu_us=b.imu_us;
                    const bool active=b.active && m.ready && !m.fault.active;
                    const bool calibrated=fabsf(m.current[0].encoder_direction)==1 && fabsf(m.current[1].encoder_direction)==1;
                    uint32_t values[13]={sequence,b.dt_us,0,active?2U:0U,0,
                        static_cast<uint32_t>(m.fault.code),(calibrated?1U:0U)|(active?6U:0U),b.command.saturated?1U:0U,
                        age(f.sent_us,b.imu_us),age(f.sent_us,s.left_encoder.timestamp_us<s.right_encoder.timestamp_us?s.left_encoder.timestamp_us:s.right_encoder.timestamp_us),
                        age(f.sent_us,m.current[0].timestamp_us),age(f.sent_us,m.current[1].timestamp_us),b.calculate_us};
                    memcpy(f.values,values,sizeof(values));
                    const balance::gains &g = b.applied_gains;
                    float data[34]={b.measured.pitch_rad,b.roll_rad,b.measured.pitch_rate_rad_s,b.measured.yaw_rad,b.measured.yaw_rate_rad_s,
                        b.measured.speed_m_s,b.controller.distance_m,b.command.target_speed_m_s,b.command.target_pitch_rad,
                        active?b.command.current_A[0]:0,active?b.command.current_A[1]:0,
                        m.current[0].iq_A,m.current[1].iq_A,m.current[0].id_A,m.current[1].id_A,
                        s.left_encoder.speed_rad_s,s.right_encoder.speed_rad_s,s.left_encoder.angle_rad,s.right_encoder.angle_rad,
                        m.bus_voltage_V,balance::limits{}.current_A,b.controller.velocity_integral,b.controller.pitch_integral,b.controller.yaw_integral,
                        g.velocity_kp,g.velocity_ki,g.pitch_kd,g.pitch_kp,g.pitch_ki,g.yaw_kp,g.yaw_ki,g.yaw_kd,g.position_kp,g.pitch_trim_rad};
                    memcpy(f.data,data,sizeof(data));
                    f.crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&f),sizeof(f)-4);
                    uart_write_bytes(UART_NUM_0,&f,sizeof(f));
                    shadow_frame c{};
                    c.magic=0x31545343U;c.sequence=f.sequence;
                    c.imu_us=s.imu.timestamp_us;c.shadow_us=b.surface.timestamp_us;
                    for(uint32_t i=0;i<3;++i)
                    {
                        c.raw[i]=s.imu.raw_acc[i];c.raw[i+3]=s.imu.raw_gyro[i];
                        if(c.raw[i]<=-32700 || c.raw[i]>=32700)c.raw_flags|=1U<<i;
                        if(c.raw[i+3]<=-32700 || c.raw[i+3]>=32700)c.raw_flags|=1U<<(i+3);
                    }
                    const auto &v=b.surface;
                    float shadow_values[19]={s.imu.acc[0],s.imu.acc[1],s.imu.acc[2],
                        s.imu.gyro[0],s.imu.gyro[1],s.imu.gyro[2],
                        v.gravity[0],v.gravity[1],v.gravity[2],v.gravity_norm_g,v.force_ratio,
                        v.body_accel_m_s2,v.wheel_accel_m_s2[0],v.wheel_accel_m_s2[1],
                        v.residual_m_s2[0],v.residual_m_s2[1],v.angular_rate_rad_s,
                        v.angular_accel_rad_s2,v.yaw_rate_residual_rad_s};
                    memcpy(c.data,shadow_values,sizeof(shadow_values));
                    c.shadow_flags=v.flags;c.condition=static_cast<uint32_t>(v.candidate);
                    c.calculate_us=v.calculate_us;
                    c.crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&c),sizeof(c)-4);
                    uart_write_bytes(UART_NUM_0,&c,sizeof(c));
                }
                vTaskDelay(1);
            }
        }
    }

    bool init()
    {
        uart_config_t config{};
        config.baud_rate=921600;config.data_bits=UART_DATA_8_BITS;config.parity=UART_PARITY_DISABLE;
        config.stop_bits=UART_STOP_BITS_1;config.flow_ctrl=UART_HW_FLOWCTRL_DISABLE;config.source_clk=UART_SCLK_DEFAULT;
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&config));
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,1024,4096,0,nullptr,0));
        return xTaskCreatePinnedToCore(task,"core_monitor",6144,nullptr,1,nullptr,0)==pdPASS;
    }
}
