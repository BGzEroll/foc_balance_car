#include "test.h"
#include "controller/balance.h"
#include "hw/motor.h"
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
                    printf("REF accepted=0\n");
                return;
            }
            motor::foc_info m{};
            balance::status b{};
            motor::get_info(m);
            balance::get_status(b);
            if(!strcmp(line,"START"))
                printf("START accepted=%u\n",!b.active && balance::start());
            else if(!strcmp(line,"STOP"))
            {
                motor::emergency_stop();
                printf("STOP accepted=1\n");
            }
            else if(!strcmp(line,"STATUS"))
                printf("STATUS ready=%u active=%u fault=%lu bus=%.3f pitch=%.6f dir_a=%.0f dir_b=%.0f\n",
                    m.ready,b.active,(unsigned long)m.fault.code,m.bus_voltage_V,
                    b.measured.pitch_rad,m.current[0].encoder_direction,m.current[1].encoder_direction);
            fflush(stdout);
        }

        void task(void *)
        {
            uint32_t sequence=0;
            uint64_t previous_imu_us=0;
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
                    const balance::gains g{};
                    float data[34]={b.measured.pitch_rad,b.roll_rad,b.measured.pitch_rate_rad_s,b.measured.yaw_rad,b.measured.yaw_rate_rad_s,
                        b.measured.speed_m_s,b.controller.distance_m,b.command.target_speed_m_s,b.command.target_pitch_rad,
                        active?b.command.current_A[0]:0,active?b.command.current_A[1]:0,
                        m.current[0].iq_A,m.current[1].iq_A,m.current[0].id_A,m.current[1].id_A,
                        s.left_encoder.speed_rad_s,s.right_encoder.speed_rad_s,s.left_encoder.angle_rad,s.right_encoder.angle_rad,
                        m.bus_voltage_V,6,b.controller.velocity_integral,b.controller.pitch_integral,b.controller.yaw_integral,
                        g.velocity_kp,g.velocity_ki,g.pitch_kd,g.pitch_kp,g.pitch_ki,g.yaw_kp,g.yaw_ki,g.yaw_kd,g.position_kp,g.pitch_trim_rad};
                    memcpy(f.data,data,sizeof(data));
                    f.crc=esp_rom_crc32_le(0,reinterpret_cast<const uint8_t*>(&f),sizeof(f)-4);
                    uart_write_bytes(UART_NUM_0,&f,sizeof(f));
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
