#include "foc_current.h"

#include <math.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
// IDF 的 C 风格 LL 头文件含部分初始化；仅在包含它时关闭此项 C++ 警告。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "hal/adc_ll.h"
#pragma GCC diagnostic pop
#include "esp_cpu.h"
#include "esp_timer.h"
#include "xtensa/hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hw/sensor.h"
#include "esp_freertos_hooks.h"

namespace foc_current {
namespace {
constexpr uint32_t RESOLUTION_HZ=10000000, PERIOD_TICKS=500, PEAK_TICKS=250;
constexpr float TWO_PI=6.28318530718f, INV_SQRT3=0.57735026919f, SQRT3_HALF=0.86602540378f;
constexpr float CURRENT_GAIN=4.0f;
constexpr uint32_t BASELINE_COUNT=512;
constexpr adc_channel_t CHANNEL[5]={ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_3,ADC_CHANNEL_4,ADC_CHANNEL_0};
adc_oneshot_unit_handle_t adc=nullptr;
mcpwm_timer_handle_t timer=nullptr;
mcpwm_cmpr_handle_t comparator[6]={};
uint16_t voltage_mv[5][4096]={};
float offset_V[4]={}, integral_d[2]={}, integral_q[2]={};
state latest;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t idle_runs=0;
struct sampling {
 uint32_t events=0, gaps=0, invalid=0, sequence[2]={}, stamp[5]={};
 uint32_t count[5]={}; uint16_t raw[5]={};
 uint64_t sum[5]={}, square[5]={};
 uint16_t low[5]={4095,4095,4095,4095,4095},high[5]={};
 float baseline_raw[5]={};
 timing pair,bus,phase,skew;
 uint32_t late_half=0,previous=0,interval_min=UINT32_MAX,interval_max=0;
} samples;
bool idle_hook(){__atomic_add_fetch(&idle_runs,1,__ATOMIC_RELAXED);taskYIELD();return false;}
inline void record(timing &t,uint32_t n){t.sum+=n;if(n<t.minimum)t.minimum=n;if(n>t.maximum)t.maximum=n;}
        bool IRAM_ATTR read_adc(adc_channel_t channel, uint16_t &raw)
        {
            const uint32_t idle_start = esp_cpu_get_cycle_count();
            while(SENS.sar_slave_addr1.meas_status != 0)
            {
                if(esp_cpu_get_cycle_count() - idle_start >= CPU_MHZ * 20 &&
                   SENS.sar_slave_addr1.meas_status != 0){return false;}
            }
            adc_oneshot_ll_set_channel(ADC_UNIT_1, channel);
            adc_oneshot_ll_start(ADC_UNIT_1);
            const uint32_t start = esp_cpu_get_cycle_count();
            while(!adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE))
            {
                // 任务可能被外设中断抢占；超时前复查完成位，避免将已完成的转换误报失败。
                if(esp_cpu_get_cycle_count() - start >= CPU_MHZ * 20 &&
                   !adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE)){return false;}
            }
            raw = static_cast<uint16_t>(adc_oneshot_ll_get_raw_result(ADC_UNIT_1));
            return true;
        }

        void calculate(uint32_t motor, float ia, float ib, float bus, float angle,
                       float &id, float &iq, float *duty, float dt)
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
            const float candidate_d = integral_d[motor] + KI * dt * error_d;
            const float candidate_q = integral_q[motor] + KI * dt * error_q;
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

        void init_adc()
        {
            float baseline_raw[5] = {};
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
                    baseline_raw[i] += static_cast<float>(raw) / static_cast<float>(BASELINE_COUNT);
                    if(i < 4)
                    {
                        offset_V[i] += static_cast<float>(voltage_mv[i][raw]) *
                                       (0.001f / static_cast<float>(BASELINE_COUNT));
                    }
                }
            }
            portENTER_CRITICAL(&mux);
            for(uint32_t i = 0; i < 5; ++i){samples.baseline_raw[i] = baseline_raw[i];}
            portEXIT_CRITICAL(&mux);
        }
void IRAM_ATTR acquire(unsigned ch){
 uint16_t raw=0; bool ok=read_adc(CHANNEL[ch],raw);
 samples.stamp[ch]=esp_cpu_get_cycle_count();
 samples.raw[ch]=raw; ++samples.count[ch];
 samples.sum[ch]+=raw; samples.square[ch]+=uint32_t(raw)*raw;
 if(raw<samples.low[ch])samples.low[ch]=raw;
 if(raw>samples.high[ch])samples.high[ch]=raw;
 if(!ok||raw==0||raw>=4095)++samples.invalid;
}
bool IRAM_ATTR on_full(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t *ev,void*){
 uint32_t start=esp_cpu_get_cycle_count();
 // Callback count is a driver snapshot shortly before entry, not ADC aperture time.
 uint32_t ticks=ev->direction==MCPWM_TIMER_DIRECTION_DOWN ? PEAK_TICKS-ev->count_value : PEAK_TICKS+ev->count_value;
 portENTER_CRITICAL_ISR(&mux);
 unsigned motor=samples.events++ & 1U;
 if(samples.previous){uint32_t d=start-samples.previous; if(d<samples.interval_min)samples.interval_min=d;if(d>samples.interval_max)samples.interval_max=d;
  if(d>CPU_MHZ*75)samples.gaps+=(d+CPU_MHZ*25)/(CPU_MHZ*50)-1;}
 samples.previous=start;
 acquire(motor*2); acquire(motor*2+1); ++samples.sequence[motor];
 record(samples.skew,samples.stamp[motor*2+1]-samples.stamp[motor*2]);
 uint32_t elapsed=esp_cpu_get_cycle_count()-start;
 record(samples.pair,elapsed);record(samples.phase,ticks*CPU_MHZ/10);
 if(ticks*CPU_MHZ/10+elapsed>=CPU_MHZ*25)++samples.late_half;
 portEXIT_CRITICAL_ISR(&mux);
 return false;
}
bool IRAM_ATTR on_empty(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 static uint32_t n=0;if(++n%20)return false;
 uint32_t start=esp_cpu_get_cycle_count();
 portENTER_CRITICAL_ISR(&mux);acquire(4);record(samples.bus,esp_cpu_get_cycle_count()-start);portEXIT_CRITICAL_ISR(&mux);
 return false;
}
void control_task(void*){
 sensor::begin_inline();
 uint32_t last=esp_cpu_get_cycle_count(),seq[2]={};
 float angle=0;
 while(true){
  uint32_t begin=esp_cpu_get_cycle_count();
  uint32_t elapsed_dt=begin-last;
  float dt=float(elapsed_dt)/(CPU_MHZ*1000000.0f);last=begin;
  sensor::poll_inline();
  sampling s;
  portENTER_CRITICAL(&mux);s=samples;portEXIT_CRITICAL(&mux);
  if(s.sequence[0]&&s.sequence[1]&&s.count[4]){
   float current[4],id[2],iq[2],duty[6];
   for(unsigned i=0;i<4;++i)current[i]=(voltage_mv[i][s.raw[i]]*.001f-offset_V[i])*CURRENT_GAIN;
   float bus=voltage_mv[4][s.raw[4]]*.003f;
   uint32_t calc=esp_cpu_get_cycle_count();
   angle+=TWO_PI*3*dt;if(angle>=TWO_PI)angle=fmodf(angle,TWO_PI);
   calculate(0,current[0],current[1],fmaxf(bus,1.0f),angle,id[0],iq[0],duty,dt);
   calculate(1,current[2],current[3],fmaxf(bus,1.0f),angle+1,id[1],iq[1],duty+3,dt);
   uint32_t written_begin=esp_cpu_get_cycle_count();unsigned errors=0;
   for(unsigned i=0;i<6;++i)errors+=mcpwm_comparator_set_compare_value(comparator[i],uint32_t(duty[i]*PEAK_TICKS))!=ESP_OK;
   uint32_t end=esp_cpu_get_cycle_count();
   portENTER_CRITICAL(&mux);
   ++latest.count;latest.compare_errors+=errors;latest.bus_V=bus;
   record(latest.calculation,written_begin-calc);record(latest.write,end-written_begin);
   record(latest.total,end-begin);record(latest.dt,elapsed_dt);
   for(unsigned m=0;m<2;++m){
    if(seq[m]==s.sequence[m])++latest.reused[m];else{++latest.fresh[m];if(seq[m])latest.skipped[m]+=s.sequence[m]-seq[m]-1;}
    seq[m]=s.sequence[m]; record(latest.age[m],calc-s.stamp[2*m]);
    latest.id_A[m]=id[m];latest.iq_A[m]=iq[m];
   }
   for(unsigned i=0;i<4;++i)latest.current_A[i]=current[i];
   for(unsigned i=0;i<6;++i)latest.duty[i]=duty[i];
   portEXIT_CRITICAL(&mux);
  }
  taskYIELD();
 }
}
}
bool init(){
 gpio_config_t pins{};pins.pin_bit_mask=(1ULL<<35)|(1ULL<<34)|(1ULL<<33)|(1ULL<<12)|(1ULL<<11)|(1ULL<<10);
 pins.mode=GPIO_MODE_INPUT;pins.pull_down_en=GPIO_PULLDOWN_ENABLE;
 ESP_ERROR_CHECK(gpio_config(&pins));ESP_ERROR_CHECK(gpio_dump_io_configuration(stdout,pins.pin_bit_mask));
 init_adc();
 mcpwm_timer_config_t cfg{};cfg.group_id=0;cfg.clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT;
 cfg.resolution_hz=RESOLUTION_HZ;cfg.period_ticks=PERIOD_TICKS;cfg.count_mode=MCPWM_TIMER_COUNT_MODE_UP_DOWN;cfg.intr_priority=3;
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&timer));
 for(unsigned i=0;i<3;++i){
  mcpwm_oper_handle_t op=nullptr;mcpwm_operator_config_t oc{};oc.group_id=0;oc.intr_priority=3;
  ESP_ERROR_CHECK(mcpwm_new_operator(&oc,&op));ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op,timer));
  for(unsigned j=0;j<2;++j){mcpwm_comparator_config_t cc{};cc.flags.update_cmp_on_tez=true;
   ESP_ERROR_CHECK(mcpwm_new_comparator(op,&cc,&comparator[2*i+j]));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[2*i+j],125));}
 }
 ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_hook,1));
 mcpwm_timer_event_callbacks_t cb{};cb.on_full=on_full;cb.on_empty=on_empty;
 ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer,&cb,nullptr));
 ESP_ERROR_CHECK(mcpwm_timer_enable(timer));ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control_task,"sensor_foc",6144,nullptr,TASK_PRIORITY,nullptr,1)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
 return true;
}
void get_package(state &out){
 portENTER_CRITICAL(&mux);out=latest;
 out.events=samples.events;out.adc_invalid=samples.invalid;out.gaps=samples.gaps;out.late_half=samples.late_half;
 out.acquisition=samples.pair;out.phase=samples.phase;out.skew=samples.skew;out.bus=samples.bus;
 out.interval_min=samples.interval_min;out.interval_max=samples.interval_max;
 for(unsigned i=0;i<5;++i){out.samples[i]=samples.count[i];out.raw[i]=samples.raw[i];out.raw_sum[i]=samples.sum[i];out.raw_square_sum[i]=samples.square[i];out.raw_min[i]=samples.low[i];out.raw_max[i]=samples.high[i];}
 portEXIT_CRITICAL(&mux);out.idle=__atomic_load_n(&idle_runs,__ATOMIC_RELAXED);
}
}
