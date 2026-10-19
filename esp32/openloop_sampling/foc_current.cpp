#include "foc_current.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "hal/adc_ll.h"
#pragma GCC diagnostic pop
#include "esp_cpu.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw/sensor.h"
#include "soc/mcpwm_struct.h"

namespace foc_current { namespace {
constexpr unsigned CPU_MHZ=160,BASELINE_COUNT=512,N=4096;
constexpr float PI2=6.28318530718f;
constexpr adc_channel_t CHANNEL[5]={ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_3,ADC_CHANNEL_4,ADC_CHANNEL_0};
adc_oneshot_unit_handle_t adc=nullptr;
uint16_t voltage_mv[5][4096]={}; float offset_V[4]={};
struct {float baseline_raw[5]={};} samples;
mcpwm_timer_handle_t timer=nullptr,sample_timer=nullptr;
mcpwm_cmpr_handle_t cmp[3]={}; mcpwm_gen_handle_t gen[3]={};
constexpr unsigned ZERO=125U|(125U<<8)|(125U<<16);
unsigned wanted=ZERO,queued=ZERO,applied=ZERO,epoch=0;
unsigned requested=0,running=0,deadline=0,heartbeat=0,reason=0;
unsigned bus_raw=0,bus_stamp=0,ready=0,logging=0,records=0,event=0,decimate=1;
unsigned capture_start=0,capture_end=0,profile_start=0;
unsigned ma_parity=2,all_on=0,all_valid=0,all_a=0,all_b=0,all_cross=0;
unsigned angle_bits=0,left_bits=0,right_bits=0,speed_bits=0;
unsigned cmp_errors=0,late_commit=0;
unsigned commit_before_min=500,commit_before_max=0,commit_after_min=500,commit_after_max=0,commit_hist[10]={};
uint16_t cut_low[2]={},cut_high[2]={};
uint16_t bus_low=0,bus_high=4095;
float command_v=0,command_speed=0; unsigned duration_ms=0;
// All timing phases are relative to main timer TEZ in 0.1 us ticks.
// a0/a1 bracket the entire conversion; analog aperture is inside this interval.
struct Row {
 unsigned us,cycle,angle,left,right,speed;
 uint16_t a,b,bus,ca,cb,cc,a0,a1,b0,b1,flags;
};
static_assert(sizeof(Row)==48);
Row *rows=nullptr;
        bool IRAM_ATTR read_adc(adc_channel_t channel, uint16_t &raw)
        {
            const unsigned idle_start = esp_cpu_get_cycle_count();
            while(SENS.sar_slave_addr1.meas_status != 0)
            {
                if(esp_cpu_get_cycle_count() - idle_start >= CPU_MHZ * 20 &&
                   SENS.sar_slave_addr1.meas_status != 0){return false;}
            }
            adc_oneshot_ll_set_channel(ADC_UNIT_1, channel);
            adc_oneshot_ll_start(ADC_UNIT_1);
            const unsigned start = esp_cpu_get_cycle_count();
            while(!adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE))
            {
                // 任务可能被外设中断抢占；超时前复查完成位，避免将已完成的转换误报失败。
                if(esp_cpu_get_cycle_count() - start >= CPU_MHZ * 20 &&
                   !adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE)){return false;}
            }
            raw = static_cast<uint16_t>(adc_oneshot_ll_get_raw_result(ADC_UNIT_1));
            return true;
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
            for(unsigned i = 0; i < 5; ++i)
            {
                ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, CHANNEL[i], &channel));
                adc_cali_curve_fitting_config_t config{};
                config.unit_id = ADC_UNIT_1;
                config.chan = CHANNEL[i];
                config.atten = ADC_ATTEN_DB_12;
                config.bitwidth = ADC_BITWIDTH_12;
                adc_cali_handle_t calibration = nullptr;
                ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&config, &calibration));
                for(unsigned raw = 0; raw < 4096; ++raw)
                {
                    int mv = 0;
                    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, raw, &mv));
                    voltage_mv[i][raw] = static_cast<uint16_t>(mv < 0 ? 0 : mv);
                }
                ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(calibration));
            }
            for(unsigned sample = 0; sample < BASELINE_COUNT; ++sample)
            {
                for(unsigned i = 0; i < 5; ++i)
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

            for(unsigned i = 0; i < 5; ++i){samples.baseline_raw[i] = baseline_raw[i];}

        }

inline unsigned IRAM_ATTR now32(){return unsigned(esp_timer_get_time());}
inline unsigned IRAM_ATTR phase(){mcpwm_timer_status_reg_t s{};s.val=MCPWM0.timer[0].timer_status.val;return (s.timer_direction?500-s.timer_value:s.timer_value)%500;}
inline unsigned bits(float f){unsigned b;memcpy(&b,&f,4);return b;}
inline float unbits(unsigned b){float f;memcpy(&f,&b,4);return f;}
void IRAM_ATTR stop_output(unsigned why){
 __atomic_store_n(&requested,0,__ATOMIC_RELEASE);
 if(!reason)reason=why;
 if(running){for(auto g:gen)mcpwm_generator_set_force_level(g,0,true);running=0;}
}
bool IRAM_ATTR on_empty(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 const unsigned now=now32(); ++epoch; applied=queued;
 if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&running)stop_output(7);
 if(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  if(int32_t(now-deadline)>=0)stop_output(1);
  else if(unsigned(now-__atomic_load_n(&heartbeat,__ATOMIC_ACQUIRE))>3000)stop_output(2);
  else if(unsigned(now-bus_stamp)>5000)stop_output(3);
 }
 const unsigned next=__atomic_load_n(&requested,__ATOMIC_ACQUIRE)?__atomic_load_n(&wanted,__ATOMIC_ACQUIRE):ZERO;
 // This ISR is the sole comparator writer. Submit the complete triplet just AFTER
 // TEZ; it latches at the NEXT TEZ. No task can race a partially written triplet.
 const unsigned before=phase();
 if(before<commit_before_min)commit_before_min=before;
 if(before>commit_before_max)commit_before_max=before;
 ++commit_hist[before/50];
 if(next==queued){/* Same complete triplet is already latched or pending. */}
 else if(before<250){
  for(unsigned i=0;i<3;++i)cmp_errors+=mcpwm_comparator_set_compare_value(cmp[i],(next>>(8*i))&255)!=ESP_OK;
  const unsigned after=phase();
  if(after<commit_after_min)commit_after_min=after;
  if(after>commit_after_max)commit_after_max=after;
  if(after<350&&after>=before)queued=next;
  else {++late_commit;if(requested||running)stop_output(4);}
 }else {++late_commit;if(requested||running)stop_output(4);}
 static unsigned n=0;
 // Learn MA parity from actual ISR observations; startup ordering can shift it.
 // The other cycle has no MA pair, so use it for the 1kHz bus conversion.
 if(++n>=20&&ma_parity<2&&(epoch&1U)!=ma_parity){n=0;uint16_t raw=0;if(read_adc(CHANNEL[4],raw)){__atomic_store_n(&bus_raw,raw,__ATOMIC_RELEASE);bus_stamp=now32();
   if(running&&(raw<bus_low||raw>bus_high))stop_output(3);}}
 return false;
}
bool IRAM_ATTR on_peak(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 if(!running&&__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  const unsigned p=phase(); unsigned mx=0;
  for(unsigned i=0;i<3;++i){unsigned c=(applied>>(8*i))&255;if(c>mx)mx=c;}
  // Release the low force only while all three PWM states are already low.
  if(p>mx+20&&p<500-mx-20){for(auto g:gen)mcpwm_generator_set_force_level(g,-1,true);running=1;}
 }
 return false;
}
bool IRAM_ATTR on_sample(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 const unsigned current=event++;
 if(current&1U)return false; // MA pair 10 kHz; MB remains input pulldown.
 ma_parity=epoch&1U;
 Row row{};row.us=now32();row.cycle=epoch;
 const unsigned active=applied;
 row.ca=active&255;row.cb=(active>>8)&255;row.cc=(active>>16)&255;
 const bool enabled=running;
 row.a0=phase();const unsigned t0=esp_cpu_get_cycle_count();
 const bool oka=read_adc(CHANNEL[0],row.a);row.a1=uint16_t(row.a0+(esp_cpu_get_cycle_count()-t0+15)/16);
 row.b0=uint16_t(row.a0+(esp_cpu_get_cycle_count()-t0+15)/16);
 const bool okb=read_adc(CHANNEL[1],row.b);row.b1=uint16_t(row.a0+(esp_cpu_get_cycle_count()-t0+15)/16);
 row.bus=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 // 2us is an experimental settling margin, not oscilloscope-validated analog settling.
 const bool wa=!enabled||(row.a0>=row.ca+20&&row.a1+20<=500-row.ca);
 const bool wb=!enabled||(row.b0>=row.cb+20&&row.b1+20<=500-row.cb);
 row.flags=(enabled?1:0)|(oka?2:0)|(okb?4:0)|(wa?8:0)|(wb?16:0)|((epoch==row.cycle&&row.b1<500)?32:0);
 if(enabled&&(!oka||!okb||row.a==0||row.b==0||row.a>=4095||row.b>=4095))stop_output(5);
 if(enabled&&(row.a<cut_low[0]||row.a>cut_high[0]||row.b<cut_low[1]||row.b>cut_high[1]))stop_output(6);
 if(__atomic_load_n(&logging,__ATOMIC_ACQUIRE)){
  if(enabled){++all_on;if(wa)++all_a;if(wb)++all_b;if(row.b1>=500)++all_cross;
   if(wa&&wb&&oka&&okb&&row.a>0&&row.a<4095&&row.b>0&&row.b<4095&&row.b1<500)++all_valid;}
  const unsigned elapsed=row.us-profile_start;
  if(elapsed>=capture_start&&elapsed<capture_end&&(current/2)%decimate==0&&records<N){
   row.angle=__atomic_load_n(&angle_bits,__ATOMIC_ACQUIRE);
   row.left=__atomic_load_n(&left_bits,__ATOMIC_ACQUIRE);row.right=__atomic_load_n(&right_bits,__ATOMIC_ACQUIRE);
   row.speed=__atomic_load_n(&speed_bits,__ATOMIC_ACQUIRE);rows[records++]=row;
  }
 }
 return false;
}
void control(void*){
 sensor::begin_inline();__atomic_store_n(&ready,1,__ATOMIC_RELEASE);
 unsigned previous=now32();float angle=0;
 while(true){
  sensor::poll_inline();const unsigned now=now32();
  sensor::package p{};sensor::get_package(p);
  __atomic_store_n(&left_bits,bits(p.left_encoder.angle_rad),__ATOMIC_RELEASE);
  __atomic_store_n(&right_bits,bits(p.right_encoder.angle_rad),__ATOMIC_RELEASE);
  __atomic_store_n(&speed_bits,bits(p.left_encoder.speed_rad_s),__ATOMIC_RELEASE);
  if(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
   const float t=float(now-profile_start)*1e-6f;
   const float ramp=fminf(t/0.4f,1.0f);
   const float dt=fminf(float(now-previous)*1e-6f,0.001f);
   angle=fmodf(angle+7*command_speed*ramp*dt,PI2);
   const unsigned br=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
   const float bus=voltage_mv[4][br]*.003f;
   if(bus<5||bus>13){__atomic_store_n(&requested,0,__ATOMIC_RELEASE);}
   else {
    const float q=command_v*fminf(t/0.15f,1.0f);
    const float a=-q*sinf(angle),b=q*cosf(angle);
    float v[3]={a,-.5f*a+.8660254f*b,-.5f*a-.8660254f*b};
    const float z=-.5f*(fmaxf(v[0],fmaxf(v[1],v[2]))+fminf(v[0],fminf(v[1],v[2])));
    unsigned pack=0;
    for(unsigned i=0;i<3;++i){unsigned c=unsigned((.5f+(v[i]+z)/bus)*250+.5f);c=c<20?20:(c>230?230:c);pack|=c<<(8*i);}
    __atomic_store_n(&wanted,pack,__ATOMIC_RELEASE);
    __atomic_store_n(&angle_bits,bits(angle),__ATOMIC_RELEASE);
   }
  }else angle=0;
  previous=now;__atomic_store_n(&heartbeat,now,__ATOMIC_RELEASE);taskYIELD();
 }
}
void status(){
 const unsigned raw=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 printf("STATUS {\"running\":%u,\"requested\":%u,\"reason\":%u,\"bus_V\":%.4f,\"left\":%.6f,\"right\":%.6f,\"speed\":%.6f,\"records\":%u,\"cmp_errors\":%u,\"late_commit\":%u,\"heap\":%u}\n",
 running,requested,reason,voltage_mv[4][raw]*.003f,unbits(__atomic_load_n(&left_bits,__ATOMIC_ACQUIRE)),unbits(__atomic_load_n(&right_bits,__ATOMIC_ACQUIRE)),unbits(__atomic_load_n(&speed_bits,__ATOMIC_ACQUIRE)),records,cmp_errors,late_commit,unsigned(heap_caps_get_free_size(MALLOC_CAP_8BIT)));fflush(stdout);
 printf("COMMIT %u %u %u %u HIST",commit_before_min,commit_before_max,commit_after_min,commit_after_max);
 for(auto n:commit_hist)printf(" %u",n);
 puts("");fflush(stdout);
}
void dump(){
 if(running||requested){puts("ERROR running");return;}
 __atomic_store_n(&logging,0,__ATOMIC_RELEASE);vTaskDelay(pdMS_TO_TICKS(5));
 printf("META {\"version\":3,\"rows\":%u,\"record_bytes\":48,\"uq\":%.6f,\"omega\":%.6f,\"duration_ms\":%u,\"decimate\":%u,\"reason\":%u,\"guard_us\":2,\"trigger_us\":11,\"offset_V\":[%.6f,%.6f],\"gain_A_per_V\":4,\"baseline_raw\":[%.3f,%.3f],\"cmp_errors\":%u,\"late_commit\":%u,\"full_rate_active\":%u,\"full_rate_valid\":%u,\"full_rate_a\":%u,\"full_rate_b\":%u,\"full_rate_cross\":%u}\n",records,command_v,command_speed,duration_ms,decimate,reason,offset_V[0],offset_V[1],samples.baseline_raw[0],samples.baseline_raw[1],cmp_errors,late_commit,all_on,all_valid,all_a,all_b,all_cross);
 // Raw LUT makes host/MATLAB conversion match the ADC calibration used on board.
 for(unsigned ch=0;ch<3;++ch){unsigned lut=ch==2?4:ch;printf("LUT %u",ch);for(unsigned j=0;j<4096;++j)printf(",%u",voltage_mv[lut][j]);puts("");vTaskDelay(1);}
 puts("CSV us,cycle,theta,left,right,speed,raw_a,raw_b,raw_bus,ca,cb,cc,a0,a1,b0,b1,flags");
 for(unsigned i=0;i<records;++i){const auto &r=rows[i];printf("R %u,%u,%.7f,%.7f,%.7f,%.7f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",r.us,r.cycle,unbits(r.angle),unbits(r.left),unbits(r.right),unbits(r.speed),r.a,r.b,r.bus,r.ca,r.cb,r.cc,r.a0,r.a1,r.b0,r.b1,r.flags);if(i%16==0)vTaskDelay(1);}
 puts("END");fflush(stdout);
}
void command(char *line){
 if(strcmp(line,"STATUS")==0){status();return;}
 if(strcmp(line,"STOP")==0){__atomic_store_n(&requested,0,__ATOMIC_RELEASE);vTaskDelay(2);status();return;}
 if(strcmp(line,"DUMP")==0){dump();return;}
 float v=0,w=0;unsigned ms=0,d=1,start=0;
 if(sscanf(line,"RUN %f %f %u %u %u",&v,&w,&ms,&d,&start)==5){
  if(running||requested||!isfinite(v)||!isfinite(w)||v<0||v>0.8f||fabsf(w)>20||ms<200||ms>4000||d<1||d>20||start>=ms){puts("ERROR limits");return;}
  const float bus=voltage_mv[4][__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE)]*.003f;
  if(bus<5||bus>13){puts("ERROR bus");return;}
  __atomic_store_n(&logging,0,__ATOMIC_RELEASE);vTaskDelay(2);
  records=0;reason=0;all_on=0;all_valid=0;all_a=0;all_b=0;all_cross=0;command_v=v;command_speed=w;duration_ms=ms;decimate=d;
  capture_start=start*1000;capture_end=ms*1000;profile_start=now32();deadline=profile_start+ms*1000;
  __atomic_store_n(&wanted,ZERO,__ATOMIC_RELEASE);__atomic_store_n(&logging,1,__ATOMIC_RELEASE);
  if(v>0)__atomic_store_n(&requested,1,__ATOMIC_RELEASE);
  puts("ACCEPTED");fflush(stdout);return;
 }
 puts("ERROR command");fflush(stdout);
}
void console(void*){
 fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);char line[128];unsigned n=0;
 puts("READY MA_OPENLOOP v3; waiting for RUN; MB pins remain inputs");status();
 while(true){char c;while(read(STDIN_FILENO,&c,1)==1){if(c=='\r')continue;if(c=='\n'){line[n]=0;command(line);n=0;}else if(n<sizeof(line)-1)line[n++]=c;}
  if(logging&&unsigned(now32()-profile_start)>=duration_ms*1000)__atomic_store_n(&logging,0,__ATOMIC_RELEASE);
  vTaskDelay(pdMS_TO_TICKS(5));
 }
}
} // anonymous
bool init(){
 gpio_config_t pins{};pins.pin_bit_mask=(1ULL<<35)|(1ULL<<34)|(1ULL<<33)|(1ULL<<12)|(1ULL<<11)|(1ULL<<10);pins.mode=GPIO_MODE_INPUT;pins.pull_down_en=GPIO_PULLDOWN_ENABLE;ESP_ERROR_CHECK(gpio_config(&pins));
 init_adc();
 for(unsigned raw=0;raw<4096;++raw){if(voltage_mv[4][raw]*.003f<5)bus_low=raw;if(voltage_mv[4][raw]*.003f<=13)bus_high=raw;}
 for(unsigned ch=0;ch<2;++ch){
  cut_low[ch]=0;cut_high[ch]=4095;
  for(unsigned raw=0;raw<4096;++raw){float a=(voltage_mv[ch][raw]*.001f-offset_V[ch])*4;
   if(a< -2.0f)cut_low[ch]=raw;
   if(a<=2.0f)cut_high[ch]=raw;}
 }
 rows=static_cast<Row*>(heap_caps_malloc(sizeof(Row)*N,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));ESP_ERROR_CHECK(rows?ESP_OK:ESP_ERR_NO_MEM);
 mcpwm_timer_config_t cfg{};cfg.group_id=0;cfg.clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT;cfg.resolution_hz=10000000;cfg.period_ticks=500;cfg.count_mode=MCPWM_TIMER_COUNT_MODE_UP_DOWN;cfg.intr_priority=3;
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&timer));
 constexpr int PIN[3]={35,34,33};
 for(unsigned opid=0;opid<2;++opid){
  mcpwm_oper_handle_t op=nullptr;mcpwm_operator_config_t oc{};oc.group_id=0;oc.intr_priority=3;ESP_ERROR_CHECK(mcpwm_new_operator(&oc,&op));ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op,timer));
  for(unsigned j=0;j<2&&2*opid+j<3;++j){unsigned i=2*opid+j;
   mcpwm_comparator_config_t cc{};cc.flags.update_cmp_on_tez=true;ESP_ERROR_CHECK(mcpwm_new_comparator(op,&cc,&cmp[i]));ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp[i],125));
   mcpwm_generator_config_t gc{};gc.gen_gpio_num=PIN[i];ESP_ERROR_CHECK(mcpwm_new_generator(op,&gc,&gen[i]));
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(gen[i],0,true));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen[i],MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH)));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[i],MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmp[i],MCPWM_GEN_ACTION_LOW)));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[i],MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,cmp[i],MCPWM_GEN_ACTION_HIGH)));
  }
 }
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&sample_timer));
 mcpwm_sync_handle_t sync=nullptr;mcpwm_timer_sync_src_config_t sc{};sc.timer_event=MCPWM_TIMER_EVENT_EMPTY;ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timer,&sc,&sync));
 mcpwm_timer_sync_phase_config_t p{};p.sync_src=sync;p.count_value=140;p.direction=MCPWM_TIMER_DIRECTION_UP;ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(sample_timer,&p));
 mcpwm_timer_event_callbacks_t sb{};sb.on_full=on_sample;ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(sample_timer,&sb,nullptr));
 mcpwm_timer_event_callbacks_t cb{};cb.on_empty=on_empty;cb.on_full=on_peak;ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer,&cb,nullptr));
 ESP_ERROR_CHECK(mcpwm_timer_enable(sample_timer));ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control,"sensor_openloop",6144,nullptr,5,nullptr,1)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
 while(!__atomic_load_n(&ready,__ATOMIC_ACQUIRE))vTaskDelay(1);
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(sample_timer,MCPWM_TIMER_START_NO_STOP));ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(console,"openloop_console",6144,nullptr,1,nullptr,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
 return true;
}
}
