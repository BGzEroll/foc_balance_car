#include "foc_current.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
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
constexpr unsigned CPU_MHZ=160,BASELINE_COUNT=512,N=2048;
constexpr float PI2=6.28318530718f;
constexpr adc_channel_t CHANNEL[5]={ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_3,ADC_CHANNEL_4,ADC_CHANNEL_0};
adc_oneshot_unit_handle_t adc=nullptr;
uint16_t voltage_mv[5][4096]={};float offset_V[4]={};
struct {float baseline_raw[5]={};}samples;
mcpwm_timer_handle_t timer=nullptr,sample_timer=nullptr;
mcpwm_cmpr_handle_t cmp[3]={};mcpwm_gen_handle_t gen[3]={};
constexpr unsigned ZERO=2000;
unsigned requested=0,running=0,enable_pending=0,enable_epoch=0,epoch=0;
unsigned queued[3]={ZERO,ZERO,ZERO},applied[3]={ZERO,ZERO,ZERO};
unsigned queued_seq=0,queued_sample=0,applied_seq=0,applied_sample=0,latch_us=0;
unsigned heartbeat=0,deadline=0,reason=0,ready=0,bus_raw=0,bus_stamp=0;
unsigned profile_start=0,duration_ms=0,decimate=1,records=0,logging=0,run_tag=0,mode=0;
unsigned all_on=0,all_valid=0,all_bad_window=0,all_cross=0,cmp_errors=0,late_commit=0;
unsigned pi_updates=0,pi_skipped=0,bad_dt=0,max_age=0,max_sample_gap=0;
unsigned fault_seen=0,fault_us=0,fault_a=0,fault_b=0,fault_flags=0;
uint16_t cut_low[2]={},cut_high[2]={};uint16_t bus_low=0,bus_high=4095;
float command_v=.06f,current_amp=.3f,kp=.015079645f,ki=56.548668f,filter_tf=0,voltage_limit=.12f;
float zero_offset=0;unsigned zero_valid=0;
enum {CAL=1,ALIGN=2,DTEST=3,QTEST=4};
struct Sample {
 unsigned us,cycle,seq,a,b,bus,ca,cb,cc,a0,a1,b0,b1,flags,applied_seq,applied_sample,latch_us;
};
struct SampleBox {unsigned version=0;Sample s{};} feedback;
struct Command {unsigned tag,seq,sample_us,ca,cb,cc;} ;
struct CommandBox {unsigned version=0;Command c{};} command_box;
struct Row {Sample s;unsigned theta,mech,speed,id,iq,rd,rq,vd,vq,calc_us,stage,pi_seq;};
static_assert(sizeof(Row)==116);
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
inline unsigned IRAM_ATTR phase(){mcpwm_timer_status_reg_t s{};s.val=MCPWM0.timer[0].timer_status.val;return (s.timer_direction?8000-s.timer_value:s.timer_value)%8000;}
inline unsigned bits(float f){unsigned b;memcpy(&b,&f,4);return b;}
inline float unbits(unsigned b){float f;memcpy(&f,&b,4);return f;}
float wrap(float x){x=fmodf(x,PI2);return x<0?x+PI2:x;}
void IRAM_ATTR stop_output(unsigned why){
 __atomic_store_n(&requested,0,__ATOMIC_RELEASE);if(!reason)reason=why;
 if(running||enable_pending){for(auto g:gen)mcpwm_generator_set_force_level(g,0,true);running=0;enable_pending=0;}
}
void publish_command(const Command&c){
 __atomic_add_fetch(&command_box.version,1,__ATOMIC_ACQ_REL);
 const unsigned *p=reinterpret_cast<const unsigned*>(&c);unsigned *q=reinterpret_cast<unsigned*>(&command_box.c);
 for(unsigned i=0;i<sizeof(c)/4;++i)__atomic_store_n(q+i,p[i],__ATOMIC_RELAXED);
 __atomic_add_fetch(&command_box.version,1,__ATOMIC_RELEASE);
}
bool read_sample(Sample&out){
 for(unsigned k=0;k<3;++k){unsigned v=__atomic_load_n(&feedback.version,__ATOMIC_ACQUIRE);if(v&1U)continue;
  unsigned *p=reinterpret_cast<unsigned*>(&out);const unsigned *q=reinterpret_cast<const unsigned*>(&feedback.s);
  for(unsigned i=0;i<sizeof(out)/4;++i)p[i]=__atomic_load_n(q+i,__ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);if(v==__atomic_load_n(&feedback.version,__ATOMIC_ACQUIRE))return true;
 }return false;
}
bool IRAM_ATTR on_empty(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 unsigned now=now32();++epoch;for(unsigned i=0;i<3;++i)applied[i]=queued[i];applied_seq=queued_seq;applied_sample=queued_sample;latch_us=now;
 if(enable_pending&&int32_t(epoch-enable_epoch)>=0){running=1;enable_pending=0;}
 if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&running)stop_output(7);
 if(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  if(int32_t(now-deadline)>=0)stop_output(1);
  else if(now-__atomic_load_n(&heartbeat,__ATOMIC_ACQUIRE)>3000)stop_output(2);
  else if(now-bus_stamp>5000)stop_output(3);
 }
 static unsigned n=0;
 if(++n>=10){n=0;uint16_t raw=0;if(read_adc(CHANNEL[4],raw)){
  __atomic_store_n(&bus_raw,raw,__ATOMIC_RELEASE);bus_stamp=now32();
  if(running&&(raw<bus_low||raw>bus_high))stop_output(3);
 }}return false;
}
bool IRAM_ATTR on_peak(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 if(!running&&!enable_pending&&__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  unsigned p=phase();if(p>=4000&&p<6400){
   for(unsigned op=0;op<2;++op){mcpwm_gen_force_reg_t f{};f.val=MCPWM0.operators[op].gen_force.val;
    f.gen_cntuforce_upmethod=1;f.gen_a_cntuforce_mode=0;f.gen_b_cntuforce_mode=0;MCPWM0.operators[op].gen_force.val=f.val;}
   enable_epoch=epoch+1;enable_pending=1;
  }
 }return false;
}
bool IRAM_ATTR on_commit(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 // Auxiliary EMPTY at TEZ+80us. Bounded snapshot; sole comparator writer.
 Command c{};bool coherent=false;
 for(unsigned k=0;k<3;++k){unsigned v=__atomic_load_n(&command_box.version,__ATOMIC_ACQUIRE);if(v&1U)continue;
  unsigned *p=reinterpret_cast<unsigned*>(&c);const unsigned *q=reinterpret_cast<const unsigned*>(&command_box.c);
  for(unsigned i=0;i<sizeof(c)/4;++i)p[i]=__atomic_load_n(q+i,__ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);if(v==__atomic_load_n(&command_box.version,__ATOMIC_ACQUIRE)){coherent=true;break;}
 }
 if(!coherent)return false;
 if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)||c.tag!=__atomic_load_n(&run_tag,__ATOMIC_ACQUIRE)){c.ca=ZERO;c.cb=ZERO;c.cc=ZERO;c.seq=0;c.sample_us=0;}
 unsigned p=phase();if(p<6400||p>7440){++late_commit;return false;}
 unsigned levels[3]={c.ca,c.cb,c.cc};for(unsigned i=0;i<3;++i)if(levels[i]!=queued[i])cmp_errors+=mcpwm_comparator_set_compare_value(cmp[i],levels[i])!=ESP_OK;
 unsigned end=phase();if(end<p||end>7840){stop_output(4);return false;}
 if(cmp_errors){stop_output(4);return false;}
 for(unsigned i=0;i<3;++i){queued[i]=levels[i];}queued_seq=c.seq;queued_sample=c.sample_us;return false;
}
bool IRAM_ATTR on_sample(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 static unsigned seq=0;Sample s{};s.us=now32();s.cycle=epoch;s.seq=++seq;
 s.ca=applied[0];s.cb=applied[1];s.cc=applied[2];
 s.applied_seq=applied_seq;s.applied_sample=applied_sample;s.latch_us=latch_us;
 bool enabled=running;s.a0=phase();unsigned t0=esp_cpu_get_cycle_count();uint16_t a=0,b=0;
 bool oka=read_adc(CHANNEL[0],a);s.a=a;s.a1=s.a0+(esp_cpu_get_cycle_count()-t0+1)/2;s.b0=s.a1;
 bool okb=read_adc(CHANNEL[1],b);s.b=b;s.b1=s.a0+(esp_cpu_get_cycle_count()-t0+1)/2;
 s.bus=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 bool wa=!enabled||(s.a0>=s.ca+160&&s.a1+160<=8000-s.ca);
 bool wb=!enabled||(s.b0>=s.cb+160&&s.b1+160<=8000-s.cb);
 bool rawok=oka&&okb&&a>0&&b>0&&a<4095&&b<4095;
 s.flags=(enabled?1:0)|(oka?2:0)|(okb?4:0)|(wa?8:0)|(wb?16:0)|((s.cycle==epoch&&s.b1<8000)?32:0);
 bool over=a<cut_low[0]||a>cut_high[0]||b<cut_low[1]||b>cut_high[1];
 if(enabled&&(!rawok||over)){
  if(!fault_seen){fault_seen=1;fault_us=s.us-profile_start;fault_a=a;fault_b=b;fault_flags=s.flags;}
  stop_output(rawok?6:5);
 }
 if(enabled&&logging){++all_on;if(wa&&wb&&rawok&&s.b1<8000)++all_valid;else ++all_bad_window;if(s.b1>=8000)++all_cross;}
 __atomic_add_fetch(&feedback.version,1,__ATOMIC_ACQ_REL);
 unsigned *q=reinterpret_cast<unsigned*>(&feedback.s);const unsigned *p=reinterpret_cast<const unsigned*>(&s);
 for(unsigned i=0;i<sizeof(s)/4;++i)__atomic_store_n(q+i,p[i],__ATOMIC_RELAXED);
 __atomic_add_fetch(&feedback.version,1,__ATOMIC_RELEASE);return false;
}
Command modulation(float vd,float vq,float theta,float bus){
 float cs=cosf(theta),sn=sinf(theta);float a=vd*cs-vq*sn,b=vd*sn+vq*cs;
 float v[3]={a,-.5f*a+.8660254f*b,-.5f*a-.8660254f*b};
 float z=-.5f*(fmaxf(v[0],fmaxf(v[1],v[2]))+fminf(v[0],fminf(v[1],v[2])));Command result{};unsigned out[3]{};
 for(unsigned i=0;i<3;++i){unsigned c=unsigned((.5f+(v[i]+z)/bus)*4000+.5f);c=c<320?320:(c>3680?3680:c);out[i]=c;}result.ca=out[0];result.cb=out[1];result.cc=out[2];return result;
}
void control(void*){
 sensor::begin_inline();__atomic_store_n(&ready,1,__ATOMIC_RELEASE);
 unsigned seen=0,tag=0,previous=0,pi_seq=0;float integrald=0,integralq=0,fd=0,fq=0;
 while(true){
  sensor::poll_inline();unsigned now=now32();sensor::package p{};sensor::get_package(p);
  __atomic_store_n(&heartbeat,now,__ATOMIC_RELEASE);
  Sample s{};if(!read_sample(s)||s.seq==seen){taskYIELD();continue;}seen=s.seq;
  now=now32();
  unsigned current_tag=__atomic_load_n(&run_tag,__ATOMIC_ACQUIRE);
  if(tag!=current_tag){tag=current_tag;previous=0;pi_seq=0;integrald=0;integralq=0;fd=0;fq=0;}
  if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){taskYIELD();continue;}
  if(int32_t(s.us-profile_start)<0){taskYIELD();continue;}
  unsigned age=now-s.us;if(age>max_age)max_age=age;
  if(age>1000||!p.left_encoder.timestamp_us||unsigned(now-unsigned(p.left_encoder.timestamp_us))>1500){stop_output(10);taskYIELD();continue;}
  if(fabsf(p.left_encoder.speed_rad_s)>15){stop_output(9);taskYIELD();continue;}
  float t=float(s.us-profile_start)*1e-6f,theta=0;unsigned stage=0;
  if(mode==CAL){stage=t<.3f?0:(t<.6f?1:2);theta=stage==1?PI2*.25f:0;}
  else if(mode==ALIGN)theta=0;
  else {
   float sensor_age=float(int32_t(s.us-unsigned(p.left_encoder.timestamp_us)))*1e-6f;
   theta=wrap(-7*(p.left_encoder.angle_rad+p.left_encoder.speed_rad_s*sensor_age)-zero_offset);
  }
  // Powered alpha/beta vector tests measured both sense channels inverted.
  float ia=(voltage_mv[0][s.a]*.001f-offset_V[0])*(-4),ib=(voltage_mv[1][s.b]*.001f-offset_V[1])*(-4);
  float beta=(ia+2*ib)*.5773502692f,cs=cosf(theta),sn=sinf(theta);
  float id=ia*cs+beta*sn,iq=beta*cs-ia*sn,rd=0,rq=0,vd=0,vq=0;
  bool valid=(s.flags&63)==63;
  if(mode>=DTEST&&previous&&s.us-previous>1000){++bad_dt;stop_output(8);taskYIELD();continue;}
  if(mode==CAL||mode==ALIGN){vd=command_v*fminf(t/.03f,1.0f);}
  else if(valid){
   unsigned dtus=previous?s.us-previous:100;previous=s.us;
   if(dtus>max_sample_gap)max_sample_gap=dtus;
   if(dtus==0||dtus>1000){++bad_dt;stop_output(8);taskYIELD();continue;}
   float dt=dtus*1e-6f;
   if(mode==DTEST){
    if(t<.05f){rd=0;stage=0;}else if(t<.10f){rd=.5f*current_amp;stage=1;}
    else if(t<.20f){rd=current_amp;stage=2;}else if(t<.30f){rd=.5f*current_amp;stage=3;}else {rd=0;stage=4;}
   }else{
    rd=.15f;float ms=t*1000;
    if((ms>=40&&ms<60)||(ms>=160&&ms<180)){rq=current_amp;stage=1;}
    else if((ms>=100&&ms<120)||(ms>=180&&ms<200)){rq=-current_amp;stage=2;}
   }
   if(!pi_seq){fd=id;fq=iq;}else{float a=filter_tf>0?dt/(filter_tf+dt):1;fd+=a*(id-fd);fq+=a*(iq-fq);}
   float ed=rd-fd,eq=rq-fq;
   float cd=integrald+ki*dt*ed,cq=integralq+ki*dt*eq;
   vd=kp*ed+cd;vq=kp*eq+cq;float mag=sqrtf(vd*vd+vq*vq);
   if(mag>voltage_limit){float a=voltage_limit/mag;vd*=a;vq*=a;integrald=vd-kp*ed;integralq=vq-kp*eq;}
   else{integrald=cd;integralq=cq;}
   ++pi_updates;++pi_seq;
  }else ++pi_skipped;
  // Invalid samples never integrate or publish a new current-loop command.
  if(valid||mode==CAL||mode==ALIGN){
   float bus=voltage_mv[4][s.bus]*.003f;
   if(bus<5||bus>13)stop_output(3);
   else {Command c=modulation(vd,vq,theta,bus);c.tag=tag;c.seq=pi_seq;c.sample_us=s.us;publish_command(c);}
  }
  if(logging&&s.seq%decimate==0&&records<N){
   Row r{};r.s=s;r.theta=bits(theta);r.mech=bits(p.left_encoder.angle_rad);r.speed=bits(p.left_encoder.speed_rad_s);
   r.id=bits(id);r.iq=bits(iq);r.rd=bits(rd);r.rq=bits(rq);r.vd=bits(vd);r.vq=bits(vq);
   r.calc_us=now32();r.stage=stage;r.pi_seq=pi_seq;rows[records++]=r;
  }
  taskYIELD();
 }
}
void status(){
 sensor::package p{};sensor::get_package(p);unsigned now=now32(),br=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 printf("STATUS {\"running\":%u,\"requested\":%u,\"reason\":%u,\"bus_V\":%.4f,\"left\":%.7f,\"left_age_us\":%u,\"zero_valid\":%u,\"records\":%u,\"heap\":%u}\n",running,requested,reason,voltage_mv[4][br]*.003f,p.left_encoder.angle_rad,now-unsigned(p.left_encoder.timestamp_us),zero_valid,records,unsigned(heap_caps_get_free_size(MALLOC_CAP_8BIT)));fflush(stdout);
}
void dump(){
 if(running||requested){puts("ERROR running");return;}
 __atomic_store_n(&logging,0,__ATOMIC_RELEASE);vTaskDelay(3);
 printf("META {\"version\":7,\"mode\":%u,\"pwm_hz\":10000,\"period_ticks\":8000,\"sample_hz\":10000,\"rows\":%u,\"duration_ms\":%u,\"decimate\":%u,\"reason\":%u,\"guard_us\":2,\"trigger_us\":30,\"commit_us\":80,\"profile_start\":%u,\"command_v\":%.7f,\"current_amp\":%.7f,\"kp\":%.9f,\"ki\":%.9f,\"filter_tf\":%.8f,\"voltage_limit\":%.6f,\"zero_offset\":%.8f,\"zero_valid\":%u,\"offset_V\":[%.7f,%.7f],\"gain_A_per_V\":-4,\"full_rate_active\":%u,\"full_rate_valid\":%u,\"bad_window\":%u,\"cross\":%u,\"pi_updates\":%u,\"pi_skipped\":%u,\"bad_dt\":%u,\"max_sample_age_us\":%u,\"max_sample_gap_us\":%u,\"cmp_errors\":%u,\"missed_commit\":%u,\"fault_seen\":%u,\"fault_time_us\":%u,\"fault_raw_a\":%u,\"fault_raw_b\":%u,\"fault_flags\":%u}\n",mode,records,duration_ms,decimate,reason,profile_start,command_v,current_amp,kp,ki,filter_tf,voltage_limit,zero_offset,zero_valid,offset_V[0],offset_V[1],all_on,all_valid,all_bad_window,all_cross,pi_updates,pi_skipped,bad_dt,max_age,max_sample_gap,cmp_errors,late_commit,fault_seen,fault_us,fault_a,fault_b,fault_flags);
 for(unsigned ch=0;ch<3;++ch){unsigned lut=ch==2?4:ch;printf("LUT %u",ch);for(unsigned i=0;i<4096;++i)printf(",%u",voltage_mv[lut][i]);puts("");vTaskDelay(1);}
 puts("CSV us,cycle,seq,raw_a,raw_b,raw_bus,ca,cb,cc,a0,a1,b0,b1,flags,applied_seq,applied_sample_us,latch_us,theta,left,speed,id,iq,id_ref,iq_ref,vd,vq,calc_us,stage,pi_seq");
 for(unsigned i=0;i<records;++i){const Row&r=rows[i];const Sample&s=r.s;
  printf("R %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%u,%u,%u\n",s.us,s.cycle,s.seq,s.a,s.b,s.bus,s.ca,s.cb,s.cc,s.a0,s.a1,s.b0,s.b1,s.flags,s.applied_seq,s.applied_sample,s.latch_us,unbits(r.theta),unbits(r.mech),unbits(r.speed),unbits(r.id),unbits(r.iq),unbits(r.rd),unbits(r.rq),unbits(r.vd),unbits(r.vq),r.calc_us,r.stage,r.pi_seq);
  if(i%10==0)vTaskDelay(1);
 }puts("END");fflush(stdout);
}
void command(char*line){
 if(!strcmp(line,"STATUS")){status();return;}
 if(!strcmp(line,"STOP")){__atomic_store_n(&requested,0,__ATOMIC_RELEASE);vTaskDelay(2);status();return;}
 if(!strcmp(line,"DUMP")){dump();return;}
 if(running||requested){puts("ERROR running");return;}
 float a=0,b=0,c=0,d=0;
 if(sscanf(line,"GAINS %f %f %f %f",&a,&b,&c,&d)==4){
  if(!isfinite(a)||!isfinite(b)||!isfinite(c)||!isfinite(d)||a<0||a>.1f||b<0||b>300||c<0||c>.003f||d<=0||d>.2f){puts("ERROR limits");return;}
  kp=a;ki=b;filter_tf=c;voltage_limit=d;puts("GAINS_OK");return;
 }
 if(sscanf(line,"ZERO %f",&a)==1){if(!isfinite(a)){puts("ERROR zero");return;}zero_offset=wrap(a);zero_valid=1;puts("ZERO_OK");return;}
 char word[16]{};unsigned ms=0,dec=0,newmode=0;
 if(sscanf(line,"%15s %f %u %u",word,&a,&ms,&dec)!=4){puts("ERROR command");return;}
 if(!strcmp(word,"CAL"))newmode=CAL;else if(!strcmp(word,"ALIGN"))newmode=ALIGN;else if(!strcmp(word,"DTEST"))newmode=DTEST;else if(!strcmp(word,"QTEST"))newmode=QTEST;
 if(!newmode||!isfinite(a)||ms<200||ms>1500||dec<1||dec>20){puts("ERROR limits");return;}
 if(newmode<=ALIGN){if(a<=0||a>.12f){puts("ERROR voltage");return;}command_v=a;}
 else {if(!zero_valid||a<=0||a>.5f||(newmode==QTEST&&a>.15f)){puts("ERROR calibration/current");return;}current_amp=a;}
 sensor::package p{};sensor::get_package(p);unsigned now=now32();unsigned br=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 if(!p.left_encoder.timestamp_us||now-unsigned(p.left_encoder.timestamp_us)>1500||voltage_mv[4][br]*.003f<5){puts("ERROR sensor/bus");return;}
 __atomic_store_n(&logging,0,__ATOMIC_RELEASE);vTaskDelay(2);
 records=0;reason=0;cmp_errors=0;late_commit=0;all_on=0;all_valid=0;all_bad_window=0;all_cross=0;pi_updates=0;pi_skipped=0;bad_dt=0;max_age=0;max_sample_gap=0;fault_seen=0;fault_us=0;fault_a=0;fault_b=0;fault_flags=0;
 mode=newmode;duration_ms=ms;decimate=dec;profile_start=now32();deadline=profile_start+ms*1000;
 __atomic_add_fetch(&run_tag,1,__ATOMIC_RELEASE);__atomic_store_n(&logging,1,__ATOMIC_RELEASE);__atomic_store_n(&requested,1,__ATOMIC_RELEASE);
 puts("ACCEPTED");fflush(stdout);
}
void console(void*){
 fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);char line[160];unsigned n=0;
 puts("READY MA_CURRENT_V7 default stopped; calibration required");status();
 while(true){char c;while(read(STDIN_FILENO,&c,1)==1){if(c=='\r')continue;if(c=='\n'){line[n]=0;command(line);n=0;}else if(n<sizeof(line)-1)line[n++]=c;}
  if(logging&&unsigned(now32()-profile_start)>=duration_ms*1000)__atomic_store_n(&logging,0,__ATOMIC_RELEASE);
  vTaskDelay(5);
 }
}
}
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
 mcpwm_timer_config_t cfg{};cfg.group_id=0;cfg.clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT;cfg.resolution_hz=80000000;cfg.period_ticks=8000;cfg.count_mode=MCPWM_TIMER_COUNT_MODE_UP_DOWN;cfg.intr_priority=3;
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&timer));
 constexpr int PIN[3]={35,34,33};
 for(unsigned opid=0;opid<2;++opid){
  mcpwm_oper_handle_t op=nullptr;mcpwm_operator_config_t oc{};oc.group_id=0;oc.intr_priority=3;ESP_ERROR_CHECK(mcpwm_new_operator(&oc,&op));ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op,timer));
  for(unsigned j=0;j<2&&2*opid+j<3;++j){unsigned i=2*opid+j;
   mcpwm_comparator_config_t cc{};cc.flags.update_cmp_on_tez=true;ESP_ERROR_CHECK(mcpwm_new_comparator(op,&cc,&cmp[i]));ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp[i],2000));
   mcpwm_generator_config_t gc{};gc.gen_gpio_num=PIN[i];ESP_ERROR_CHECK(mcpwm_new_generator(op,&gc,&gen[i]));
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(gen[i],0,true));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen[i],MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH)));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[i],MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmp[i],MCPWM_GEN_ACTION_LOW)));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[i],MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,cmp[i],MCPWM_GEN_ACTION_HIGH)));
  }
 }
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&sample_timer));
 mcpwm_sync_handle_t sync=nullptr;mcpwm_timer_sync_src_config_t sc{};sc.timer_event=MCPWM_TIMER_EVENT_EMPTY;ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timer,&sc,&sync));
 mcpwm_timer_sync_phase_config_t p{};p.sync_src=sync;p.count_value=1600;p.direction=MCPWM_TIMER_DIRECTION_UP;ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(sample_timer,&p));
 mcpwm_timer_event_callbacks_t sb{};sb.on_full=on_sample;sb.on_empty=on_commit;ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(sample_timer,&sb,nullptr));
 mcpwm_timer_event_callbacks_t cb{};cb.on_empty=on_empty;cb.on_full=on_peak;ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer,&cb,nullptr));
 ESP_ERROR_CHECK(mcpwm_timer_enable(sample_timer));ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control,"sensor_current",8192,nullptr,5,nullptr,1)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
 while(!__atomic_load_n(&ready,__ATOMIC_ACQUIRE))vTaskDelay(1);
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(sample_timer,MCPWM_TIMER_START_NO_STOP));ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(console,"current_console",8192,nullptr,1,nullptr,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);
 return true;
}
}

