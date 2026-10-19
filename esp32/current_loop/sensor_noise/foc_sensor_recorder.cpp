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

#include "sensor_record.inc"
namespace foc_current { namespace {
constexpr unsigned CPU_MHZ=160,BASELINE_COUNT=512,N=1800,ZERO=2000;
constexpr float PI2=6.28318530718f;
constexpr adc_channel_t CHANNEL[5]={ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_3,ADC_CHANNEL_4,ADC_CHANNEL_0};
adc_oneshot_unit_handle_t adc=nullptr;
uint16_t voltage_mv[5][4096]={};float offset_V[4]={};
struct {float baseline_raw[5]={};}samples;
mcpwm_timer_handle_t timer=nullptr,sample_timer=nullptr;
mcpwm_cmpr_handle_t cmp[6]={};mcpwm_gen_handle_t gen[6]={};
unsigned requested=0,running=0,enable_pending=0,enable_epoch=0,epoch=0;
unsigned queued[6]={ZERO,ZERO,ZERO,ZERO,ZERO,ZERO},applied[6]={ZERO,ZERO,ZERO,ZERO,ZERO,ZERO};
unsigned queued_seq[2]={},queued_sample[2]={},applied_seq[2]={},applied_sample[2]={},latch_us=0;
unsigned heartbeat=0,deadline=0,reason=0,ready=0,bus_raw=0,bus_stamp=0;
unsigned profile_start=0,duration_ms=0,decimate=1,records=0,logging=0,run_tag=0,mode=0,mask=1;
unsigned cmp_errors=0,late_commit=0,max_latch_age=0;
unsigned last_valid_sample_us[2]={};
unsigned first_latch_min[2]={0xffffffffU,0xffffffffU},first_latch_max[2]={},first_latch_count[2]={},first_latch_sum[2]={};
unsigned all_on[2]={},all_valid[2]={},all_bad_window[2]={},all_cross[2]={};
unsigned pi_updates[2]={},pi_skipped[2]={},bad_dt[2]={},max_age[2]={},max_sample_gap[2]={};
unsigned max_encoder_age[2]={},raw_min[4]={4095,4095,4095,4095},raw_max[4]={};
unsigned fault_seen=0,fault_us=0,fault_a=0,fault_b=0,fault_motor=0;
uint16_t cut_low[4]={},cut_high[4]={},bus_low=0,bus_high=4095;
float command_v=.06f,current_amp=.3f,kp=.060318579f,ki=226.194672f,filter_tf=0;
float voltage_limit=.12f,bus_ratio=.015f,zero_offset[2]={},direction[2]={-1,1},sense_gain[2]={-4,-4};
unsigned zero_valid[2]={};
enum {CAL=1,ALIGN=2,DTEST=3,QTEST=4,QZERO=5,SPEED=6};
float speed_target=3,velocity_kp=.20f,velocity_ki=.4f,current_limit=.7f;
struct Sample {unsigned us,cycle,seq,a,b,bus,ca,cb,cc,a0,a1,b0,b1,flags,applied_seq,applied_sample,latch_us;};
struct SampleBox {unsigned version=0;Sample s{};} feedback[2];
struct Command {unsigned tag,seq,sample_us,ca,cb,cc;};
struct CommandBox {unsigned version=0;Command c{};} command_box[2];
struct Row {Sample s;unsigned motor,theta,mech,speed,id,iq,rd,rq,vd,vq,calc_us,stage,pi_seq,speed_ref,vlimit,encoder_age;};
static_assert(sizeof(Row)==132);
Row *rows=nullptr;
struct Metrics {unsigned n=0,saturated=0;float id=0,id2=0,eq2=0,ed2=0;}metrics[2];
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
void publish_command(unsigned m,const Command&c){
 auto &box=command_box[m];__atomic_add_fetch(&box.version,1,__ATOMIC_ACQ_REL);
 const unsigned *p=reinterpret_cast<const unsigned*>(&c);unsigned *q=reinterpret_cast<unsigned*>(&box.c);
 for(unsigned i=0;i<sizeof(c)/4;++i)__atomic_store_n(q+i,p[i],__ATOMIC_RELAXED);
 __atomic_add_fetch(&box.version,1,__ATOMIC_RELEASE);
}
bool read_sample(unsigned m,Sample&out){
 auto &box=feedback[m];for(unsigned k=0;k<3;++k){unsigned v=__atomic_load_n(&box.version,__ATOMIC_ACQUIRE);if(v&1U)continue;
  unsigned *p=reinterpret_cast<unsigned*>(&out);const unsigned *q=reinterpret_cast<const unsigned*>(&box.s);
  for(unsigned i=0;i<sizeof(out)/4;++i)p[i]=__atomic_load_n(q+i,__ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);if(v==__atomic_load_n(&box.version,__ATOMIC_ACQUIRE))return true;
 }return false;
}
bool IRAM_ATTR on_empty(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 unsigned now=now32();++epoch;for(unsigned i=0;i<6;++i)applied[i]=queued[i];
 for(unsigned m=0;m<2;++m){
  if(running&&(mask&(1U<<m))&&queued_sample[m]&&queued_sample[m]!=applied_sample[m]){
   unsigned age=now-queued_sample[m];if(age<first_latch_min[m])first_latch_min[m]=age;if(age>first_latch_max[m])first_latch_max[m]=age;++first_latch_count[m];first_latch_sum[m]+=age;
  }
  applied_seq[m]=queued_seq[m];applied_sample[m]=queued_sample[m];if(running&&applied_sample[m]&&(mask&(1U<<m))){unsigned age=now-applied_sample[m];if(age>max_latch_age)max_latch_age=age;}}
 latch_us=now;if(enable_pending&&int32_t(epoch-enable_epoch)>=0){running=1;enable_pending=0;}
 if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&running)stop_output(7);
 if(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  if(int32_t(now-deadline)>=0)stop_output(1);
  else if(now-__atomic_load_n(&heartbeat,__ATOMIC_ACQUIRE)>3000)stop_output(2);
  else if(now-bus_stamp>5000)stop_output(3);
  // This monitor also covers a complete loss of sample publication: a busy
  // control task heartbeat alone does not prove fresh current feedback.
  else if(running&&((mask&1U&&now-last_valid_sample_us[0]>1000)||(mask&2U&&now-last_valid_sample_us[1]>1000)))stop_output(8);
 }
 static unsigned n=0;if(++n>=10){n=0;uint16_t raw=0;if(read_adc(CHANNEL[4],raw)){
  __atomic_store_n(&bus_raw,raw,__ATOMIC_RELEASE);__atomic_store_n(&bus_stamp,now32(),__ATOMIC_RELEASE);
  if(running&&(raw<bus_low||raw>bus_high))stop_output(3);
 }}return false;
}
bool IRAM_ATTR on_peak(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 if(!running&&!enable_pending&&__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  unsigned p=phase();if(p>=4000&&p<6400){
   for(unsigned op=0;op<3;++op){mcpwm_gen_force_reg_t f{};f.val=MCPWM0.operators[op].gen_force.val;
    f.gen_cntuforce_upmethod=1;
    f.gen_a_cntuforce_mode=(mask&(1U<<((2*op)/3)))?0:1;
    f.gen_b_cntuforce_mode=(mask&(1U<<((2*op+1)/3)))?0:1;
    MCPWM0.operators[op].gen_force.val=f.val;}
   enable_epoch=epoch+1;enable_pending=1;
  }
 }return false;
}
bool IRAM_ATTR on_commit(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 const bool active=running||__atomic_load_n(&requested,__ATOMIC_ACQUIRE);
 Command c[2]{};bool coherent[2]={};
 for(unsigned m=0;m<2;++m){auto &box=command_box[m];for(unsigned k=0;k<3;++k){unsigned v=__atomic_load_n(&box.version,__ATOMIC_ACQUIRE);if(v&1U)continue;
  unsigned *p=reinterpret_cast<unsigned*>(&c[m]);const unsigned *q=reinterpret_cast<const unsigned*>(&box.c);
  for(unsigned i=0;i<sizeof(Command)/4;++i)p[i]=__atomic_load_n(q+i,__ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);if(v==__atomic_load_n(&box.version,__ATOMIC_ACQUIRE)){coherent[m]=true;break;}
 }}
 if(!coherent[0]||!coherent[1])return false;
 for(unsigned m=0;m<2;++m)if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)||!(mask&(1U<<m))||c[m].tag!=__atomic_load_n(&run_tag,__ATOMIC_ACQUIRE)){c[m].ca=ZERO;c[m].cb=ZERO;c[m].cc=ZERO;c[m].seq=0;c[m].sample_us=0;}
 unsigned p=phase();if(p<6400||p>7440){if(active)++late_commit;return false;}
 unsigned levels[6]={c[0].ca,c[0].cb,c[0].cc,c[1].ca,c[1].cb,c[1].cc};
 for(unsigned i=0;i<6;++i)if(levels[i]!=queued[i])cmp_errors+=mcpwm_comparator_set_compare_value(cmp[i],levels[i])!=ESP_OK;
 unsigned end=phase();if(end<p||end>7840||cmp_errors){stop_output(4);return false;}
 for(unsigned i=0;i<6;++i)queued[i]=levels[i];
 for(unsigned m=0;m<2;++m){queued_seq[m]=c[m].seq;queued_sample[m]=c[m].sample_us;}
 return false;
}


bool IRAM_ATTR on_sample(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 static unsigned turn=0,seq[2]={};unsigned m=(turn++)&1U;Sample s{};s.us=now32();s.cycle=epoch;s.seq=++seq[m];
 s.ca=applied[3*m];s.cb=applied[3*m+1];s.cc=applied[3*m+2];
 s.applied_seq=applied_seq[m];s.applied_sample=applied_sample[m];s.latch_us=latch_us;
 bool enabled=running&&(mask&(1U<<m));s.a0=phase();unsigned t0=esp_cpu_get_cycle_count();uint16_t a=0,b=0;
 bool oka=read_adc(CHANNEL[2*m],a);s.a=a;s.a1=s.a0+(esp_cpu_get_cycle_count()-t0+1)/2;s.b0=s.a1;
 bool okb=read_adc(CHANNEL[2*m+1],b);s.b=b;s.b1=s.a0+(esp_cpu_get_cycle_count()-t0+1)/2;
 s.bus=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 bool wa=!enabled||(s.a0>=s.ca+160&&s.a1+160<=8000-s.ca),wb=!enabled||(s.b0>=s.cb+160&&s.b1+160<=8000-s.cb);
 bool rawok=oka&&okb&&a>0&&b>0&&a<4095&&b<4095;
 s.flags=(enabled?1:0)|(oka?2:0)|(okb?4:0)|(wa?8:0)|(wb?16:0)|((s.cycle==epoch&&s.b1<8000)?32:0);
 bool over=a<cut_low[2*m]||a>cut_high[2*m]||b<cut_low[2*m+1]||b>cut_high[2*m+1];
 if(enabled){unsigned raw[2]={a,b};for(unsigned k=0;k<2;++k){if(raw[k]<raw_min[2*m+k])raw_min[2*m+k]=raw[k];if(raw[k]>raw_max[2*m+k])raw_max[2*m+k]=raw[k];}}
 if(enabled&&(!rawok||over)){if(!fault_seen){fault_seen=1;fault_us=s.us-profile_start;fault_a=a;fault_b=b;fault_motor=m+1;}stop_output(rawok?6:5);}
 if(enabled&&wa&&wb&&rawok&&s.b1<8000){last_valid_sample_us[m]=s.us;}
 if(enabled&&logging){++all_on[m];if(wa&&wb&&rawok&&s.b1<8000)++all_valid[m];else ++all_bad_window[m];if(s.b1>=8000)++all_cross[m];}
 auto &box=feedback[m];__atomic_add_fetch(&box.version,1,__ATOMIC_ACQ_REL);
 unsigned *q=reinterpret_cast<unsigned*>(&box.s);const unsigned *p=reinterpret_cast<const unsigned*>(&s);
 for(unsigned i=0;i<sizeof(s)/4;++i)__atomic_store_n(q+i,p[i],__ATOMIC_RELAXED);
 __atomic_add_fetch(&box.version,1,__ATOMIC_RELEASE);return false;
}
Command modulation(float vd,float vq,float theta,float bus){
 float cs=cosf(theta),sn=sinf(theta);float a=vd*cs-vq*sn,b=vd*sn+vq*cs;
 float v[3]={a,-.5f*a+.8660254f*b,-.5f*a-.8660254f*b};
 float z=-.5f*(fmaxf(v[0],fmaxf(v[1],v[2]))+fminf(v[0],fminf(v[1],v[2])));Command result{};unsigned out[3]{};
 for(unsigned i=0;i<3;++i){float f=(.5f+(v[i]+z)/bus)*4000+.5f;unsigned c=unsigned(f);out[i]=c<320?320:(c>3680?3680:c);}
 result.ca=out[0];result.cb=out[1];result.cc=out[2];return result;
}
void control(void*){
 sensor::begin_inline();__atomic_store_n(&ready,1,__ATOMIC_RELEASE);
 unsigned seen[2]={},tag=0,previous[2]={},pi_seq[2]={},velocity_previous[2]={};
 uint64_t velocity_seen[2]={};float integrald[2]={},integralq[2]={},fd[2]={},fq[2]={},velocity_integral[2]={},velocity_current[2]={};
 while(true){
  sensor::poll_inline(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE));unsigned now=now32();sensor::package p{};sensor::get_package(p);
  __atomic_store_n(&heartbeat,now,__ATOMIC_RELEASE);
  unsigned current_tag=__atomic_load_n(&run_tag,__ATOMIC_ACQUIRE);
  if(tag!=current_tag){tag=current_tag;for(unsigned m=0;m<2;++m){previous[m]=0;pi_seq[m]=0;integrald[m]=0;integralq[m]=0;fd[m]=0;fq[m]=0;velocity_previous[m]=0;velocity_seen[m]=0;velocity_integral[m]=0;velocity_current[m]=0;}}
  for(unsigned m=0;m<2;++m){Sample s{};if(!read_sample(m,s)||s.seq==seen[m])continue;seen[m]=s.seq;
   if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE)||!(mask&(1U<<m))||int32_t(s.us-profile_start)<0)continue;
   now=now32();auto enc=m?p.right_encoder:p.left_encoder;
   unsigned age=now-s.us,encage=now-unsigned(enc.timestamp_us);if(age>max_age[m])max_age[m]=age;if(encage>max_encoder_age[m])max_encoder_age[m]=encage;
   if(age>1000||!enc.timestamp_us||encage>1500){stop_output(10);break;}
   if(fabsf(enc.speed_rad_s)>40){stop_output(9);break;}
   float t=float(s.us-profile_start)*1e-6f,theta=0;unsigned stage=0;
   if(mode==CAL){stage=t<.3f?0:(t<.6f?1:2);theta=stage==1?PI2*.25f:0;}
   else if(mode==ALIGN)theta=0;
   else{float sensor_age=float(int32_t(s.us-unsigned(enc.timestamp_us)))*1e-6f;theta=wrap(direction[m]*7*(enc.angle_rad+enc.speed_rad_s*sensor_age)-zero_offset[m]);}
   float ia=(voltage_mv[2*m][s.a]*.001f-offset_V[2*m])*sense_gain[m];
   float ib=(voltage_mv[2*m+1][s.b]*.001f-offset_V[2*m+1])*sense_gain[m];
   float beta=(ia+2*ib)*.5773502692f,cs=cosf(theta),sn=sinf(theta);
   float bus=voltage_mv[4][s.bus]*.003f;
   unsigned sampled_bus_stamp=__atomic_load_n(&bus_stamp,__ATOMIC_ACQUIRE);
   if(!isfinite(bus)||bus<5||bus>13||now32()-sampled_bus_stamp>3000){stop_output(3);break;}
   float limit=fminf(voltage_limit,bus_ratio*bus);
   float id=ia*cs+beta*sn,iq=beta*cs-ia*sn,rd=0,rq=0,vd=0,vq=0,speed_ref=0;
   if(mode==SPEED){float remain=duration_ms*.001f-t;
    if(t<.1f){stage=0;speed_ref=0;}else if(t<.75f){stage=1;speed_ref=speed_target*(t-.1f)/.65f;}
    else if(remain<.65f){stage=3;speed_ref=speed_target*fmaxf(remain/.65f,0);}else{stage=2;speed_ref=speed_target;}
   }
   bool valid=(s.flags&63)==63;
   if(mode>=DTEST&&previous[m]&&s.us-previous[m]>1000){++bad_dt[m];stop_output(8);break;}
   if(mode==CAL||mode==ALIGN)vd=fminf(limit,command_v*fminf(t/.05f,1.0f));
   else if(valid){unsigned dtus=previous[m]?s.us-previous[m]:200;previous[m]=s.us;
    if(dtus>max_sample_gap[m])max_sample_gap[m]=dtus;
    if(dtus==0||dtus>1000){++bad_dt[m];stop_output(8);break;}
    float dt=dtus*1e-6f;
    if(mode==DTEST){if(t<.05f){rd=0;stage=0;}else if(t<.10f){rd=.5f*current_amp;stage=1;}else if(t<.20f){rd=current_amp;stage=2;}else if(t<.30f){rd=.5f*current_amp;stage=3;}else{rd=0;stage=4;}}
    else if(mode==SPEED){if(enc.timestamp_us!=velocity_seen[m]&&(!velocity_previous[m]||s.us-velocity_previous[m]>=1000)){
      float vdt=velocity_previous[m]?(s.us-velocity_previous[m])*1e-6f:.001f;velocity_previous[m]=s.us;velocity_seen[m]=enc.timestamp_us;
      float error=speed_ref-direction[m]*enc.speed_rad_s,candidate=velocity_integral[m]+velocity_ki*vdt*error,output=velocity_kp*error+candidate;
      velocity_current[m]=fmaxf(-current_limit,fminf(current_limit,output));if(fabsf(output)<=current_limit||output*error<0)velocity_integral[m]=candidate;
     }rq=velocity_current[m];rd=0;
    }else{rd=mode==QTEST?.15f:0;float ms=t*1000;
     if((ms>=40&&ms<60)||(ms>=160&&ms<180)){rq=current_amp;stage=1;}else if((ms>=100&&ms<120)||(ms>=180&&ms<200)){rq=-current_amp;stage=2;}
    }
    if(!pi_seq[m]){fd[m]=id;fq[m]=iq;}else{float a=filter_tf>0?dt/(filter_tf+dt):1;fd[m]+=a*(id-fd[m]);fq[m]+=a*(iq-fq[m]);}
    float ed=rd-fd[m],eq=rq-fq[m],cd=integrald[m]+ki*dt*ed,cq=integralq[m]+ki*dt*eq;
    vd=kp*ed+cd;vq=kp*eq+cq;float mag=sqrtf(vd*vd+vq*vq);
    bool saturated=mag>limit;if(saturated){float a=limit/mag;vd*=a;vq*=a;integrald[m]=vd-kp*ed;integralq[m]=vq-kp*eq;}else{integrald[m]=cd;integralq[m]=cq;}
    ++pi_updates[m];++pi_seq[m];
    bool platform=mode==DTEST?(t>.14f&&t<.19f):(mode==SPEED&&stage==2&&t>1.0f);
    if(platform){auto &q=metrics[m];++q.n;q.id+=id;q.id2+=id*id;q.eq2+=eq*eq;q.ed2+=ed*ed;q.saturated+=saturated;}
   }else ++pi_skipped[m];
   if(valid||mode==CAL||mode==ALIGN){Command c=modulation(vd,vq,theta,bus);c.tag=tag;c.seq=pi_seq[m];c.sample_us=s.us;publish_command(m,c);}
   if(logging&&!sensor_record::owns_buffer()&&s.seq%decimate==0&&records<N){Row row{};row.s=s;row.motor=m+1;row.theta=bits(theta);row.mech=bits(enc.angle_rad);row.speed=bits(direction[m]*enc.speed_rad_s);
    row.id=bits(id);row.iq=bits(iq);row.rd=bits(rd);row.rq=bits(rq);row.vd=bits(vd);row.vq=bits(vq);row.vlimit=bits(limit);row.speed_ref=bits(speed_ref);row.encoder_age=encage;row.calc_us=now32();row.stage=stage;row.pi_seq=pi_seq[m];rows[records++]=row;}
  }taskYIELD();
 }
}

void health(){
 sensor::package p{};sensor::get_package(p);unsigned now=now32();auto h=sensor::get_health();
 printf("HEALTH {\"left_updates\":%u,\"right_updates\":%u,\"left_recoveries\":%u,\"right_recoveries\":%u,\"left_age_us\":%u,\"right_age_us\":%u,\"imu_age_us\":%u}\n",unsigned(h.left_updates),unsigned(h.right_updates),unsigned(h.left_recoveries),unsigned(h.right_recoveries),now-unsigned(p.left_encoder.timestamp_us),now-unsigned(p.right_encoder.timestamp_us),now-unsigned(p.imu.timestamp_us));
}
void status(){
 sensor::package p{};sensor::get_package(p);unsigned now=now32(),br=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 printf("STATUS {\"version\":24,\"running\":%u,\"requested\":%u,\"reason\":%u,\"mask\":%u,\"bus_V\":%.4f,\"bus_age_us\":%u,\"left\":%.7f,\"right\":%.7f,\"left_age_us\":%u,\"right_age_us\":%u,\"zero_valid\":[%u,%u],\"records\":%u,\"heap\":%u}\n",running,requested,reason,mask,voltage_mv[4][br]*.003f,now-__atomic_load_n(&bus_stamp,__ATOMIC_ACQUIRE),p.left_encoder.angle_rad,p.right_encoder.angle_rad,now-unsigned(p.left_encoder.timestamp_us),now-unsigned(p.right_encoder.timestamp_us),zero_valid[0],zero_valid[1],records,unsigned(heap_caps_get_free_size(MALLOC_CAP_8BIT)));health();fflush(stdout);
}
void dump(){
 if(running||requested){puts("ERROR running");return;}__atomic_store_n(&logging,0,__ATOMIC_RELEASE);vTaskDelay(3);health();
 printf("SETUP {\"speed_target\":%.7f,\"velocity_kp\":%.7f,\"velocity_ki\":%.7f,\"current_limit\":%.7f,\"row_bytes\":132,\"direction\":[%.0f,%.0f],\"zero_offset\":[%.9f,%.9f],\"sense_gain\":[%.2f,%.2f],\"bus_ratio\":%.8f,\"voltage_cap\":%.6f,\"offset_V\":[%.8f,%.8f,%.8f,%.8f]}\n",speed_target,velocity_kp,velocity_ki,current_limit,direction[0],direction[1],zero_offset[0],zero_offset[1],sense_gain[0],sense_gain[1],bus_ratio,voltage_limit,offset_V[0],offset_V[1],offset_V[2],offset_V[3]);
 printf("META {\"version\":24,\"mode\":%u,\"mask\":%u,\"pwm_hz\":10000,\"sample_hz_each\":5000,\"period_ticks\":8000,\"rows\":%u,\"duration_ms\":%u,\"decimate\":%u,\"reason\":%u,\"guard_us\":2,\"trigger_us\":30,\"commit_us\":80,\"profile_start\":%u,\"kp\":%.9f,\"ki\":%.9f,\"filter_tf\":%.8f,\"cmp_errors\":%u,\"missed_commit\":%u,\"max_latch_age_us\":%u,\"fault_seen\":%u,\"fault_time_us\":%u,\"fault_motor\":%u,\"fault_raw_a\":%u,\"fault_raw_b\":%u,\"full_rate_active\":[%u,%u],\"full_rate_valid\":[%u,%u],\"bad_window\":[%u,%u],\"cross\":[%u,%u],\"pi_updates\":[%u,%u],\"max_sample_age_us\":[%u,%u],\"max_sample_gap_us\":[%u,%u],\"max_encoder_age_us\":[%u,%u],\"raw_min\":[%u,%u,%u,%u],\"raw_max\":[%u,%u,%u,%u]}\n",mode,mask,records,duration_ms,decimate,reason,profile_start,kp,ki,filter_tf,cmp_errors,late_commit,max_latch_age,fault_seen,fault_us,fault_motor,fault_a,fault_b,all_on[0],all_on[1],all_valid[0],all_valid[1],all_bad_window[0],all_bad_window[1],all_cross[0],all_cross[1],pi_updates[0],pi_updates[1],max_age[0],max_age[1],max_sample_gap[0],max_sample_gap[1],max_encoder_age[0],max_encoder_age[1],raw_min[0],raw_min[1],raw_min[2],raw_min[3],raw_max[0],raw_max[1],raw_max[2],raw_max[3]);
 for(unsigned m=0;m<2;++m){
  printf("LATENCY {\"motor\":%u,\"first_activation_count\":%u,\"first_activation_min_us\":%u,\"first_activation_max_us\":%u,\"first_activation_mean_us\":%.3f}\n",m+1,first_latch_count[m],first_latch_count[m]?first_latch_min[m]:0,first_latch_max[m],first_latch_count[m]?float(first_latch_sum[m])/first_latch_count[m]:0);
  auto &q=metrics[m];float n=q.n?float(q.n):1;
  printf("METRIC {\"motor\":%u,\"n\":%u,\"id_mean_A\":%.8f,\"id_std_A\":%.8f,\"iq_error_rms_A\":%.8f,\"id_error_rms_A\":%.8f,\"saturation_fraction\":%.8f}\n",m+1,q.n,q.id/n,sqrtf(fmaxf(0,q.id2/n-(q.id/n)*(q.id/n))),sqrtf(q.eq2/n),sqrtf(q.ed2/n),q.saturated/n);}
 for(unsigned ch=0;ch<5;++ch){printf("LUT %u",ch);for(unsigned i=0;i<4096;++i)printf(",%u",voltage_mv[ch][i]);puts("");vTaskDelay(1);}
 puts("CSV us,cycle,seq,raw_a,raw_b,raw_bus,ca,cb,cc,a0,a1,b0,b1,flags,applied_seq,applied_sample_us,latch_us,motor,theta,angle,speed,id,iq,id_ref,iq_ref,vd,vq,calc_us,stage,pi_seq,speed_ref,vlimit,encoder_age_us");
 for(unsigned i=0;i<records;++i){const Row&r=rows[i];const Sample&s=r.s;
  printf("R %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%u,%u,%u,%.7f,%.7f,%u\n",s.us,s.cycle,s.seq,s.a,s.b,s.bus,s.ca,s.cb,s.cc,s.a0,s.a1,s.b0,s.b1,s.flags,s.applied_seq,s.applied_sample,s.latch_us,r.motor,unbits(r.theta),unbits(r.mech),unbits(r.speed),unbits(r.id),unbits(r.iq),unbits(r.rd),unbits(r.rq),unbits(r.vd),unbits(r.vq),r.calc_us,r.stage,r.pi_seq,unbits(r.speed_ref),unbits(r.vlimit),r.encoder_age);
  if(i%10==0)vTaskDelay(1);
 }puts("END");fflush(stdout);
}
void command(char*line){
 if(sensor_record::command(line,!running&&!requested))return;
 if(!strcmp(line,"STATUS")){status();return;}
 if(!strcmp(line,"STOP")){__atomic_store_n(&requested,0,__ATOMIC_RELEASE);vTaskDelay(2);status();return;}
 if(!strcmp(line,"DUMP")){dump();return;}if(running||requested){puts("ERROR running");return;}
 float a=0,b=0,c=0;unsigned motor=0;
 if(sscanf(line,"MOTOR %u",&motor)==1){if(motor<1||motor>3){puts("ERROR motor");return;}mask=motor;puts("MOTOR_OK");return;}
 if(sscanf(line,"ZERO %u %f %f",&motor,&a,&b)==3){if(motor<1||motor>2||!isfinite(a)||!isfinite(b)||fabsf(a)!=1){puts("ERROR zero");return;}direction[motor-1]=a;zero_offset[motor-1]=wrap(b);zero_valid[motor-1]=1;puts("ZERO_OK");return;}
 if(sscanf(line,"SENSE %u %f",&motor,&a)==2){if(motor<1||motor>2||!isfinite(a)||fabsf(a)!=4){puts("ERROR gain");return;}sense_gain[motor-1]=a;puts("SENSE_OK");return;}
 if(sscanf(line,"LIMIT %f %f",&a,&b)==2){if(!isfinite(a)||!isfinite(b)||a<.005f||a>.04f||b<=0||b>.4f){puts("ERROR limit");return;}bus_ratio=a;voltage_limit=b;puts("LIMIT_OK");return;}
 if(sscanf(line,"VEL %f %f %f",&a,&b,&c)==3){if(!isfinite(a)||!isfinite(b)||!isfinite(c)||a<.005f||a>.3f||b<0||b>2||c<.1f||c>.8f){puts("ERROR velocity");return;}velocity_kp=a;velocity_ki=b;current_limit=c;puts("VEL_OK");return;}
 if(sscanf(line,"GAINS %f %f %f",&a,&b,&c)==3){if(!isfinite(a)||!isfinite(b)||!isfinite(c)||a<0||a>.1f||b<0||b>300||c<0||c>.003f){puts("ERROR gains");return;}kp=a;ki=b;filter_tf=c;puts("GAINS_OK");return;}
 char word[16]{};unsigned ms=0,dec=0,newmode=0;if(sscanf(line,"%15s %f %u %u",word,&a,&ms,&dec)!=4){puts("ERROR command");return;}
 if(!strcmp(word,"CAL"))newmode=CAL;else if(!strcmp(word,"ALIGN"))newmode=ALIGN;else if(!strcmp(word,"DTEST"))newmode=DTEST;else if(!strcmp(word,"QTEST"))newmode=QTEST;else if(!strcmp(word,"QZERO"))newmode=QZERO;else if(!strcmp(word,"SPEED"))newmode=SPEED;
 if(!newmode||!isfinite(a)||ms<200||ms>(newmode==SPEED?4000U:1500U)||dec<1||dec>100){puts("ERROR limits");return;}
 if(ms*5*unsigned((mask&1)+((mask>>1)&1))>N*dec){puts("ERROR record buffer");return;}
 if(newmode==SPEED){if(fabsf(a)<.5f||fabsf(a)>28||ms<1800){puts("ERROR speed");return;}speed_target=a;}
 else if(newmode<=ALIGN){if(mask==3||a<=0||a>.12f){puts("ERROR calibration");return;}command_v=a;}
 else{if(a<=0||a>.5f||((newmode==QTEST||newmode==QZERO)&&a>.2f)){puts("ERROR current");return;}current_amp=a;}
 sensor::package p{};sensor::get_package(p);unsigned now=now32(),br=__atomic_load_n(&bus_raw,__ATOMIC_ACQUIRE);
 unsigned bus_check_stamp=__atomic_load_n(&bus_stamp,__ATOMIC_ACQUIRE);float bus=voltage_mv[4][br]*.003f;if(bus<5||bus>13||unsigned(now32()-bus_check_stamp)>3000){puts("ERROR bus");return;}
 for(unsigned m=0;m<2;++m)if(mask&(1U<<m)){auto e=m?p.right_encoder:p.left_encoder;if(!e.timestamp_us||now-unsigned(e.timestamp_us)>1500||(newmode>=DTEST&&!zero_valid[m])){puts("ERROR sensor/calibration");return;}}
 __atomic_store_n(&logging,0,__ATOMIC_RELEASE);vTaskDelay(2);
 records=0;reason=0;cmp_errors=0;late_commit=0;max_latch_age=0;fault_seen=0;fault_us=0;fault_a=0;fault_b=0;fault_motor=0;
 for(unsigned m=0;m<2;++m){all_on[m]=0;all_valid[m]=0;all_bad_window[m]=0;all_cross[m]=0;pi_updates[m]=0;pi_skipped[m]=0;bad_dt[m]=0;max_age[m]=0;max_sample_gap[m]=0;max_encoder_age[m]=0;metrics[m]={};first_latch_min[m]=0xffffffffU;first_latch_max[m]=0;first_latch_count[m]=0;first_latch_sum[m]=0;}
 for(unsigned i=0;i<4;++i){raw_min[i]=4095;raw_max[i]=0;}
 mode=newmode;duration_ms=ms;decimate=dec;profile_start=now32();last_valid_sample_us[0]=profile_start;last_valid_sample_us[1]=profile_start;deadline=profile_start+ms*1000;
 __atomic_add_fetch(&run_tag,1,__ATOMIC_RELEASE);__atomic_store_n(&logging,1,__ATOMIC_RELEASE);__atomic_store_n(&requested,1,__ATOMIC_RELEASE);puts("ACCEPTED");fflush(stdout);
}
void console(void*){
 fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);char line[160];unsigned n=0;puts("READY DUAL_CURRENT_V24_SENSOR_RECORDER default stopped; per-motor calibration required");status();
 while(true){char c;while(read(STDIN_FILENO,&c,1)==1){if(c=='\r')continue;if(c=='\n'){line[n]=0;command(line);n=0;}else if(n<sizeof(line)-1)line[n++]=c;}
  if(logging&&unsigned(now32()-profile_start)>=duration_ms*1000){__atomic_store_n(&logging,0,__ATOMIC_RELEASE);}
  vTaskDelay(5);}
}
}
bool init(){
 gpio_config_t pins{};pins.pin_bit_mask=(1ULL<<35)|(1ULL<<34)|(1ULL<<33)|(1ULL<<12)|(1ULL<<11)|(1ULL<<10);pins.mode=GPIO_MODE_INPUT;pins.pull_down_en=GPIO_PULLDOWN_ENABLE;ESP_ERROR_CHECK(gpio_config(&pins));init_adc();
 for(unsigned raw=0;raw<4096;++raw){if(voltage_mv[4][raw]*.003f<5)bus_low=raw;if(voltage_mv[4][raw]*.003f<=13)bus_high=raw;}
 for(unsigned ch=0;ch<4;++ch)for(unsigned raw=0;raw<4096;++raw){float a=(voltage_mv[ch][raw]*.001f-offset_V[ch])*4;if(a< -2)cut_low[ch]=raw;if(a<=2)cut_high[ch]=raw;}
 rows=static_cast<Row*>(heap_caps_malloc(sizeof(Row)*N,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));ESP_ERROR_CHECK(rows?ESP_OK:ESP_ERR_NO_MEM);
 mcpwm_timer_config_t cfg{};cfg.group_id=0;cfg.clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT;cfg.resolution_hz=80000000;cfg.period_ticks=8000;cfg.count_mode=MCPWM_TIMER_COUNT_MODE_UP_DOWN;cfg.intr_priority=3;ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&timer));
 constexpr int PIN[6]={35,34,33,12,11,10};
 for(unsigned opid=0;opid<3;++opid){mcpwm_oper_handle_t op=nullptr;mcpwm_operator_config_t oc{};oc.group_id=0;oc.intr_priority=3;ESP_ERROR_CHECK(mcpwm_new_operator(&oc,&op));ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op,timer));
  for(unsigned j=0;j<2;++j){unsigned i=2*opid+j;mcpwm_comparator_config_t cc{};cc.flags.update_cmp_on_tez=true;ESP_ERROR_CHECK(mcpwm_new_comparator(op,&cc,&cmp[i]));ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp[i],2000));
   mcpwm_generator_config_t gc{};gc.gen_gpio_num=PIN[i];ESP_ERROR_CHECK(mcpwm_new_generator(op,&gc,&gen[i]));ESP_ERROR_CHECK(mcpwm_generator_set_force_level(gen[i],0,true));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen[i],MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH)));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[i],MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmp[i],MCPWM_GEN_ACTION_LOW)));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[i],MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,cmp[i],MCPWM_GEN_ACTION_HIGH)));
  }
 }
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&sample_timer));mcpwm_sync_handle_t sync=nullptr;mcpwm_timer_sync_src_config_t sc{};sc.timer_event=MCPWM_TIMER_EVENT_EMPTY;ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timer,&sc,&sync));
 mcpwm_timer_sync_phase_config_t p{};p.sync_src=sync;p.count_value=1600;p.direction=MCPWM_TIMER_DIRECTION_UP;ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(sample_timer,&p));
 mcpwm_timer_event_callbacks_t sb{};sb.on_full=on_sample;sb.on_empty=on_commit;ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(sample_timer,&sb,nullptr));
 mcpwm_timer_event_callbacks_t cb{};cb.on_empty=on_empty;cb.on_full=on_peak;ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer,&cb,nullptr));
 ESP_ERROR_CHECK(mcpwm_timer_enable(sample_timer));ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control,"sensor_dual",8192,nullptr,5,nullptr,1)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);while(!__atomic_load_n(&ready,__ATOMIC_ACQUIRE))vTaskDelay(1);
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(sample_timer,MCPWM_TIMER_START_NO_STOP));ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
 sensor_record::init(rows,sizeof(Row)*N);
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(console,"dual_console",8192,nullptr,1,nullptr,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);return true;
}
}
