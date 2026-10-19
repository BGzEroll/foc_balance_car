
#include <cstdint>
#include <cstdio>
#include <cassert>
#define IRAM_ATTR
using mcpwm_timer_handle_t=void*;struct mcpwm_timer_event_data_t{};
unsigned clock_us=0,epoch=0,queued[6]={},applied[6]={},queued_seq[2]={},queued_sample[2]={},applied_seq[2]={},applied_sample[2]={};
unsigned running=0,requested=0,mask=3,max_latch_age=0,first_latch_min[2]={},first_latch_max[2]={},first_latch_count[2]={},first_latch_sum[2]={};
unsigned latch_us=0,enable_pending=0,enable_epoch=0,deadline=0,heartbeat=0,bus_stamp=0,last_valid_sample_us[2]={},bus_raw=0,reason=0;
unsigned CHANNEL[5]={};uint16_t bus_low=0,bus_high=4095;
unsigned now32(){return clock_us;}
void stop_output(unsigned why){reason=why;running=0;requested=0;}
bool read_adc(unsigned,uint16_t&){return false;}
void reset(unsigned t=100000){clock_us=t;requested=running=1;mask=3;deadline=t+10000;heartbeat=bus_stamp=t;last_valid_sample_us[0]=last_valid_sample_us[1]=t;reason=0;}
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

int main(){
 reset();last_valid_sample_us[0]-=1000;on_empty(nullptr,nullptr,nullptr);assert(reason==0);
 reset();last_valid_sample_us[0]-=1001;on_empty(nullptr,nullptr,nullptr);assert(reason==8&&!running&&!requested);
 reset();last_valid_sample_us[1]-=1001;on_empty(nullptr,nullptr,nullptr);assert(reason==8);
 reset();mask=1;last_valid_sample_us[1]-=1001;on_empty(nullptr,nullptr,nullptr);assert(reason==0);
 reset();running=0;last_valid_sample_us[0]-=2000;on_empty(nullptr,nullptr,nullptr);assert(reason==0);
 reset(500);last_valid_sample_us[0]=0xffffffffU-400;on_empty(nullptr,nullptr,nullptr);assert(reason==0);
 reset(500);last_valid_sample_us[0]=0xffffffffU-600;on_empty(nullptr,nullptr,nullptr);assert(reason==8);
 reset();heartbeat-=3001;on_empty(nullptr,nullptr,nullptr);assert(reason==2);
 reset();bus_stamp-=5001;on_empty(nullptr,nullptr,nullptr);assert(reason==3);
 reset();deadline=clock_us;on_empty(nullptr,nullptr,nullptr);assert(reason==1);
 puts("PASS 10 TEZ callback fault cases: lost MA/MB feedback, inactive motor, startup, uint32 wrap, task stall, bus stall, deadline");
}
