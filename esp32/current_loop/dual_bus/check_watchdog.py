"""Host fault tests of the actual TEZ callback, with peripheral I/O stubbed."""
from pathlib import Path
r=Path(__file__).resolve().parent;s=(r/'dual_foc_current.cpp').read_text(encoding='utf-8')
fn=s[s.index('bool IRAM_ATTR on_empty('):s.index('bool IRAM_ATTR on_peak(')]
stub=r'''
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
'''
cases=r'''
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
'''
(r/'watchdog_test.cpp').write_text(stub+fn+cases,encoding='utf-8',newline='\n')
