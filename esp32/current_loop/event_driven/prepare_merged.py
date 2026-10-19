"""Third arm: keep one owner task, wait on ADC/I2C event bits instead of polling."""
from pathlib import Path
r=Path(__file__).resolve().parent
s=(r/'foc_event.cpp').read_text(encoding='utf-8')
def sub(old,new):
 global s
 assert old in s,old
 s=s.replace(old,new)
sub('xTaskNotifyGive(control_task_handle);','xTaskNotify(control_task_handle,4U,eSetBits);')
sub('vTaskNotifyGiveFromISR(control_task_handle,&woken);','xTaskNotifyFromISR(control_task_handle,4U,eSetBits,&woken);')
sub('if(!EVENT_DRIVEN){sensor::begin_inline();}','sensor::begin_inline();')
sub('''while(true){
  if(EVENT_DRIVEN){''','''while(true){
  uint32_t events=0;
  if(EVENT_DRIVEN){''')
sub('unsigned n=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);','xTaskNotifyWait(0,UINT32_MAX,&events,1);')
sub('++sched_waits;++sched_wakes;sched_coalesced+=n>1?n-1:0;sched_wait_us+=now32()-t0;', '++sched_waits;sched_wakes+=events!=0;sched_wait_us+=now32()-t0;')
sub('if(!EVENT_DRIVEN){sensor::poll_inline(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE));}', 'sensor::process_inline_events(events,sensor_recovery_allowed());')
sub('EVENT_DRIVEN?"event_split":"poll_merged"','"event_merged"')
sub('if(EVENT_DRIVEN){ESP_ERROR_CHECK(sensor::start_event_task(sensor_recovery_allowed)?ESP_OK:ESP_FAIL);}','// I2C and FOC share this single task and its notification event bits.')
sub('EVENT_DRIVEN?"foc_event":"sensor_dual"','"sensor_foc_evt"')
sub(r'\"version\":22',r'\"version\":23')
sub('DUAL_CURRENT_V22_EVENT','DUAL_CURRENT_V23_MERGED_EVENTS')
(r/'foc_merged.cpp').write_text(s,encoding='utf-8',newline='\n')
sensor=(r/'sensor.cpp').read_text(encoding='utf-8')
old='''        xTaskNotifyWait(0,UINT32_MAX,&notification,wait_ticks);
        if(recovery_permission)'''
new='''        xTaskNotifyWait(0,UINT32_MAX,&notification,wait_ticks);
        process_inline_events(notification,allow_recovery);
    }
    void process_inline_events(uint32_t notification, bool allow_recovery){
        if(recovery_permission)'''
assert old in sensor;sensor=sensor.replace(old,new)
(r/'sensor_merged.cpp').write_text(sensor,encoding='utf-8',newline='\n')
h=(r/'sensor.h').read_text(encoding='utf-8').replace('void begin_inline();','void begin_inline();\n    void process_inline_events(uint32_t notification, bool allow_recovery);')
(r/'sensor_merged.h').write_text(h,encoding='utf-8',newline='\n')
host=(r/'run.py').read_text(encoding='utf-8').replace('[18,19,20,21,22]','[18,19,20,21,22,23]')
(r/'run.py').write_text(host,encoding='utf-8',newline='\n')
