"""Generate a matched polling baseline or sample-notified split-task bench."""
from pathlib import Path
import argparse
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['poll','event'],default='event');p.add_argument('--priority',type=int,choices=[5,7],default=5);a=p.parse_args()
s=(r/'v18_before.cpp').read_text(encoding='utf-8')
def sub(old,new):
 global s
 assert old in s,old
 s=s.replace(old,new)
sub('constexpr float PI2=',f'constexpr bool EVENT_DRIVEN={str(a.mode=="event").lower()};\nconstexpr float PI2=')
sub('constexpr float PI2=',f'constexpr unsigned FOC_PRIORITY={a.priority if a.mode=="event" else 5};\nconstexpr float PI2=')
sub('Row *rows=nullptr;', '''Row *rows=nullptr;
TaskHandle_t control_task_handle=nullptr;
unsigned sched_loops=0,sched_waits=0,sched_wakes=0,sched_coalesced=0,sched_wait_us=0,sched_max_work_us=0;
bool sensor_recovery_allowed(){
 return !__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&!__atomic_load_n(&running,__ATOMIC_ACQUIRE)&&!__atomic_load_n(&enable_pending,__ATOMIC_ACQUIRE);
}
void notify_control(){if(EVENT_DRIVEN&&control_task_handle)xTaskNotifyGive(control_task_handle);}
''')
sub('__atomic_add_fetch(&box.version,1,__ATOMIC_RELEASE);return false;', '''__atomic_add_fetch(&box.version,1,__ATOMIC_RELEASE);
 // Publish the whole sample before waking the consumer. Invalid groups never
 // trigger a new PI operation; TEZ independently monitors missing valid data.
 if(EVENT_DRIVEN&&enabled&&(s.flags&63)==63&&rawok&&control_task_handle){
  BaseType_t woken=pdFALSE;vTaskNotifyGiveFromISR(control_task_handle,&woken);return woken==pdTRUE;
 }
 return false;''')
sub('sensor::begin_inline();__atomic_store_n(&ready,1,__ATOMIC_RELEASE);','if(!EVENT_DRIVEN){sensor::begin_inline();}\n __atomic_store_n(&ready,1,__ATOMIC_RELEASE);')
sub('''while(true){
  sensor::poll_inline(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE));unsigned now=now32();sensor::package p{};sensor::get_package(p);''', '''while(true){
  if(EVENT_DRIVEN){
   unsigned wait_tag=__atomic_load_n(&run_tag,__ATOMIC_ACQUIRE),t0=now32();
   bool measure=__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&logging;
   unsigned n=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   if(measure&&wait_tag==__atomic_load_n(&run_tag,__ATOMIC_ACQUIRE)&&logging){
    ++sched_waits;++sched_wakes;sched_coalesced+=n>1?n-1:0;sched_wait_us+=now32()-t0;
   }
  }
  unsigned work_start=now32();
  if(!EVENT_DRIVEN){sensor::poll_inline(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE));}
  unsigned now=now32();sensor::package p{};sensor::get_package(p);
  if(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&logging){++sched_loops;}''')
sub('''  }taskYIELD();
 }
}''','''  }
  if(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)&&logging){unsigned us=now32()-work_start;if(us>sched_max_work_us)sched_max_work_us=us;}
  if(!EVENT_DRIVEN){taskYIELD();}
 }
}''')
sub('health();\n printf("SETUP ', '''health();
 printf("SCHED {\\"mode\\":\\"%s\\",\\"foc_priority\\":%u,\\"sensor_priority\\":5,\\"active_loops\\":%u,\\"wait_calls\\":%u,\\"wakes\\":%u,\\"coalesced_notifications\\":%u,\\"wait_elapsed_us\\":%u,\\"max_work_elapsed_us\\":%u}\\n",EVENT_DRIVEN?"event_split":"poll_merged",EVENT_DRIVEN?7U:5U,sched_loops,sched_waits,sched_wakes,sched_coalesced,sched_wait_us,sched_max_work_us);
 printf("SETUP ''')
s=s.replace('EVENT_DRIVEN?7U:5U','FOC_PRIORITY')
sub('health();fflush(stdout);','''printf("TASK_STATE {\\"foc\\":%u,\\"blocked_enum\\":%u,\\"event_driven\\":%u}\\n",unsigned(eTaskGetState(control_task_handle)),unsigned(eBlocked),unsigned(EVENT_DRIVEN));health();fflush(stdout);''')
sub('if(!strcmp(line,"STOP")){__atomic_store_n(&requested,0,__ATOMIC_RELEASE);vTaskDelay(2);status();return;}', 'if(!strcmp(line,"STOP")){__atomic_store_n(&requested,0,__ATOMIC_RELEASE);notify_control();vTaskDelay(2);status();return;}')
sub('records=0;reason=0;cmp_errors=0;', 'sched_loops=0;sched_waits=0;sched_wakes=0;sched_coalesced=0;sched_wait_us=0;sched_max_work_us=0;\n records=0;reason=0;cmp_errors=0;')
sub('__atomic_store_n(&requested,1,__ATOMIC_RELEASE);puts("ACCEPTED")', '__atomic_store_n(&heartbeat,now32(),__ATOMIC_RELEASE);__atomic_store_n(&requested,1,__ATOMIC_RELEASE);notify_control();puts("ACCEPTED")')
sub('ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control,"sensor_dual",8192,nullptr,5,nullptr,1)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);', '''if(EVENT_DRIVEN){ESP_ERROR_CHECK(sensor::start_event_task(sensor_recovery_allowed)?ESP_OK:ESP_FAIL);}
 ESP_ERROR_CHECK(xTaskCreatePinnedToCore(control,EVENT_DRIVEN?"foc_event":"sensor_dual",8192,nullptr,FOC_PRIORITY,&control_task_handle,1)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);''')
version=(22 if a.priority==5 else 21) if a.mode=='event' else 19
sub(r'\"version\":18',rf'\"version\":{version}')
sub('DUAL_CURRENT_V18_INTERLEAVED',f'DUAL_CURRENT_V{version}_{a.mode.upper()}')
(r/f'foc_{a.mode}.cpp').write_text(s,encoding='utf-8',newline='\n')

sensor=(r/'sensor_before.cpp').read_text(encoding='utf-8')
sensor=sensor.replace('#include "esp_err.h"','#include "esp_err.h"\n#include "esp_idf_version.h"')
sensor=sensor.replace('''// The ESP-IDF driver performs the yield after this callback returns.
            return task_woken == pdTRUE;''','''// Verified in the installed IDF v6.0.2 async driver: the callback
            // return value is ignored, so its local HPTaskAwoken misses this
            // task notification. Request a switch explicitly on this version.
#if ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(6, 0, 2)
            if(task_woken == pdTRUE){portYIELD_FROM_ISR();}
#endif
            return task_woken == pdTRUE;''')
sensor=sensor.replace('bool started = false;', 'bool started = false;\n        bool (*recovery_permission)() = nullptr;\n        unsigned event_ready = 0;')
start=sensor.index('        void task(void *)');end=sensor.index('    // Inline mode',start)
sensor=sensor[:start]+'''        void task(void *)
        {
            begin_inline();
            __atomic_store_n(&event_ready,1,__ATOMIC_RELEASE);
            while(true){poll_inline(true,1);}
        }
    }

'''+sensor[end:]
sensor=sensor.replace('void poll_inline(bool allow_recovery){','void poll_inline(bool allow_recovery, uint32_t wait_ticks){')
sensor=sensor.replace('xTaskNotifyWait(0,UINT32_MAX,&notification,0);','''xTaskNotifyWait(0,UINT32_MAX,&notification,wait_ticks);
        if(recovery_permission){allow_recovery=allow_recovery&&recovery_permission();}''')
pos=sensor.index('    health_data get_health()')
sensor=sensor[:pos]+'''    bool start_event_task(bool (*can_recover)()){
        if(started){return false;}
        recovery_permission=can_recover;
        if(!init()){return false;}
        for(unsigned i=0;i<2000;++i){
            if(__atomic_load_n(&event_ready,__ATOMIC_ACQUIRE)){return true;}
            vTaskDelay(1);
        }
        return false;
    }
'''+sensor[pos:]
(r/'sensor.cpp').write_text(sensor,encoding='utf-8',newline='\n')
h=(r/'sensor_before.h').read_text(encoding='utf-8')
h=h.replace('void poll_inline(bool allow_recovery = true);','''void poll_inline(bool allow_recovery = true, uint32_t wait_ticks = 0);
    // One owner task for I2C completion notifications; mutually exclusive with inline mode.
    bool start_event_task(bool (*can_recover)());''')
(r/'sensor.h').write_text(h,encoding='utf-8',newline='\n')
host=(r.parent/'dual_bus/run_dual.py').read_text(encoding='utf-8')
host=host.replace('[13,14,15,16,17,18]','[18,19,20,21,22]')
host=host.replace("'metric':metric,'latency'", "'metric':metric,'sched':parse('SCHED '),'latency'")
host=host.replace("'status':parse('STATUS ')","'task_state':parse('TASK_STATE '),'status':parse('STATUS ')")
(r/'run.py').write_text(host,encoding='utf-8',newline='\n')
