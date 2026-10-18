from pathlib import Path
import argparse

p=argparse.ArgumentParser()
p.add_argument('--mode',choices=['split','oldlock','presample'],default='presample')
p.add_argument('--compute',type=int,default=1)
p.add_argument('--lead-us',type=int,default=12)
p.add_argument('--adc-core',type=int,choices=[0,1],default=0)
p.add_argument('--reject-late',action=argparse.BooleanOptionalAction,default=True)
args=p.parse_args()
if args.mode=='split' and args.adc_core==0:
 p.error('split mode requires --adc-core 1; use presample for the final core0 ADC design')
r=Path(__file__).parent
s=(r/'foc_before.cpp').read_text(encoding='utf-8')
h=(r/'header_before.h').read_text(encoding='utf-8')
t=(r/'test_before.cpp').read_text(encoding='utf-8')
s=s.replace('#include "esp_freertos_hooks.h"','#include "esp_freertos_hooks.h"\n#include "soc/mcpwm_struct.h"')
s=s.replace('state latest;',f'constexpr bool ENABLE_FOC={str(bool(args.compute)).lower()};\nstate latest, published;\nportMUX_TYPE adc_snapshot_mux=portMUX_INITIALIZER_UNLOCKED;')
s=s.replace('timing pair,bus,phase,skew;', 'timing pair,bus,phase,skew,atomic_phase,completion;\n uint32_t late_atomic=0,phase_hist[50]={},phase_disagree=0;')
helper='''
// Read count and direction in ONE APB transaction; do not combine two different instants.
inline uint32_t IRAM_ATTR phase_from_tep(){
 mcpwm_timer_status_reg_t st{};st.val=MCPWM0.timer[0].timer_status.val;
 // Use the raw next-counter value as a phase estimate with <=1 tick uncertainty.
 // Folding the SDK's +1 correction modulo251 at count250 would invent a 25us jump.
 return (st.timer_direction ? PEAK_TICKS-st.timer_value : PEAK_TICKS+st.timer_value)%PERIOD_TICKS;
}
'''
s=s.replace('bool IRAM_ATTR on_full(',helper+'bool IRAM_ATTR on_full(')
s=s.replace('uint32_t start=esp_cpu_get_cycle_count();\n // Callback', 'uint32_t start=esp_cpu_get_cycle_count();\n uint32_t atomic_ticks=phase_from_tep();\n // Callback')
s=s.replace('record(samples.pair,elapsed);record(samples.phase,ticks*CPU_MHZ/10);', '''record(samples.pair,elapsed);record(samples.phase,ticks*CPU_MHZ/10);
 record(samples.atomic_phase,atomic_ticks*CPU_MHZ/10);
 record(samples.completion,atomic_ticks*CPU_MHZ/10+elapsed);
 if(atomic_ticks*CPU_MHZ/10+elapsed>=CPU_MHZ*25)++samples.late_atomic;
 ++samples.phase_hist[(atomic_ticks/10)%50];
 if(ticks>atomic_ticks+10||atomic_ticks>ticks+10)++samples.phase_disagree;''')
if args.mode!='oldlock':
 # ISR owns samples; no lock shared with core0 or the FOC statistics.
 s=s.replace('portENTER_CRITICAL_ISR(&mux);','').replace('portEXIT_CRITICAL_ISR(&mux);','')
 # FOC needs only the data, sequence and timestamps; copy the smallest snapshot on core1.
 s=s.replace('  sampling s;\n  portENTER_CRITICAL(&mux);s=samples;portEXIT_CRITICAL(&mux);', '''  sampling s{};
  portENTER_CRITICAL(&adc_snapshot_mux);
  for(unsigned i=0;i<5;++i){s.raw[i]=samples.raw[i];s.stamp[i]=samples.stamp[i];}
  s.sequence[0]=samples.sequence[0];s.sequence[1]=samples.sequence[1];s.count[4]=samples.count[4];
  portEXIT_CRITICAL(&adc_snapshot_mux);''')
 # latest is now owned only by control_task; core0 reads published.
 s=s.replace('   portENTER_CRITICAL(&mux);','').replace('   portEXIT_CRITICAL(&mux);','')
 s=s.replace('  taskYIELD();','  publish_stats();\n  taskYIELD();')
 s=s.replace('if(s.sequence[0]&&s.sequence[1]&&s.count[4])','if(ENABLE_FOC&&s.sequence[0]&&s.sequence[1]&&s.count[4])')
 start=s.index('void get_package(state &out){')
 body=s[start:s.index('\n}\n}',start)+2]
 # Same conversion as the old observer, but performed on core1 every100ms.
 copy=body[body.index(' out.events='):body.index('\n portEXIT_CRITICAL(&mux);')]
 extra='''
 out.atomic_phase=samples.atomic_phase;out.completion=samples.completion;out.late_atomic=samples.late_atomic;
 out.phase_disagree=samples.phase_disagree;
 for(unsigned i=0;i<50;++i)out.phase_hist[i]=samples.phase_hist[i];
'''
 pub='''
void publish_stats(){
 static int64_t next=0;int64_t now=esp_timer_get_time();if(now<next)return;next=now+100000;
 state out=latest;
 portENTER_CRITICAL(&adc_snapshot_mux);
'''+copy+extra+'''
 portEXIT_CRITICAL(&adc_snapshot_mux);
 out.idle=__atomic_load_n(&idle_runs,__ATOMIC_RELAXED);
 portENTER_CRITICAL(&mux);published=out;portEXIT_CRITICAL(&mux);
}
'''
 s=s.replace('void control_task(void*){',pub+'void control_task(void*){')
 s=s.replace(body,'void get_package(state &out){portENTER_CRITICAL(&mux);out=published;portEXIT_CRITICAL(&mux);}')
else:
 s=s.replace('out.acquisition=samples.pair;', 'out.atomic_phase=samples.atomic_phase;out.completion=samples.completion;out.late_atomic=samples.late_atomic;out.phase_disagree=samples.phase_disagree;for(unsigned i=0;i<50;++i)out.phase_hist[i]=samples.phase_hist[i];\n out.acquisition=samples.pair;')
# Start high-rate ADC only after synchronous I2C/MPU configuration has completed.
s=s.replace('ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));','')
s=s.replace(' sensor::begin_inline();',' sensor::begin_inline();\n ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));')
if args.mode=='presample':
 lead=args.lead_us
 s=s.replace('mcpwm_timer_handle_t timer=nullptr;','mcpwm_timer_handle_t timer=nullptr,sample_timer=nullptr;')
 setup=f'''
 mcpwm_timer_config_t scfg=cfg;
 ESP_ERROR_CHECK(mcpwm_new_timer(&scfg,&sample_timer));
 mcpwm_sync_handle_t sync=nullptr;mcpwm_timer_sync_src_config_t sync_cfg{{}};
 sync_cfg.timer_event=MCPWM_TIMER_EVENT_EMPTY;
 ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timer,&sync_cfg,&sync));
 mcpwm_timer_sync_phase_config_t phase{{}};phase.sync_src=sync;phase.count_value={lead*10};phase.direction=MCPWM_TIMER_DIRECTION_UP;
 ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(sample_timer,&phase));
 mcpwm_timer_event_callbacks_t scb{{}};scb.on_full=on_full;
 ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(sample_timer,&scb,nullptr));
 ESP_ERROR_CHECK(mcpwm_timer_enable(sample_timer));
'''
 s=s.replace(' mcpwm_timer_event_callbacks_t cb{};',setup+' mcpwm_timer_event_callbacks_t cb{};')
 s=s.replace('cb.on_full=on_full;cb.on_empty=on_empty;','cb.on_empty=on_empty;')
 s=s.replace(' sensor::begin_inline();',' sensor::begin_inline();\n ESP_ERROR_CHECK(mcpwm_timer_start_stop(sample_timer,MCPWM_TIMER_START_NO_STOP));')
 # All latency statistics now reference the earlier hardware trigger, not TEP.
 # The main timer is still timer0, while sample_timer is timer1.
 s=s.replace('uint32_t atomic_ticks=phase_from_tep();',f'uint32_t atomic_ticks=(phase_from_tep()+{lead*10})%PERIOD_TICKS;')
 s=s.replace('uint32_t ticks=ev->direction==MCPWM_TIMER_DIRECTION_DOWN ? PEAK_TICKS-ev->count_value : PEAK_TICKS+ev->count_value;',
             'uint32_t ticks=atomic_ticks; (void)ev;')
 s=s.replace('CPU_MHZ*25)++samples.late_atomic',f'CPU_MHZ*{25+lead})++samples.late_atomic')
 s=s.replace('CPU_MHZ*25)++samples.late_half',f'CPU_MHZ*{25+lead})++samples.late_half')
if args.adc_core==0 and args.mode!='oldlock':
 # Allocation on core0 binds MCPWM interrupts there; worker remains pinned to core1.
 # Publish all ADC fields atomically, with a sequence lock, avoiding cross-core spin waits in ISR.
 s=s.replace('state latest, published;', '''state latest, published;
struct adc_frame {uint32_t version=0,raw[5]={},stamp[5]={},sequence[2]={},bus_count=0;};
adc_frame frame;
void IRAM_ATTR commit_adc(){
 __atomic_add_fetch(&frame.version,1,__ATOMIC_ACQ_REL);
 for(unsigned i=0;i<5;++i){__atomic_store_n(&frame.raw[i],samples.raw[i],__ATOMIC_RELAXED);__atomic_store_n(&frame.stamp[i],samples.stamp[i],__ATOMIC_RELAXED);}
 for(unsigned i=0;i<2;++i)__atomic_store_n(&frame.sequence[i],samples.sequence[i],__ATOMIC_RELAXED);
 __atomic_store_n(&frame.bus_count,samples.count[4],__ATOMIC_RELAXED);
 __atomic_add_fetch(&frame.version,1,__ATOMIC_RELEASE);
}
''')
 # commit_adc must follow the definition of samples.
 a=s.index('void IRAM_ATTR commit_adc(){');b=s.index('\n}',a)+2
 commit=s[a:b];s=s[:a]+s[b:];s=s.replace('} samples;','} samples;\n'+commit)
 s=s.replace(' if(ticks*CPU_MHZ/10+elapsed>=CPU_MHZ*'+str(25+args.lead_us)+')++samples.late_half;',
             ' if(ticks*CPU_MHZ/10+elapsed>=CPU_MHZ*'+str(25+args.lead_us)+')++samples.late_half;\n commit_adc();')
 s=s.replace('acquire(4);record(samples.bus,esp_cpu_get_cycle_count()-start);',
             'acquire(4);record(samples.bus,esp_cpu_get_cycle_count()-start);commit_adc();')
 snap='''  portENTER_CRITICAL(&adc_snapshot_mux);
  for(unsigned i=0;i<5;++i){s.raw[i]=samples.raw[i];s.stamp[i]=samples.stamp[i];}
  s.sequence[0]=samples.sequence[0];s.sequence[1]=samples.sequence[1];s.count[4]=samples.count[4];
  portEXIT_CRITICAL(&adc_snapshot_mux);'''
 replacement='''  static sampling cached{};
  for(unsigned attempt=0;attempt<3;++attempt){
   uint32_t v=__atomic_load_n(&frame.version,__ATOMIC_ACQUIRE);if(v&1U)continue;
   for(unsigned i=0;i<5;++i){s.raw[i]=__atomic_load_n(&frame.raw[i],__ATOMIC_RELAXED);s.stamp[i]=__atomic_load_n(&frame.stamp[i],__ATOMIC_RELAXED);}
   for(unsigned i=0;i<2;++i)s.sequence[i]=__atomic_load_n(&frame.sequence[i],__ATOMIC_RELAXED);
   s.count[4]=__atomic_load_n(&frame.bus_count,__ATOMIC_RELAXED);
   __atomic_thread_fence(__ATOMIC_ACQUIRE);
   if(__atomic_load_n(&frame.version,__ATOMIC_ACQUIRE)==v){cached=s;break;}
  }
  s=cached;'''
 assert snap in s
 s=s.replace(snap,replacement)
 # Per-core CCOUNT registers are not a shared clock. Use esp_timer for cross-core age.
 s=s.replace('samples.stamp[ch]=esp_cpu_get_cycle_count();',
             'samples.stamp[ch]=uint32_t(esp_timer_get_time())*CPU_MHZ;')
 s=s.replace('seq[m]=s.sequence[m]; record(latest.age[m],calc-s.stamp[2*m]);',
             'seq[m]=s.sequence[m]; record(latest.age[m],uint32_t(esp_timer_get_time())*CPU_MHZ-s.stamp[2*m]);')
 # Startup stays in core0 so event IRQ allocation remains there. Wait for sensor begin on core1.
 s=s.replace('volatile uint32_t idle_runs=0;','volatile uint32_t idle_runs=0;\nuint32_t sensor_ready=0;')
 startcalls='''
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(sample_timer,MCPWM_TIMER_START_NO_STOP));
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));'''
 assert startcalls in s
 s=s.replace(startcalls,'\n __atomic_store_n(&sensor_ready,1,__ATOMIC_RELEASE);')
 s=s.replace(' return true;\n}\nvoid get_package',
             ''' while(!__atomic_load_n(&sensor_ready,__ATOMIC_ACQUIRE))vTaskDelay(1);
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(sample_timer,MCPWM_TIMER_START_NO_STOP));
 ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
 return true;
}
void get_package''')
 # Stats are read on the owning ADC core, avoiding cross-core reads of ISR-owned uint64 fields.
 a=s.index(' portENTER_CRITICAL(&adc_snapshot_mux);',s.index('void publish_stats()'))
 b=s.index(' portEXIT_CRITICAL(&adc_snapshot_mux);',a)+len(' portEXIT_CRITICAL(&adc_snapshot_mux);')
 stats=s[a:b]
 s=s[:a]+s[b:]
 s=s.replace('void get_package(state &out){portENTER_CRITICAL(&mux);out=published;portEXIT_CRITICAL(&mux);}',
             'void get_package(state &out){portENTER_CRITICAL(&mux);out=published;portEXIT_CRITICAL(&mux);\n'+stats+'\n}')
 t=t.replace('1,nullptr,1)==pdPASS','1,nullptr,0)==pdPASS')
if args.reject_late and args.mode=='presample' and args.adc_core==0:
 s=s.replace('uint32_t late_atomic=0,phase_hist[50]={},phase_disagree=0;',
             'uint32_t late_atomic=0,phase_hist[50]={},phase_disagree=0,rejected[2]={};')
 s=s.replace(' unsigned motor=samples.events++ & 1U;', ''' unsigned motor=samples.events++ & 1U;
 const uint16_t old_a=samples.raw[motor*2],old_b=samples.raw[motor*2+1];
 const uint32_t old_ta=samples.stamp[motor*2],old_tb=samples.stamp[motor*2+1],old_invalid=samples.invalid;''')
 s=s.replace('acquire(motor*2); acquire(motor*2+1); ++samples.sequence[motor];',
             'acquire(motor*2); acquire(motor*2+1);')
 s=s.replace(' commit_adc();',f'''
 // Reject the entire pair on conversion failure or estimated completion at/after next TEZ.
 // 0.1us guard covers one raw timer tick of phase ambiguity. This is a schedule guard,
 // NOT validation of the electrical low-side window for arbitrary phase duty cycles.
 if(samples.invalid!=old_invalid || atomic_ticks*CPU_MHZ/10+elapsed+CPU_MHZ/10>=CPU_MHZ*{25+args.lead_us}){{
  samples.raw[motor*2]=old_a;samples.raw[motor*2+1]=old_b;
  samples.stamp[motor*2]=old_ta;samples.stamp[motor*2+1]=old_tb;++samples.rejected[motor];
 }}else ++samples.sequence[motor];
 commit_adc();''',1)
 # For core1-only candidate, commit via its existing protected snapshot.
 h=h.replace('timing acquisition,','uint32_t rejected[2]={};\n timing acquisition,')
 s=s.replace('out.phase_disagree=samples.phase_disagree;',
             'out.phase_disagree=samples.phase_disagree;out.rejected[0]=samples.rejected[0];out.rejected[1]=samples.rejected[1];')
 t=t.replace('timing("pair",','printf(",\\\"rejected\\\":[%u,%u]",unsigned(s.rejected[0]),unsigned(s.rejected[1]));\n  timing("pair",')
h=h.replace('timing acquisition,','uint32_t late_atomic=0,phase_disagree=0,phase_hist[50]={};\n timing atomic_phase,completion;\n timing acquisition,')
t=t.replace('timing("pair",','printf(",\\\"late_atomic\\\":%u,\\\"phase_disagree\\\":%u",unsigned(s.late_atomic),unsigned(s.phase_disagree));\n  timing("atomic_phase",s.atomic_phase,s.events);timing("completion",s.completion,s.events);\n  timing("pair",')
t=t.replace('printf("]}\\n");','printf("],\\\"phase_hist\\\":[");for(unsigned i=0;i<50;++i)printf("%s%u",i?",":"",unsigned(s.phase_hist[i]));printf("]}\\n");')
for name,text in [('foc_current.cpp',s),('foc_current.h',h),('test.cpp',t)]:
 text='\n'.join(line.rstrip() for line in text.splitlines())+'\n'
 (r/name).write_text(text,encoding='utf-8',newline='\n')
