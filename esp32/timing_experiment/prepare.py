from pathlib import Path
import argparse

parser=argparse.ArgumentParser()
parser.add_argument('--priority',type=int,default=0)
parser.add_argument('--irq',type=int,default=3)
parser.add_argument('--idle-yield',action=argparse.BooleanOptionalAction,default=True)
args=parser.parse_args()

root = Path(__file__).parent
old = (root / 'foc_baseline.cpp').read_text(encoding='utf-8')
calculate = old[old.index('        void calculate('):old.index('        /**', old.index('        void calculate('))]
calculate = calculate.replace('float &id, float &iq, float *duty)', 'float &id, float &iq, float *duty, float dt)')
calculate = calculate.replace('KI * DT', 'KI * dt')
init_adc = old[old.index('        void init_adc()'):old.index('\n    }\n\n    /**', old.index('        void init_adc()'))]
init_adc = init_adc.replace('latest.baseline_raw[i]', 'samples.baseline_raw[i]')
prefix = old[:old.index('namespace foc_current')]
prefix += '#include "hw/sensor.h"\n#include "esp_freertos_hooks.h"\n'
body = r'''
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
bool idle_hook(){__atomic_add_fetch(&idle_runs,1,__ATOMIC_RELAXED);return true;}
inline void record(timing &t,uint32_t n){t.sum+=n;if(n<t.minimum)t.minimum=n;if(n>t.maximum)t.maximum=n;}
'''
read_adc = old[old.index('        bool IRAM_ATTR read_adc('):old.index('        /**',old.index('        bool IRAM_ATTR read_adc('))]
body += read_adc + calculate + init_adc
body += r'''
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
  float dt=float(begin-last)/(CPU_MHZ*1000000.0f);last=begin;
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
   record(latest.total,end-begin);record(latest.dt,begin-(last-uint32_t(dt*CPU_MHZ*1000000.0f)));
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
 cfg.resolution_hz=RESOLUTION_HZ;cfg.period_ticks=PERIOD_TICKS;cfg.count_mode=MCPWM_TIMER_COUNT_MODE_UP_DOWN;cfg.intr_priority=1;
 ESP_ERROR_CHECK(mcpwm_new_timer(&cfg,&timer));
 for(unsigned i=0;i<3;++i){
  mcpwm_oper_handle_t op=nullptr;mcpwm_operator_config_t oc{};oc.group_id=0;oc.intr_priority=1;
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
'''
# Use the integer cycle difference for dt statistics (no float roundtrip).
body = body.replace('float dt=float(begin-last)', 'uint32_t elapsed_dt=begin-last;\n  float dt=float(elapsed_dt)')
body = body.replace('begin-(last-uint32_t(dt*CPU_MHZ*1000000.0f))', 'elapsed_dt')
if args.idle_yield:
 body=body.replace('bool idle_hook(){__atomic_add_fetch(&idle_runs,1,__ATOMIC_RELAXED);return true;}',
                   'bool idle_hook(){__atomic_add_fetch(&idle_runs,1,__ATOMIC_RELAXED);taskYIELD();return false;}')
body=body.replace('intr_priority=1',f'intr_priority={args.irq}')
(root/'foc_current.cpp').write_text(prefix+body,encoding='utf-8')
(root/'foc_current.h').write_text('''#pragma once
#include <stdint.h>
namespace foc_current {
constexpr uint32_t CPU_MHZ=160, TASK_PRIORITY=20;
struct timing{uint64_t sum=0;uint32_t maximum=0,minimum=UINT32_MAX;};
struct state{
 uint32_t count=0,events=0,adc_invalid=0,compare_errors=0,gaps=0,late_half=0,idle=0;
 uint32_t samples[5]={},fresh[2]={},reused[2]={},skipped[2]={};
 uint32_t interval_min=UINT32_MAX,interval_max=0;
 uint16_t raw[5]={},raw_min[5]={},raw_max[5]={};uint64_t raw_sum[5]={},raw_square_sum[5]={};
 float bus_V=0,current_A[4]={},id_A[2]={},iq_A[2]={},duty[6]={};
 timing acquisition,calculation,write,total,dt,phase,skew,bus,age[2];
};
bool init();void get_package(state&);
}
'''.replace('TASK_PRIORITY=20',f'TASK_PRIORITY={args.priority}'),encoding='utf-8')
s=(root/'sensor_baseline.cpp').read_text(encoding='utf-8')
start=s.index('    /**\n     * @brief 初始化传感器模块')
s=s[:start]+'''    // Inline mode is mutually exclusive with the standalone sensor task.
    void begin_inline(){
        i2c::init(xTaskGetCurrentTaskHandle());
        mpu6050::init();
        next_imu_time_us=sys_time::get_us_tick()+MPU_PERIOD_US;
        as5600::start_left_read();as5600::start_right_read();
        started=true;
    }
    void poll_inline(){
        uint32_t notification=0;
        xTaskNotifyWait(0,UINT32_MAX,&notification,0);
        if(notification&i2c::LEFT_DONE)handle_left_encoder();
        if(notification&i2c::RIGHT_DONE){if(right_reading_imu)handle_imu();else handle_right_encoder();}
    }
'''+s[start:]
(root/'sensor.cpp').write_text(s,encoding='utf-8')
(root/'app.cpp').write_text('#include "test.h"\nextern "C" void app_init(void){test::init();}\n',encoding='utf-8')
