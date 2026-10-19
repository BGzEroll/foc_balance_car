"""Generate v5: release all MA force modes at one hardware TEZ, not sequentially."""
from pathlib import Path
r=Path(__file__).resolve().parent;out=r/'pwm10k_sync_start';out.mkdir(exist_ok=True)
s=(r/'pwm10k/foc_current.cpp').read_text(encoding='utf-8')
def change(a,b):
 global s
 assert a in s,a
 s=s.replace(a,b,1)
change('unsigned requested=0,running=0,deadline=0,heartbeat=0,reason=0;',
       'unsigned requested=0,running=0,deadline=0,heartbeat=0,reason=0;\nunsigned enable_pending=0,enable_epoch=0;')
change('Row *rows=nullptr;','Row *rows=nullptr;\nRow fault_row{};unsigned fault_seen=0;')
change('if(running){for(auto g:gen)mcpwm_generator_set_force_level(g,0,true);running=0;}',
       'if(running||enable_pending){for(auto g:gen)mcpwm_generator_set_force_level(g,0,true);running=0;enable_pending=0;}')
change('const unsigned now=now32(); ++epoch; applied=queued;',
       '''const unsigned now=now32(); ++epoch; applied=queued;
 // Hardware has already latched the queued force-disable at this TEZ.
 if(enable_pending&&int32_t(epoch-enable_epoch)>=0){running=1;enable_pending=0;}''')
a=s.index('bool IRAM_ATTR on_peak(');b=s.index('bool IRAM_ATTR on_sample(',a)
s=s[:a]+'''bool IRAM_ATTR on_peak(mcpwm_timer_handle_t,const mcpwm_timer_event_data_t*,void*){
 if(!running&&!enable_pending&&__atomic_load_n(&requested,__ATOMIC_ACQUIRE)){
  const unsigned p=phase();
  if(p>=500&&p<800){
   // This experiment owns operators 0 and 1, both connected to main timer 0.
   // One APB write per operator queues force-disable. Bit0 selects next TEZ.
   // Do not call the force API here: IDF resets its update method to immediate.
   for(unsigned op=0;op<2;++op){
    mcpwm_gen_force_reg_t f{};f.val=MCPWM0.operators[op].gen_force.val;
    f.gen_cntuforce_upmethod=1;f.gen_a_cntuforce_mode=0;f.gen_b_cntuforce_mode=0;
    MCPWM0.operators[op].gen_force.val=f.val;
   }
   enable_epoch=epoch+1;enable_pending=1;
  }
 }
 return false;
}
'''+s[b:]
change('if(enabled&&(!oka||!okb||row.a==0||row.b==0||row.a>=4095||row.b>=4095))stop_output(5);\n if(enabled&&(row.a<cut_low[0]||row.a>cut_high[0]||row.b<cut_low[1]||row.b>cut_high[1]))stop_output(6);',
       '''const bool bad_adc=!oka||!okb||row.a==0||row.b==0||row.a>=4095||row.b>=4095;
 const bool over=row.a<cut_low[0]||row.a>cut_high[0]||row.b<cut_low[1]||row.b>cut_high[1];
 if(enabled&&(bad_adc||over)){
  if(!fault_seen){fault_row=row;fault_seen=1;}
  stop_output(bad_adc?5:6);
 }''')
change('records=0;reason=0;all_on=0;', 'records=0;reason=0;fault_seen=0;fault_row=Row{};all_on=0;')
change('\\"version\\":4','\\"version\\":5,\\"start_mode\\":\\"hardware_tez_sync\\"')
change('\\"full_rate_cross\\":%u}\\n','\\"full_rate_cross\\":%u,\\"fault_seen\\":%u,\\"fault_time_us\\":%u,\\"fault_raw_a\\":%u,\\"fault_raw_b\\":%u,\\"fault_flags\\":%u,\\"fault_ca\\":%u,\\"fault_cb\\":%u,\\"fault_a0\\":%u,\\"fault_a1\\":%u,\\"fault_b0\\":%u,\\"fault_b1\\":%u}\\n')
change('cmp_errors,late_commit,all_on,all_valid,all_a,all_b,all_cross);',
       'cmp_errors,late_commit,all_on,all_valid,all_a,all_b,all_cross,fault_seen,fault_seen?fault_row.us-profile_start:0,fault_row.a,fault_row.b,fault_row.flags,fault_row.ca,fault_row.cb,fault_row.a0,fault_row.a1,fault_row.b0,fault_row.b1);')
change('READY MA_OPENLOOP v4 PWM10kHz','READY MA_OPENLOOP v5 PWM10kHz TEZ-sync start')
(out/'foc_current.cpp').write_text(s,encoding='utf-8',newline='\n')
for name in ['foc_current.h','test.cpp']:
 (out/name).write_text((r/'pwm10k'/name).read_text(encoding='utf-8'),encoding='utf-8',newline='\n')
