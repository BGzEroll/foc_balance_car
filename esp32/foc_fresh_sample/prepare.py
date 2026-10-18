from pathlib import Path

r=Path(__file__).parent
s=(r/'before.cpp').read_text(encoding='utf-8')
h=(r/'before.h').read_text(encoding='utf-8')
t=(r/'test_before.cpp').read_text(encoding='utf-8')

# Count at the actual math entry so telemetry distinguishes PI execution from task loops.
needle='''        {
            const float sine = sinf(angle);'''
assert needle in s
s=s.replace(needle,'''        {
            ++latest.pi_calls[motor];
            const float sine = sinf(angle);''',1)

start=s.index('void control_task(void*){')
end=s.index('\n}\n}\nbool init(){',start)+2
snapshot=s[s.index('  sampling s{};',start):s.index('  if(ENABLE_FOC',start)]
control=r'''void control_task(void*){
 sensor::begin_inline();
 __atomic_store_n(&sensor_ready,1,__ATOMIC_RELEASE);
 uint32_t last_loop=esp_cpu_get_cycle_count();
 uint32_t seen[2]={},previous_sample[2]={};
 bool primed[2]={};
 float angle[2]={0.0f,1.0f};
 // Experimental guard, not a tuned motor-protection threshold.
 constexpr uint32_t MAX_SAMPLE_GAP_CYCLES=CPU_MHZ*1000;
 while(true){
  const uint32_t begin=esp_cpu_get_cycle_count();
  const uint32_t loop_dt=begin-last_loop;last_loop=begin;
  sensor::poll_inline();
SNAPSHOT
  bool updated=false;
  if(ENABLE_FOC&&s.count[4]){
   const float measured_bus=voltage_mv[4][s.raw[4]]*.003f;
   for(unsigned m=0;m<2;++m){
    const uint32_t sequence=s.sequence[m];
    if(sequence==0)continue;
    if(sequence==seen[m]){++latest.reused[m];continue;}
    ++latest.fresh[m];
    if(seen[m])latest.skipped[m]+=sequence-seen[m]-1;
    seen[m]=sequence;
    // Both timestamps are from esp_timer's shared clock, represented in CPU-cycle units.
    // The midpoint of conversion-completion times is a representative pair time,
    // not a claim that both analog sample apertures coincide.
    const uint32_t sample_time=s.stamp[2*m]+(s.stamp[2*m+1]-s.stamp[2*m])/2;
    const uint32_t sample_dt=sample_time-previous_sample[m];
    previous_sample[m]=sample_time;
    if(!primed[m]){primed[m]=true;++latest.prime_count[m];continue;}
    if(sample_dt==0||sample_dt>MAX_SAMPLE_GAP_CYCLES){
     ++latest.bad_sample_dt[m];integral_d[m]=0;integral_q[m]=0;continue;
    }
    const float dt=float(sample_dt)/(CPU_MHZ*1000000.0f);
    float id=0,iq=0,duty[3];
    const float ia=(voltage_mv[2*m][s.raw[2*m]]*.001f-offset_V[2*m])*CURRENT_GAIN;
    const float ib=(voltage_mv[2*m+1][s.raw[2*m+1]]*.001f-offset_V[2*m+1])*CURRENT_GAIN;
    const uint32_t calc=esp_cpu_get_cycle_count();
    const uint32_t common_calc=uint32_t(esp_timer_get_time())*CPU_MHZ;
    angle[m]+=TWO_PI*3*dt;if(angle[m]>=TWO_PI)angle[m]=fmodf(angle[m],TWO_PI);
    calculate(m,ia,ib,fmaxf(measured_bus,1.0f),angle[m],id,iq,duty,dt);
    const uint32_t write_begin=esp_cpu_get_cycle_count();
    uint32_t errors=0;
    // Only this motor's complete triplet is written; the other motor is untouched.
    for(unsigned i=0;i<3;++i)errors+=mcpwm_comparator_set_compare_value(comparator[3*m+i],uint32_t(duty[i]*PEAK_TICKS))!=ESP_OK;
    const uint32_t end=esp_cpu_get_cycle_count();
    ++latest.updates[m];updated=true;latest.compare_errors+=errors;latest.bus_V=measured_bus;
    latest.current_A[2*m]=ia;latest.current_A[2*m+1]=ib;
    latest.id_A[m]=id;latest.iq_A[m]=iq;
    latest.integral_d[m]=integral_d[m];latest.integral_q[m]=integral_q[m];
    for(unsigned i=0;i<3;++i)latest.duty[3*m+i]=duty[i];
    record(latest.sample_dt[m],sample_dt);
    record(latest.calculation,write_begin-calc);record(latest.write,end-write_begin);
    record(latest.age[m],common_calc-sample_time);
   }
  }
  ++latest.loop_count;
  if(updated)++latest.count;else ++latest.no_update_loops;
  record(latest.total,esp_cpu_get_cycle_count()-begin);record(latest.dt,loop_dt);
  publish_stats();
  taskYIELD();
 }
}'''.replace('SNAPSHOT',snapshot.rstrip())
s=s[:start]+control+s[end:]
h=h.replace(' uint32_t rejected[2]={};',''' uint32_t rejected[2]={};
 uint32_t loop_count=0,no_update_loops=0,updates[2]={},pi_calls[2]={},prime_count[2]={},bad_sample_dt[2]={};
 float integral_d[2]={},integral_q[2]={};
 timing sample_dt[2];''')
t=t.replace('timing("calc",s.calculation,s.count);timing("write",s.write,s.count);timing("work",s.total,s.count);timing("dt",s.dt,s.count);timing("age_m1",s.age[0],s.count);timing("age_m2",s.age[1],s.count);',
'''timing("calc",s.calculation,s.updates[0]+s.updates[1]);timing("write",s.write,s.updates[0]+s.updates[1]);
  timing("work",s.total,s.loop_count);timing("dt",s.dt,s.loop_count);
  timing("age_m1",s.age[0],s.updates[0]);timing("age_m2",s.age[1],s.updates[1]);
  timing("sample_dt_m1",s.sample_dt[0],s.updates[0]);timing("sample_dt_m2",s.sample_dt[1],s.updates[1]);
  printf(",\\\"loops\\\":%u,\\\"no_update_loops\\\":%u,\\\"updates\\\":[%u,%u],\\\"pi_calls\\\":[%u,%u],\\\"prime_count\\\":[%u,%u],\\\"bad_sample_dt\\\":[%u,%u]",unsigned(s.loop_count),unsigned(s.no_update_loops),unsigned(s.updates[0]),unsigned(s.updates[1]),unsigned(s.pi_calls[0]),unsigned(s.pi_calls[1]),unsigned(s.prime_count[0]),unsigned(s.prime_count[1]),unsigned(s.bad_sample_dt[0]),unsigned(s.bad_sample_dt[1]));
  printf(",\\\"integral_d\\\":[%.6f,%.6f],\\\"integral_q\\\":[%.6f,%.6f]",s.integral_d[0],s.integral_d[1],s.integral_q[0],s.integral_q[1]);''')
assert 'sample_dt_m1' in t
for name,text in [('foc_current.cpp',s),('foc_current.h',h),('test.cpp',t)]:
 (r/name).write_text('\n'.join(line.rstrip() for line in text.splitlines())+'\n',encoding='utf-8',newline='\n')
