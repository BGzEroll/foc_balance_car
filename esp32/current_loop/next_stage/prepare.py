from pathlib import Path
r=Path(__file__).resolve().parent
s=(r/'before/foc_current.cpp').read_text(encoding='utf-8')
def sub(a,b):
 global s
 assert a in s,a
 s=s.replace(a,b)
sub('enum {CAL=1,ALIGN=2,DTEST=3,QTEST=4};',
    'enum {CAL=1,ALIGN=2,DTEST=3,QTEST=4,QZERO=5,SPEED=6};\nfloat speed_target=3,velocity_kp=.04f,velocity_ki=.2f,current_limit=.35f;')
sub('calc_us,stage,pi_seq;','calc_us,stage,pi_seq,speed_ref;')
sub('sizeof(Row)==116','sizeof(Row)==120')
sub('float integrald=0,integralq=0,fd=0,fq=0;',
    'float integrald=0,integralq=0,fd=0,fq=0;\n unsigned velocity_previous=0;uint64_t velocity_seen=0;float velocity_integral=0,velocity_current=0;')
sub('sensor::poll_inline();','sensor::poll_inline(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE));')
sub('integrald=0;integralq=0;fd=0;fq=0;}',
    'integrald=0;integralq=0;fd=0;fq=0;velocity_previous=0;velocity_seen=0;velocity_integral=0;velocity_current=0;}')
sub('float id=ia*cs+beta*sn,iq=beta*cs-ia*sn,rd=0,rq=0,vd=0,vq=0;',
    '''float id=ia*cs+beta*sn,iq=beta*cs-ia*sn,rd=0,rq=0,vd=0,vq=0,speed_ref=0;
  if(mode==SPEED){
   float remain=duration_ms*.001f-t;
   if(t<.1f){stage=0;speed_ref=0;}
   else if(t<.45f){stage=1;speed_ref=speed_target*(t-.1f)/.35f;}
   else if(remain<.35f){stage=3;speed_ref=speed_target*fmaxf(remain/.35f,0);}
   else {stage=2;speed_ref=speed_target;}
  }''')
sub('}else{\n    rd=.15f;float ms=t*1000;',
    '''}else if(mode==SPEED){
    if(p.left_encoder.timestamp_us!=velocity_seen&&(!velocity_previous||s.us-velocity_previous>=1000)){
     float vdt=velocity_previous?(s.us-velocity_previous)*1e-6f:.001f;
     velocity_previous=s.us;velocity_seen=p.left_encoder.timestamp_us;
     float error=speed_ref+p.left_encoder.speed_rad_s;
     float candidate=velocity_integral+velocity_ki*vdt*error;
     float output=velocity_kp*error+candidate;
     velocity_current=fmaxf(-current_limit,fminf(current_limit,output));
     // Conditional integration avoids windup while preserving a smooth ramp.
     if(fabsf(output)<=current_limit||output*error<0)velocity_integral=candidate;
    }
    rq=velocity_current;rd=0;
   }else{
    rd=mode==QTEST?.15f:0;float ms=t*1000;''')
sub('r.calc_us=now32();r.stage=stage;', 'r.speed_ref=bits(speed_ref);r.calc_us=now32();r.stage=stage;')
sub('void status(){', '''void health(){
 sensor::package p{};sensor::get_package(p);unsigned now=now32();auto h=sensor::get_health();
 printf("HEALTH {\\"left_updates\\":%u,\\"right_updates\\":%u,\\"left_recoveries\\":%u,\\"right_recoveries\\":%u,\\"left_submit_errors\\":%u,\\"right_submit_errors\\":%u,\\"left_rejected_events\\":%u,\\"right_rejected_events\\":%u,\\"left_age_us\\":%u,\\"right_age_us\\":%u,\\"imu_age_us\\":%u}\\n",h.left_updates,h.right_updates,h.left_recoveries,h.right_recoveries,h.left_submit_errors,h.right_submit_errors,h.left_rejected_events,h.right_rejected_events,now-unsigned(p.left_encoder.timestamp_us),now-unsigned(p.right_encoder.timestamp_us),now-unsigned(p.imu.timestamp_us));fflush(stdout);
}
void status(){''')
sub('MALLOC_CAP_8BIT)));fflush(stdout);','MALLOC_CAP_8BIT)));health();fflush(stdout);')
sub('vTaskDelay(3);\n printf("META ', '''vTaskDelay(3);
 health();printf("SETUP {\\"speed_target\\":%.7f,\\"velocity_kp\\":%.7f,\\"velocity_ki\\":%.7f,\\"current_limit\\":%.7f,\\"row_bytes\\":120}\\n",speed_target,velocity_kp,velocity_ki,current_limit);
 printf("META ''')
sub(r'\"version\":8',r'\"version\":9')
sub('MA_CURRENT_V8','MA_CURRENT_V9')
sub('calc_us,stage,pi_seq");','calc_us,stage,pi_seq,speed_ref");')
sub('%.7f,%u,%u,%u\\n",s.us','%.7f,%u,%u,%u,%.7f\\n",s.us')
sub('r.calc_us,r.stage,r.pi_seq);','r.calc_us,r.stage,r.pi_seq,unbits(r.speed_ref));')
sub('float a=0,b=0,c=0,d=0;', '''if(!strcmp(line,"SENSOR_NACK")){sensor::request_test_nack();puts("SENSOR_NACK_QUEUED");return;}
 float a=0,b=0,c=0,d=0;
 if(sscanf(line,"VEL %f %f %f",&a,&b,&c)==3){
  if(!isfinite(a)||!isfinite(b)||!isfinite(c)||a<.005f||a>.2f||b<0||b>2||c<.1f||c>.8f){puts("ERROR velocity limits");return;}
  velocity_kp=a;velocity_ki=b;current_limit=c;puts("VEL_OK");return;
 }''')
sub('newmode=QTEST;','newmode=QTEST;else if(!strcmp(word,"QZERO"))newmode=QZERO;else if(!strcmp(word,"SPEED"))newmode=SPEED;')
sub('if(newmode<=ALIGN){', '''if(ms*10>N*dec){puts("ERROR record buffer");return;}
 if(newmode==SPEED){if(!zero_valid||fabsf(a)<.5f||fabsf(a)>5||ms<1000){puts("ERROR speed limits");return;}speed_target=a;}
 else if(newmode<=ALIGN){''')
sub('(newmode==QTEST&&a>.15f)', '((newmode==QTEST||newmode==QZERO)&&a>.20f)')
for field in ['left_updates','right_updates','left_recoveries','right_recoveries','left_submit_errors','right_submit_errors','left_rejected_events','right_rejected_events']:
 sub(',h.'+field,',unsigned(h.'+field+')')
sub(r'\"version\":9',r'\"version\":10');sub('MA_CURRENT_V9','MA_CURRENT_V10')
sub('kp=.030159289f,ki=113.097336f','kp=.060318579f,ki=226.194672f')
sub('velocity_kp=.04f,velocity_ki=.2f,current_limit=.35f','velocity_kp=.20f,velocity_ki=.4f,current_limit=.7f')
sub('ms>1500||dec<1','ms>(newmode==SPEED?4000U:1500U)||dec<1')
sub(r'\"version\":10',r'\"version\":11');sub('MA_CURRENT_V10','MA_CURRENT_V11')
(r/'foc_current.cpp').write_text(s.rstrip()+'\n',encoding='utf-8',newline='\n')
for file in ['sensor.cpp','sensor.h']:
 p=r/file
 p.write_text('\n'.join(line.rstrip() for line in p.read_text(encoding='utf-8').splitlines()).rstrip()+'\n',encoding='utf-8',newline='\n')
