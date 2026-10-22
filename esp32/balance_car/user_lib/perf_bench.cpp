// Test-only instrumentation of the actual extracted module, not a FOC rewrite.
#include "hw/motor.h"
namespace pwm { struct sample; }
static void observe_sample(unsigned, const pwm::sample &, bool);
static void observe_control(unsigned, const motor::current_info &, const motor::target_current &);
#define MOTOR_SAMPLE_HOOK observe_sample
#define MOTOR_BENCH_MANUAL_ARM 1
#include "hw/motor.cpp"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>

namespace bench {
struct Stats { unsigned n=0,saturated=0,last=0,max_gap=0; float id_sum=0,iq_sum=0,ed2=0,eq2=0,vmax=0; };
struct Row { unsigned us,m; float id,iq,td,tq,speed,angle,vd,vq,bus,limit; };
static Row rows[1600];
static unsigned count=0,seen[2]={},start=0,duration=0,kind=0,mask=0;
static unsigned active[2]={},valid[2]={};
static int margin[2]={8000,8000};
static Stats stats[2];
static bool logging=false,aligning=false;
static float value=0,speed_integral[2]={};
void stop() {
    pwm::stop(); __atomic_store_n(&motor::ready,0U,__ATOMIC_RELEASE);
    if(aligning) { motor::calibration.state=motor::stage::IDLE; aligning=false; }
}
void status() {
    motor::foc_info s{}; motor::get_info(s);
    printf("STATUS {\"running\":%u,\"requested\":%u,\"ready\":%u,\"fault\":%u,\"fault_motor\":%u,\"fault_pc\":%lu,\"raw_a\":%u,\"raw_b\":%u,\"bus\":%.5f,\"stage\":%u,\"aligned\":[%u,%u],\"direction\":[%.0f,%.0f],\"zero\":[%.9f,%.9f],\"pi_updates\":[%u,%u]}\n",
        pwm::is_running(),pwm::is_requested(),s.ready,unsigned(s.fault.code),s.fault.motor_index,
        (unsigned long)pwm::fault_pc,s.fault.raw_a,s.fault.raw_b,s.bus_voltage_V,unsigned(motor::calibration.state),
        foc::motors[0].aligned,foc::motors[1].aligned,foc::motors[0].direction,foc::motors[1].direction,
        foc::motors[0].zero_rad,foc::motors[1].zero_rad,unsigned(foc::motors[0].updates),unsigned(foc::motors[1].updates));
}
void dump() {
    stop(); vTaskDelay(2); logging=false; vTaskDelay(2);status();
    for(unsigned m=0;m<2;++m) {
        auto &s=stats[m]; float n=s.n?float(s.n):1.0f;
        printf("METRIC {\"motor\":%u,\"n\":%u,\"id_mean\":%.7f,\"iq_mean\":%.7f,\"id_error_rms\":%.7f,\"iq_error_rms\":%.7f,\"voltage_max\":%.7f,\"saturated\":%u,\"max_gap_us\":%u,\"active\":%u,\"valid\":%u,\"min_margin_ticks\":%d}\n",
            m+1,s.n,s.id_sum/n,s.iq_sum/n,sqrtf(s.ed2/n),sqrtf(s.eq2/n),s.vmax,s.saturated,unsigned(foc::motors[m].max_dt_us),active[m],valid[m],margin[m]);
    }
    printf("META {\"kind\":%u,\"value\":%.5f,\"duration_ms\":%u,\"mask\":%u,\"cap\":%.5f,\"ratio\":%.5f,\"rows\":%u,\"metric_snapshot_hz\":1000,\"csv_hz\":200}\n",kind,value,duration/1000,mask,foc::voltage_limit,foc::bus_limit_ratio,count);
    puts("CSV us,motor,id,iq,target_d,target_q,speed,angle,vd,vq,bus,limit");
    for(unsigned i=0;i<count;++i) {auto &r=rows[i];
        printf("R %u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.5f,%.6f\n",r.us,r.m,r.id,r.iq,r.td,r.tq,r.speed,r.angle,r.vd,r.vq,r.bus,r.limit);
        if(i%10==0)vTaskDelay(1);
    }
    for(unsigned m=0;m<2;++m) printf("TIMING {\"motor\":%u,\"compute_max_us\":%u,\"command_age_max_us\":%u}\n",m+1,unsigned(foc::motors[m].max_compute_us),unsigned(foc::motors[m].max_command_age_us));
    puts("END");
}
void command(char *line) {
    if(!strcmp(line,"STOP")) {stop();status();return;}
    if(!strcmp(line,"STATUS")) {status();return;}
    if(!strcmp(line,"WIDE")) {
        if(pwm::is_requested()) {puts("ERROR running");return;}
        puts("ERROR unsupported");return;
    }
    if(!strcmp(line,"DUMP")) {if(!logging)dump();else puts("ERROR running");return;}
    if(!strcmp(line,"ARM")) {
        if(pwm::is_requested()||pwm::get_fault_code()!=motor::fault_code::NONE) {puts("ERROR state");return;}
        motor::calibration=motor::session{};
        motor::calibration.boot_us=pwm::now32();motor::calibration.began_us=pwm::now32();
        pwm::bench_deadline_us=pwm::now32()+8000000;
        aligning=true;motor::calibration.state=motor::stage::WAITING;puts("ARM_OK");return;
    }
    char cmd[16]{};unsigned m=0,ms=0;float x=0,cap=0,ratio=0;
    if(sscanf(line,"%15s %u %f %u %f %f",cmd,&m,&x,&ms,&cap,&ratio)!=6 ||
       (strcmp(cmd,"DTEST")&&strcmp(cmd,"SPEED")) ||m<1||m>3||ms<500||ms>6000||
       !isfinite(x)||!isfinite(cap)||!isfinite(ratio)||cap<=0||cap>2.6f||ratio<=0||ratio>.08f||
       (strcmp(cmd,"SPEED")==0?fabsf(x)>100.0f:fabsf(x)>.7f)||pwm::is_requested()||
       pwm::get_fault_code()!=motor::fault_code::NONE||!foc::motors[0].aligned||!foc::motors[1].aligned) {puts("ERROR parameters/state");return;}
    logging=false;count=0;mask=m;value=x;duration=ms*1000;kind=!strcmp(cmd,"SPEED")?2:1;
    for(unsigned i=0;i<2;++i) {stats[i]=Stats{};active[i]=valid[i]=seen[i]=0;margin[i]=8000;speed_integral[i]=0;}
    foc::reset();foc::voltage_limit=cap;foc::bus_limit_ratio=ratio;
    motor::speed_trip_rad_s=std::max(40.0f,std::min(120.0f,fabsf(x)*1.2f+10.0f));
    portENTER_CRITICAL(&motor::info_lock);motor::latest.target[0]={};motor::latest.target[1]={};portEXIT_CRITICAL(&motor::info_lock);
    start=pwm::now32();motor::calibration.began_us=start;pwm::bench_deadline_us=start+duration;
    logging=true;if(!pwm::start(mask)) {logging=false;puts("ERROR start");return;}puts("ACCEPTED");
}
}
static void observe_sample(unsigned m,const pwm::sample &s,bool enabled) {
    if(!bench::logging||!enabled)return;
    ++bench::active[m];
    if((s.flags&63U)==63U) {
        ++bench::valid[m];int v=int(s.a_start)-int(s.duty_a)-160;
        v=std::min(v,int(pwm::PERIOD-s.duty_a)-int(s.a_end)-160);
        v=std::min(v,int(s.b_start)-int(s.duty_b)-160);
        v=std::min(v,int(pwm::PERIOD-s.duty_b)-int(s.b_end)-160);
        bench::margin[m]=std::min(bench::margin[m],v);
    }
}
static void observe_control(unsigned m,const motor::current_info &s,const motor::target_current &target) {
    using namespace bench;if(!logging||!(mask&(1U<<m)))return;
    unsigned t=unsigned(s.timestamp_us)-start;float elapsed=t*1e-6f;
    if(t>=duration || unsigned(s.timestamp_us)==stats[m].last)return;
    float remain=(duration-t)*1e-6f;
    float ramp=elapsed<.1f?0.0f:(elapsed<.75f?(elapsed-.1f)/.65f:1.0f);
    if(remain<.65f)ramp=fmaxf(remain/.65f,0.0f);
    motor::target_current next{};
    if(kind==1) {next.id_A=(t>200000&&t+200000<duration)?value:0;}
    else {
        float e=value*ramp-foc::motors[m].direction*s.mechanical_speed_rad_s;
        float dt=stats[m].last?(unsigned(s.timestamp_us)-stats[m].last)*1e-6f:.0002f;
        float candidate=speed_integral[m]+.4f*e*dt;
        next.iq_A=fmaxf(-.7f,fminf(.7f,.2f*e+candidate));
        if(fabsf(.2f*e+candidate)<=.7f||(.2f*e+candidate)*e<0)speed_integral[m]=candidate;
    }
    motor::set_target_current(static_cast<motor::motor_id>(m),next);
    auto &a=stats[m];if(a.last)a.max_gap=std::max(a.max_gap,unsigned(s.timestamp_us)-a.last);a.last=unsigned(s.timestamp_us);
    float v=sqrtf(s.vd_V*s.vd_V+s.vq_V*s.vq_V);a.vmax=fmaxf(a.vmax,v);
    if(t>=1000000&&t+(kind==2?650000U:500000U)<duration) {
        ++a.n;a.id_sum+=s.id_A;a.iq_sum+=s.iq_A;
        a.ed2+=(target.id_A-s.id_A)*(target.id_A-s.id_A);a.eq2+=(target.iq_A-s.iq_A)*(target.iq_A-s.iq_A);
        if(v>=s.voltage_limit_V*.999f)++a.saturated;
    }
    if(++seen[m]%5==0&&count<1600)rows[count++]={t,m+1,s.id_A,s.iq_A,target.id_A,target.iq_A,
        s.mechanical_speed_rad_s,s.mechanical_angle_rad,s.vd_V,s.vq_V,adc::voltage(pwm::bus_raw),s.voltage_limit_V};
}
void foc_bench_init() {
    extern void math_benchmark(); math_benchmark();
    setvbuf(stdout,nullptr,_IONBF,0);if(!motor::init()){puts("INIT_FAILED");return;}
    fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);puts("BENCH_READY default=stopped ARM then finite DTEST/SPEED; STOP anytime");
    char line[160]{};unsigned n=0;
    while(true) {
        char c;while(read(STDIN_FILENO,&c,1)==1) {
            if(c=='\r')continue;
            if(c=='\n'){line[n]=0;bench::command(line);n=0;}else if(n<sizeof(line)-1)line[n++]=c;
        }
        if(bench::aligning&&motor::calibration.state==motor::stage::RUN&&foc::motors[0].aligned&&foc::motors[1].aligned) {
            bench::aligning=false;bench::stop();puts("ALIGNED");bench::status();
        }
        if(bench::logging && pwm::is_running()) {
            motor::foc_info state{};motor::get_info(state);
            for(unsigned m=0;m<2;++m)if(state.current[m].valid)observe_control(m,state.current[m],state.target[m]);
        }
        if(bench::logging&&(!pwm::is_requested()||pwm::get_fault_code()!=motor::fault_code::NONE)) {
            bench::stop();vTaskDelay(2);bench::logging=false;puts("DONE");bench::status();
        }
        if(bench::aligning&&(pwm::get_fault_code()!=motor::fault_code::NONE||static_cast<int32_t>(pwm::now32()-pwm::bench_deadline_us)>=0)) {
            bench::stop();puts("ALIGN_FAILED");bench::status();
        }
        vTaskDelay(1);
    }
}

#include "math_benchmark.inc"
