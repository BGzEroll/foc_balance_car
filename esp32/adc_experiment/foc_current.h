#pragma once
#include <stdint.h>
namespace foc_current {
constexpr uint32_t CPU_MHZ=160, TASK_PRIORITY=0;
struct timing{uint64_t sum=0;uint32_t maximum=0,minimum=UINT32_MAX;};
struct state{
 uint32_t count=0,events=0,adc_invalid=0,compare_errors=0,gaps=0,late_half=0,idle=0;
 uint32_t samples[5]={},fresh[2]={},reused[2]={},skipped[2]={};
 uint32_t interval_min=UINT32_MAX,interval_max=0;
 uint16_t raw[5]={},raw_min[5]={},raw_max[5]={};uint64_t raw_sum[5]={},raw_square_sum[5]={};
 float bus_V=0,current_A[4]={},id_A[2]={},iq_A[2]={},duty[6]={};
 uint32_t rejected[2]={};
 uint32_t late_atomic=0,phase_disagree=0,phase_hist[50]={};
 timing atomic_phase,completion;
 timing acquisition,calculation,write,total,dt,phase,skew,bus,age[2];
};
bool init();void get_package(state&);
}
