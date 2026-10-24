#include "balance.h"
#include "lqi_config.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
int main(){unsigned checks=0;
 for(float force: {0.0f,.15f,.25f,.35f,.4f})for(int i=-100;i<=100;++i){
  balance::gains g;g.friction_ff_N=force;balance::state s;balance::limits cap;
  balance::observation m;m.pitch_rad=lqi_config::HARDWARE_TRIM_RAD;m.speed_m_s=.0005f*i;m.pitch_rate_rad_s=.01f*i;
  balance::reference r;r.speed_m_s=.0002f*i;r.position_m=.0005f*i;
  auto out=balance::update(m,r,cap,s,g);
  double common=2.75092903575523*force*std::tanh(r.speed_m_s/.003);
  double pitch=std::asin(.11901611068648812*common);
  double e[8]={-r.position_m,m.speed_m_s-r.speed_m_s,-pitch,m.pitch_rate_rad_s,0,0,0,0};
  double u=common;for(int j=0;j<8;++j)u-=lqi_config::K[0][j]*e[j];
  double expected=std::fmax(-6.,std::fmin(6.,u/2));
  assert(std::fabs(out.current_A[0]-expected)<2e-5 && std::fabs(out.current_A[1]-expected)<2e-5);
  assert(std::fabs(out.target_pitch_rad-lqi_config::HARDWARE_TRIM_RAD-pitch)<1e-6);
  if(out.saturated)assert(s.velocity_integral==0);
  ++checks;
 }
 balance::gains a,b;b.friction_ff_N=.4f;balance::state sa,sb;balance::observation m;m.pitch_rad=.1f;
 auto oa=balance::update(m,{}, {},sa,a),ob=balance::update(m,{}, {},sb,b);
 assert(oa.current_A[0]==ob.current_A[0] && oa.current_A[1]==ob.current_A[1]);
 std::printf("FRICTION_FF_PASS moving_cases=%u zero_speed_equivalence=PASS current_cap=6A\n",checks);
}
