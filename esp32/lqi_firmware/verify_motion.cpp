#include "motion_test.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){
 motion_test::state s;s.advance(10,1);assert(!s.armed && !s.hard_stop);
 s.start(12345678900ULL,.003f);float previous=s.position;float peak_speed=0,peak_acc=0,previous_speed=0;
 for(unsigned ms=0;ms<=60000;++ms){
  s.advance(s.began_us+ms*1000ULL,s.position);
  assert(!s.aborted && !s.hard_stop && std::fabs(s.position-s.origin)<=.050001f);
  peak_speed=std::fmax(peak_speed,std::fabs(s.speed));
  if(ms>0){assert(std::fabs((s.position-previous)/.001f-s.speed)<.00005f);peak_acc=std::fmax(peak_acc,std::fabs(s.speed-previous_speed)/.001f);}
  previous=s.position;previous_speed=s.speed;
 }
 assert(s.phase==4 && s.speed==0 && std::fabs(s.position-s.origin)<1e-7f);
 assert(peak_speed<=.015626f && peak_acc<.0081f);
 s.advance(s.began_us+61000000ULL,s.origin+.101f);assert(s.aborted && !s.hard_stop && s.position==s.origin && s.speed==0);
 s.advance(s.began_us+62000000ULL,s.origin+.151f);assert(s.hard_stop);
 std::printf("MOTION_PASS checks=60001 peak_speed=%.8f peak_acceleration=%.8f soft_and_hard_guards=PASS\n",peak_speed,peak_acc);
}
