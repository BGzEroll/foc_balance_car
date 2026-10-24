#include "motion_integral.h"
#include <cassert>
#include <cstdio>
int main(){unsigned checks=0;
 for(int i=-100;i<=100;++i){
  float xi=i*.001f,origin=i*.0001f,kx=-17.673595f,ki=-7.05226f;
  motion_integral::state m;float next=m.begin(xi,origin,kx,ki);
  assert(std::fabs((-kx*origin-ki*xi)-(-ki*next))<1e-6f);
  assert(std::fabs(m.end(next)-xi)<1e-7f);
  assert(m.update(next+.1f,10,0)==next+.1f);
  assert(std::fabs(m.update(next+.1f,10,.04f)-next-.04f)<1e-7f);
  float excess=next+.04f;
  for(int j=0;j<200;++j)excess=m.update(excess,12+j*.005,.04f);
  assert(std::fabs(excess-next)<.0015f);
  assert(m.update(next+.1f,50,.04f)==next+.1f);
  ++checks;
 }
 std::printf("MOTION_INTEGRAL_PASS cases=%u bumpless_reference=PASS reversal_release=PASS normal_balance_unchanged=PASS\n",checks);
}
