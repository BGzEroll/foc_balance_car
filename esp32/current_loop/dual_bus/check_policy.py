"""Extract the actual firmware modulation function for a host C++ invariant test."""
from pathlib import Path
r=Path(__file__).resolve().parent
s=(r/'dual_foc_current.cpp').read_text(encoding='utf-8')
fn=s[s.index('Command modulation('):s.index('void control(')]
test=r'''
#include <cmath>
#include <cstdio>
#include <cassert>
struct Command {unsigned tag,seq,sample_us,ca,cb,cc;};
'''+fn+r'''
int main(){
 unsigned tested=0;
 for(int bi=500;bi<=1300;bi+=5){float bus=bi*.01f;
  for(float cap:{.12f,.2f,.4f})for(float ratio:{.015f,.025f,.04f}){
   float limit=fminf(cap,ratio*bus);assert(limit>0&&limit<=cap&&limit<=ratio*bus+1e-6f);
   for(int ti=0;ti<3600;++ti){float t=ti*6.28318530718f/3600;
    auto c=modulation(0,limit,t,bus);unsigned v[3]={c.ca,c.cb,c.cc};
    for(auto x:v){assert(x>=320&&x<=3680);assert(x+160<=2400);}
    float a=-limit*sinf(t),b=limit*cosf(t),vb=-.5f*a+.8660254f*b;
    float realized=(float(c.ca)-float(c.cb))*bus/4000;
    assert(fabsf(realized-(a-vb))<=bus/4000+2e-6f);++tested;
   }
  }
 }
 std::printf("PASS %u angle/bus/limit cases: line voltage quantization and nominal low-side entry margin\n",tested);
}
'''
test=test.replace('#include <cassert>','#include <cassert>\n#include <initializer_list>')
(r/'policy_test.cpp').write_text(test,newline='\n')
