"""Host accuracy tests use the actual header; host timing is deliberately omitted."""
from pathlib import Path
import subprocess
base=next(q for p in Path(__file__).resolve().parents for q in [p,p/'esp32/balance_car'] if (q/'user_lib/hw/foc_math.h').exists())
out=base/'build/perf_evidence'
out.mkdir(parents=True,exist_ok=True)
header=(base/'user_lib/hw/foc_math.h').read_text().replace('#include "esp_attr.h"','#define DRAM_ATTR')
(out/'foc_math_host.h').write_text(header)
cpp=r'''
#include "foc_math_host.h"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
static uint32_t seed=123456789;
float random01(){seed=seed*1664525u+1013904223u;return (seed>>8)*(1.0f/16777216);}
void duties(float vd,float vq,float cs,float sn,float bus,int out[3]){
 float alpha=vd*cs-vq*sn,beta=vd*sn+vq*cs;
 float v[3]={alpha,-.5f*alpha+.8660254f*beta,-.5f*alpha-.8660254f*beta};
 float high=fmaxf(v[0],fmaxf(v[1],v[2])),low=fminf(v[0],fminf(v[1],v[2])),zero=-.5f*(high+low);
 for(int i=0;i<3;++i)out[i]=std::clamp(int((.5f+(v[i]+zero)/bus)*4000+.5f),320,3680);
}
int main(){
 foc_math::init(); double lut_error=0,norm_error=0,vector_error=0,square_error=0;int duty_error=0;
 const unsigned n=2000000;
 for(unsigned i=0;i<n;++i){
  float a=random01()*foc_math::TWO_PI,s,c;foc_math::sincos(a,s,c);
  lut_error=std::max(lut_error,std::max(fabs(double(s)-sin(double(a))),fabs(double(c)-cos(double(a)))));
  float raw=-64+128*random01();float old=fmodf(raw,foc_math::TWO_PI);if(old<0)old+=foc_math::TWO_PI;
  float next=foc_math::wrap(raw);if(!(next>=0&&next<foc_math::TWO_PI))return 2;
  norm_error=std::max(norm_error,fabs(remainder(double(next)-old,double(foc_math::TWO_PI))));
  foc_math::sincos(next,s,c);float old_s=sinf(old),old_c=cosf(old);
  vector_error=std::max(vector_error,std::max(fabs(double(s)-old_s),fabs(double(c)-old_c)));
  float bus=5+8*random01(),amp=.08f*bus*random01(),p=random01()*foc_math::TWO_PI;
  float vd=amp*cosf(p),vq=amp*sinf(p);int x[3],y[3];
  duties(vd,vq,old_c,old_s,bus,x);duties(vd,vq,c,s,bus,y);
  for(int k=0;k<3;++k)duty_error=std::max(duty_error,std::abs(x[k]-y[k]));
  float limit=.08f*bus,d=-3+6*random01(),q=-3+6*random01(),od=d,oq=q,nd=d,nq=q;
  float sq=d*d+q*q,mag=sqrtf(sq);if(mag>limit){float z=limit/mag;od*=z;oq*=z;}
  if(sq>limit*limit){float z=limit/sqrtf(sq);nd*=z;nq*=z;}
  square_error=std::max(square_error,std::max(fabs(double(od)-nd),fabs(double(oq)-nq)));
 }
 // Explicit negative boundaries, zero, complete turns and fallback range.
 for(int k=-1000;k<=1000;++k){float a=k*foc_math::TWO_PI;for(float x:{a,nextafterf(a,-INFINITY),nextafterf(a,INFINITY)}){
  float y=foc_math::wrap(x);if(!(y>=0&&y<foc_math::TWO_PI))return 3;
 }}
 printf("{\"result\":\"%s\",\"random_cases\":%u,\"lut_component_abs_error\":%.12g,\"wrap_circular_error_rad\":%.12g,\"combined_vs_old_component_error\":%.12g,\"max_pwm_compare_delta_ticks\":%d,\"conditional_sqrt_voltage_delta\":%.12g,\"lut_ram_bytes\":%zu,\"note\":\"Host numerical verification only; MCU cycles measured separately.\"}\n",
  lut_error<2e-6&&norm_error<1e-5&&duty_error<=1&&square_error<1e-6?"PASS_NUMERICAL":"FAIL",n,lut_error,norm_error,vector_error,duty_error,square_error,sizeof(foc_math::sine_table));
 return lut_error<2e-6&&norm_error<1e-5&&duty_error<=1&&square_error<1e-6?0:4;
}
'''
(out/'math_accuracy.cpp').write_text(cpp)
subprocess.run(['g++','-O2','-ffp-contract=off','-std=c++17',str(out/'math_accuracy.cpp'),'-o',str(out/'math_accuracy')],check=True)
result=subprocess.check_output([str(out/'math_accuracy')],text=True)
(out/'math_accuracy.json').write_text(result)
print(result)
