"""Extract and compile the actual firmware modulation; validate line voltage and aperture bounds."""
from pathlib import Path
import subprocess
here=Path(__file__).resolve().parent
source=Path(__file__).resolve().parents[2]/'esp32/balance_car/user_lib/hw/motor.cpp'
if not source.exists(): source=here.parents[1]/'user_lib/hw/motor.cpp'
s=source.read_text();start=s.index('pwm::command modulation(');opening=s.index('{',start);depth=1;end=opening+1
while depth:
 depth+=(s[end]=='{')-(s[end]=='}');end+=1
cpp='''#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
using namespace std;
namespace pwm {struct command {uint32_t duty_a=2000,duty_b=2000,duty_c=2000;};}
bool low_side_clamp=false;
'''+s[start:end]+r'''
int main(){
 unsigned count=0;double worst=0; unsigned min_d=4000,max_d=0;
 float ratios[]={0,.005f,.015f,.04f,.08f,.12f,.18f,.24f};
 for(int mode=0;mode<2;++mode){low_side_clamp=mode;
 for(int b=50;b<=130;++b)for(int a=0;a<720;++a)for(int q=0;q<4;++q)for(float r:ratios){
  if(!mode&&r>.04f)continue;
  float bus=b*.1f,theta=a*6.28318530718f/720,phi=q*1.57079632679f;
  float vd=bus*r*cosf(phi),vq=bus*r*sinf(phi);
  auto c=modulation(vd,vq,theta,bus);
  unsigned d[]={c.duty_a,c.duty_b,c.duty_c};
  float alpha=vd*cosf(theta)-vq*sinf(theta),beta=vd*sinf(theta)+vq*cosf(theta);
  float p[]={alpha,-.5f*alpha+.8660254f*beta,-.5f*alpha-.8660254f*beta};
  for(int i=0;i<3;++i){
   if(d[i]<320||d[i]>3680||(mode&&d[i]>2000)){printf("BOUND FAIL\n");return 1;}
   if(mode){min_d=min(min_d,d[i]);max_d=max(max_d,d[i]);}
   for(int j=0;j<3;++j){double err=fabs((float(d[i])-float(d[j]))*bus/4000-(p[i]-p[j]));worst=max(worst,err);
    if(err>bus/4000+1e-5){printf("VOLTAGE FAIL %g\n",err);return 2;}}
  }++count;
 }}
 printf("{\"cases\":%u,\"line_voltage_error_max_V\":%.9f,\"wide_compare_min\":%u,\"wide_compare_max\":%u,\"original_neutral_compare\":2000,\"result\":\"PASS_GEOMETRY_ONLY\"}\n",count,worst,min_d,max_d);
}
'''
(here/'modulation_check.cpp').write_text(cpp)
subprocess.run(['c++','-O2','-std=c++17',str(here/'modulation_check.cpp'),'-o',str(here/'modulation_check')],check=True)
result=subprocess.check_output([str(here/'modulation_check')],text=True)
(here/'modulation_check.json').write_text(result);print(result)
