
#include <cmath>
#include <cstdio>
#include <cassert>
#include <initializer_list>
struct Command {unsigned tag,seq,sample_us,ca,cb,cc;};
Command modulation(float vd,float vq,float theta,float bus){
 float cs=cosf(theta),sn=sinf(theta);float a=vd*cs-vq*sn,b=vd*sn+vq*cs;
 float v[3]={a,-.5f*a+.8660254f*b,-.5f*a-.8660254f*b};
 float z=-.5f*(fmaxf(v[0],fmaxf(v[1],v[2]))+fminf(v[0],fminf(v[1],v[2])));Command result{};unsigned out[3]{};
 for(unsigned i=0;i<3;++i){float f=(.5f+(v[i]+z)/bus)*4000+.5f;unsigned c=unsigned(f);out[i]=c<320?320:(c>3680?3680:c);}
 result.ca=out[0];result.cb=out[1];result.cc=out[2];return result;
}

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
