#include "balance.h"
#include <iostream>
#include <iomanip>
int main(){
 std::cout<<std::setprecision(10);
 float v[15];
 while(std::cin>>v[0]){
  for(unsigned i=1;i<15;++i)if(!(std::cin>>v[i]))return 2;
  balance::observation m{v[0],v[1],v[2],v[3],v[4]};
  balance::reference ref{v[5],v[6],v[7],v[8]};
  balance::state s{};s.distance_m=v[9];s.previous_speed_m_s=v[10];s.velocity_integral=v[11];s.yaw_integral=v[12];s.first_sample=v[13]!=0;
  balance::limits lim{};lim.current_A=v[14];auto o=balance::update(m,ref,lim,s);
  std::cout<<o.current_A[0]<<' '<<o.current_A[1]<<' '<<s.distance_m<<' '<<s.previous_speed_m_s<<' '<<s.velocity_integral<<' '<<s.yaw_integral<<' '<<s.pitch_integral<<' '<<o.saturated<<'\n';
 }
}
