#pragma once
#include <cmath>
#include "pid_config.h"
namespace motion_integral {
inline float clamp(float x){return std::fmax(-pid_config::VELOCITY_INTEGRAL_LIMIT,std::fmin(pid_config::VELOCITY_INTEGRAL_LIMIT,x));}
struct state {
 float anchor=0;
 float begin(float current){
  anchor=clamp(current);return anchor;
 }
 float update(float current,double seconds,float limit,float dt_s) const {
  if(limit<=0 || seconds>=16)return current;
  current=std::fmax(anchor-limit,std::fmin(anchor+limit,current));
  if((seconds>=4&&seconds<5)||(seconds>=12&&seconds<13))
   current=anchor+(current-anchor)*std::exp(-dt_s/.3f);
  return clamp(current);
 }
 float end(float current)const{return clamp(current);}
};
}
