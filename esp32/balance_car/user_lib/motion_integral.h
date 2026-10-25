#pragma once
#include <cmath>
namespace motion_integral {
inline float clamp(float x){return std::fmax(-.3f,std::fmin(.3f,x));}
struct state {
 float anchor=0;
 float begin(float current){
  anchor=clamp(current);return anchor;
 }
 float update(float current,double seconds,float limit) const {
  if(limit<=0 || seconds>=48)return current;
  current=std::fmax(anchor-limit,std::fmin(anchor+limit,current));
  if((seconds>=12&&seconds<13)||(seconds>=36&&seconds<37))
   current=anchor+(current-anchor)*.9834714538f;
  return clamp(current);
 }
 float end(float current)const{return clamp(current);}
};
}
