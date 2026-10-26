#pragma once
#include "yaw_observer.h"
#include <cstdint>
namespace yaw_test {
struct state {
 bool armed=false,aborted=false;
 uint64_t began_us=0;
 float origin=0,encoder_origin=0,position_origin=0,amplitude=0,move_seconds=3,target=0,rate=0,peak=0;
 unsigned phase=0;
 void start(uint64_t now,float angle,float encoder,float position,float excursion,float move_s){
  *this=state{};armed=true;began_us=now;origin=angle;encoder_origin=encoder;
  position_origin=position;amplitude=excursion;move_seconds=move_s;
 }
 void advance(uint64_t now,float angle,float encoder,float position){
  if(!armed)return;
  const float displacement=std::fmax(std::fabs(yaw_observer::wrap(angle-origin)),std::fabs(encoder-encoder_origin));
  peak=std::fmax(peak,displacement);
  if(displacement>=1.134464014f || std::fabs(position-position_origin)>.18f)aborted=true;
  const double t=(now-began_us)*1e-6;
  const double phase_seconds=move_seconds+2;
  if(aborted || t>=4*phase_seconds){target=rate=0;phase=aborted?5:4;return;}
  phase=static_cast<unsigned>(t/phase_seconds);
  const float from[4]={0,amplitude,0,-amplitude},to[4]={amplitude,0,-amplitude,0};
  const double u=std::fmin(1.0,(t-phase_seconds*phase)/move_seconds),h=u*u*u*(10+u*(-15+6*u));
  target=from[phase]+(to[phase]-from[phase])*h;
  rate=(to[phase]-from[phase])*30*u*u*(1-u)*(1-u)/move_seconds;
 }
};
}
