#pragma once
#include <cmath>
namespace yaw_observer {
constexpr float WHEEL_FACTOR=.0325f/.1325364f;
inline float wrap(float a){return std::atan2(std::sin(a),std::cos(a));}
// Encoders qualify zero-net-rotation windows. Only the gyro mean updates bias.
struct bias_observer {
 bool initialized=false,candidate=true,learning=false;
 float yaw=0,raw_yaw=0,encoder_yaw=0,bias=0,gyro_rate=0,encoder_rate=0,corrected_rate=0;
 float left=0,right=0,base_left=0,base_right=0,base_yaw=0;
 float seconds=0,gyro_integral=0,travel=0,window_angle=0,minimum=0,maximum=0,quiet_time=0;
 void begin_window(){seconds=gyro_integral=travel=0;window_angle=minimum=maximum=encoder_yaw;candidate=true;}
 void rebase(float heading,float l,float r){
  initialized=true;yaw=raw_yaw=encoder_yaw=base_yaw=heading;
  left=base_left=l;right=base_right=r;learning=false;begin_window();
 }
 void update(float heading,float rate,float l,float r,float dt,bool allow_learning){
  if(!initialized){rebase(heading,l,r);return;}
  const float dl=l-left,dr=r-right;left=l;right=r;gyro_rate=rate;
  encoder_rate=WHEEL_FACTOR*(dr-dl)/dt;
  encoder_yaw=base_yaw+WHEEL_FACTOR*((r-base_right)-(l-base_left));
  seconds+=dt;gyro_integral+=rate*dt;travel+=.0325f*(std::fabs(dl)+std::fabs(dr))*.5f;
  minimum=std::fmin(minimum,encoder_yaw);maximum=std::fmax(maximum,encoder_yaw);
  candidate=candidate && allow_learning && std::fabs(rate-bias)<.02f;
  if(seconds>=1){
   const float mean=gyro_integral/seconds;
   learning=candidate && std::fabs(encoder_yaw-window_angle)<.00004712389f &&
            maximum-minimum<.000872664626f && travel<.005f && std::fabs(mean-bias)<.001745329252f;
   if(learning){bias+=(1-std::exp(-seconds/15))*(mean-bias);quiet_time+=seconds;}
   begin_window();
  }
  corrected_rate=rate-bias;raw_yaw+=dt*rate;yaw+=dt*corrected_rate;
 }
};
}
