function out=balance_suite_controller(raw,cfg)
%#codegen
% raw=[time x vx y vy z vz qw qx qy qz wx wy wz]，角速度在世界坐标。
% out=[实际tau(2) state(6) ref(6) force(3) torque(3) fall 指令tau(2) PID目标(2)]。
persistent distance prevV velocityIntegral pitchIntegral yawIntegral
if isempty(distance)
 distance=0;prevV=0;velocityIntegral=0;pitchIntegral=0;yawIntegral=0;
end
t=raw(1); qw=raw(8);qx=raw(9);qy=raw(10);qz=raw(11);
R=[1-2*(qy^2+qz^2) 2*(qx*qy-qw*qz) 2*(qx*qz+qw*qy); ...
   2*(qx*qy+qw*qz) 1-2*(qx^2+qz^2) 2*(qy*qz-qw*qx); ...
   2*(qx*qz-qw*qy) 2*(qy*qz+qw*qx) 1-2*(qx^2+qy^2)];
pitch=atan2(-R(3,1),hypot(R(1,1),R(2,1)));yaw=atan2(R(2,1),R(1,1));
roll=atan2(R(3,2),R(3,3)); wb=R'*raw(12:14);
pitchRate=cos(roll)*wb(2)-sin(roll)*wb(3);
yawRate=(sin(roll)*wb(2)+cos(roll)*wb(3))/cos(pitch);
v=cos(yaw)*raw(3)+sin(yaw)*raw(5);
Ts=cfg(31);
if t>0,distance=distance+Ts*(prevV+v)/2;end
prevV=v;
state=[distance;v;pitch-cfg(3);pitchRate;yaw;yawRate];
ref=zeros(6,1);F=zeros(3,1);T=zeros(3,1);scenario=round(cfg(1));
fast=round(cfg(34))==1;
if scenario==2 % 指令行驶：0.3m前进，随后后退到0.1m。
 if fast
  [p1,v1]=smoothmove(t,1,1.2,.30);[p2,v2]=smoothmove(t,3.8,1.2,-.20);
 else
  [p1,v1]=smoothmove(t,1,3,.30);[p2,v2]=smoothmove(t,6,2,-.20);
 end
 ref(1)=p1+p2;ref(2)=v1+v2;
elseif scenario==3 % 原地转60°，随后返回20°。
 if fast
  [a,w]=smoothmove(t,1,1.2,pi/3);[a2,w2]=smoothmove(t,4,1.2,-2*pi/9);
 else
  [a,w]=smoothmove(t,1,2,pi/3);[a2,w2]=smoothmove(t,5,2,-2*pi/9);
 end
 ref(5)=a+a2;ref(6)=w+w2;
elseif scenario==4 % 向前运动同时转弯。
 if fast
  [s,vs]=smoothmove(t,1,3,.6);[a,w]=smoothmove(t,1.5,1.5,pi/3);
 else
  [s,vs]=smoothmove(t,1,6,.6);[a,w]=smoothmove(t,2,3,pi/3);
 end
 ref(1)=s;ref(2)=vs;ref(5)=a;ref(6)=w;
elseif scenario==5 % 轮轴处的外力及绕竖直轴的外转矩脉冲。
 if t>=2 && t<2.15,F(1)=.4;end
 if t>=4 && t<4.15,T(3)=.002;end
elseif scenario==6 % 地形路线：经过缓坡、平台、下坡和单侧小凸起。
 [s,vs]=smoothmove(t,1,8,.75);ref(1)=s;ref(2)=vs;
elseif scenario==7 % 恒定扰动：比较两种控制器的稳态误差。
 if t>=2,F(1)=.08;T(3)=.0004;end
elseif scenario==8 % 明显坡道：上坡、平台、下坡、单侧凸起。
 [s,vs]=smoothmove(t,1,20,cfg(36));ref(1)=s;ref(2)=vs;
elseif scenario==9 % 斜面中段停12秒，随后继续通过坡顶与下坡。
 [s1,v1]=smoothmove(t,1,6,cfg(35));
 [s2,v2]=smoothmove(t,19,11,cfg(36)-cfg(35));
 ref(1)=s1+s2;ref(2)=v1+v2;
end
error=state-ref;
pidTarget=zeros(2,1);
if cfg(16)<.5
 K=reshape(cfg(4:15),2,6);effort=-K*error;
else
 targetV=clamp(ref(2)+cfg(22)*(ref(1)-state(1)),cfg(23));
 velocityError=targetV-v;
 vi=clamp(velocityIntegral+Ts*velocityError,cfg(25));
 targetUnclamped=cfg(20)*velocityError+cfg(21)*vi;
 targetPitch=clamp(targetUnclamped,cfg(24));
 pitchError=state(3)-targetPitch;
 piCandidate=clamp(pitchIntegral+Ts*pitchError,cfg(26));
 yawError=atan2(sin(ref(5)-yaw),cos(ref(5)-yaw));
 yi=clamp(yawIntegral+Ts*yawError,cfg(30));
 effort=[cfg(17)*pitchError+cfg(18)*piCandidate+cfg(19)*pitchRate; ...
  cfg(27)*yawError+cfg(28)*yi+cfg(29)*(ref(6)-yawRate)];
 % 转矩或倾角饱和时冻结相关积分。
 if max(abs([(effort(1)-effort(2))/2;(effort(1)+effort(2))/2]))<=cfg(2)
  pitchIntegral=piCandidate;yawIntegral=yi;
  if abs(targetUnclamped)<=cfg(24),velocityIntegral=vi;end
 end
 pidTarget=[targetV;targetPitch];
end
% effort(1)=左右转矩和，effort(2)=右减左。
tau=[(effort(1)-effort(2))/2;(effort(1)+effort(2))/2];
limit=cfg(2);
command=max(-limit,min(limit,tau));
tau=command.*[cfg(32);cfg(33)]; % 显式列向量，避免Simulink向量方向推断差异
out=[tau;state;ref;F;T;double(abs(pitch)>pi/4 || abs(roll)>pi/6);command;pidTarget];
end

function y=clamp(x,limit)
y=max(-limit,min(limit,x));
end

function [p,v]=smoothmove(t,start,duration,amplitude)
u=max(0,min(1,(t-start)/duration));
p=amplitude*(10*u^3-15*u^4+6*u^5);
if t<=start || t>=start+duration,v=0;
else,v=amplitude*(30*u^2-60*u^3+30*u^4)/duration;end
end
