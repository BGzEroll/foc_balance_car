function out=balance_suite_controller(raw,cfg)
%#codegen
% raw=[time x vx y vy z vz qw qx qy qz wx wy wz]，角速度在世界坐标。
% out=[tauL tauR state(6) reference(6) forceWorld(3) torqueWorld(3) fall]。
persistent distance prevV
if isempty(distance),distance=0;prevV=0;end
t=raw(1); qw=raw(8);qx=raw(9);qy=raw(10);qz=raw(11);
R=[1-2*(qy^2+qz^2) 2*(qx*qy-qw*qz) 2*(qx*qz+qw*qy); ...
   2*(qx*qy+qw*qz) 1-2*(qx^2+qz^2) 2*(qy*qz-qw*qx); ...
   2*(qx*qz-qw*qy) 2*(qy*qz+qw*qx) 1-2*(qx^2+qy^2)];
pitch=atan2(-R(3,1),hypot(R(1,1),R(2,1)));yaw=atan2(R(2,1),R(1,1));
roll=atan2(R(3,2),R(3,3)); wb=R'*raw(12:14);
pitchRate=cos(roll)*wb(2)-sin(roll)*wb(3);
yawRate=(sin(roll)*wb(2)+cos(roll)*wb(3))/cos(pitch);
v=cos(yaw)*raw(3)+sin(yaw)*raw(5);
if t>0,distance=distance+.005*(prevV+v)/2;end
prevV=v;
state=[distance;v;pitch-cfg(3);pitchRate;yaw;yawRate];
ref=zeros(6,1);F=zeros(3,1);T=zeros(3,1);scenario=round(cfg(1));
if scenario==2 % 指令行驶：0.3m前进，随后后退到0.1m。
 [p1,v1]=smoothmove(t,1,3,.30);[p2,v2]=smoothmove(t,6,2,-.20);
 ref(1)=p1+p2;ref(2)=v1+v2;
elseif scenario==3 % 原地转60°，随后返回20°。
 [a,w]=smoothmove(t,1,2,pi/3);[a2,w2]=smoothmove(t,5,2,-2*pi/9);
 ref(5)=a+a2;ref(6)=w+w2;
elseif scenario==4 % 向前运动同时转弯。
 [s,vs]=smoothmove(t,1,6,.6);[a,w]=smoothmove(t,2,3,pi/3);
 ref(1)=s;ref(2)=vs;ref(5)=a;ref(6)=w;
elseif scenario==5 % 轮轴处的外力及绕竖直轴的外转矩脉冲。
 if t>=2 && t<2.15,F(1)=.4;end
 if t>=4 && t<4.15,T(3)=.002;end
elseif scenario==6 % 地形路线：经过缓坡、平台、下坡和单侧小凸起。
 [s,vs]=smoothmove(t,1,8,.75);ref(1)=s;ref(2)=vs;
elseif scenario==7 % 恒定扰动：观察LQR稳态误差。
 if t>=2,F(1)=.08;T(3)=.0004;end
end
error=state-ref;
K=reshape(cfg(4:15),2,6);
effort=-K*error;
% effort(1)=左右转矩和，effort(2)=右减左。
tau=[(effort(1)-effort(2))/2;(effort(1)+effort(2))/2];
limit=cfg(2);
tau=max(-limit,min(limit,tau));
out=[tau;state;ref;F;T;double(abs(pitch)>pi/4 || abs(roll)>pi/6)];
end

function [p,v]=smoothmove(t,start,duration,amplitude)
u=max(0,min(1,(t-start)/duration));
p=amplitude*(10*u^3-15*u^4+6*u^5);
if t<=start || t>=start+duration,v=0;
else,v=amplitude*(30*u^2-60*u^3+30*u^4)/duration;end
end
