function out=measured_car_pid(raw,cfg,moves)
%#codegen
% PID only. Outputs actual-actuator placeholder, observed states, references,
% world disturbances, fall flag, left/right Iq commands, velocity/pitch targets.
raw=[raw(1);raw(4);raw(5);-raw(2);-raw(3);raw(6);raw(7);raw(8);raw(10);-raw(9);raw(11);raw(13);-raw(12);raw(14)];
persistent distance prevV velocityIntegral pitchIntegral yawIntegral
if isempty(distance),distance=0;prevV=0;velocityIntegral=0;pitchIntegral=0;yawIntegral=0;end
t=raw(1);qw=raw(8);qx=raw(9);qy=raw(10);qz=raw(11);
R=[1-2*(qy^2+qz^2) 2*(qx*qy-qw*qz) 2*(qx*qz+qw*qy);2*(qx*qy+qw*qz) 1-2*(qx^2+qz^2) 2*(qy*qz-qw*qx);2*(qx*qz-qw*qy) 2*(qy*qz+qw*qx) 1-2*(qx^2+qy^2)];
pitch=atan2(-R(3,1),hypot(R(1,1),R(2,1)));yaw=atan2(R(2,1),R(1,1));roll=atan2(R(3,2),R(3,3));wb=R'*raw(12:14);
pitchRate=cos(roll)*wb(2)-sin(roll)*wb(3);yawRate=(sin(roll)*wb(2)+cos(roll)*wb(3))/max(.2,cos(pitch));
v=cos(yaw)*raw(3)+sin(yaw)*raw(5);Ts=cfg(31);
if t>0,distance=distance+Ts*(prevV+v)/2;end
prevV=v;state=[distance;v;pitch-cfg(3);pitchRate;yaw;yawRate];
[ref,F,T]=measured_car_reference(t,round(cfg(1)),moves);
targetV=clamp(ref(2)+cfg(22)*(ref(1)-distance),cfg(23));ve=targetV-v;
vi=clamp(velocityIntegral+Ts*ve,cfg(25));targetUnclamped=cfg(20)*ve+cfg(21)*vi;targetPitch=clamp(targetUnclamped,cfg(24));
pe=state(3)-targetPitch;piCandidate=clamp(pitchIntegral+Ts*pe,cfg(26));ye=atan2(sin(ref(5)-yaw),cos(ref(5)-yaw));yi=clamp(yawIntegral+Ts*ye,cfg(30));
effort=[cfg(17)*pe+cfg(18)*piCandidate+cfg(19)*pitchRate;cfg(27)*ye+cfg(28)*yi+cfg(29)*(ref(6)-yawRate)];
current=[(effort(1)-effort(2))/2;(effort(1)+effort(2))/2];
if max(abs(current))<=cfg(2)
 pitchIntegral=piCandidate;yawIntegral=yi;
 if abs(targetUnclamped)<=cfg(24),velocityIntegral=vi;end
end
command=max(-cfg(2),min(cfg(2),current));
out=[command*cfg(37);state;ref;F;T;double(abs(pitch)>pi/4||abs(roll)>pi/6);command;targetV;targetPitch];
end
function y=clamp(x,limit)
y=max(-limit,min(limit,x));
end
