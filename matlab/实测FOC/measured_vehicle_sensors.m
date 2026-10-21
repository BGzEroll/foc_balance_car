function [sensed,yawDiagnostic]=measured_vehicle_sensors(raw,actuator,cfg)
%#codegen
% Reproduce MPU scaling, approximate hardware bandwidth and original complementary filter.
% raw world frame: X right, Y forward, Z up. Encoder observations come from 5kHz actuator.
persistent prevV accLP gyroLP accNoise gyroNoise angles seed historyAcc historyGyro qEstimate ekfBias ekfP yawState
sensed=raw;
yawDiagnostic=zeros(12,1);
q=raw(8:11);R=rotation(q);dt=cfg(7);v=[raw(3);raw(5);raw(7)];
if isempty(prevV)
 prevV=v;accLP=R'*[0;0;1];gyroLP=R'*raw(12:14);accNoise=zeros(3,1);gyroNoise=zeros(3,1);angles=zeros(3,1);seed=9259;
 angles(1)=atan2(accLP(2),accLP(3));angles(2)=atan2(-accLP(1),hypot(accLP(2),accLP(3)));angles(3)=atan2(-R(1,2),R(2,2));
 qEstimate=q/norm(q); % Same initial attitude reference as the legacy estimator.
 ep=measured_ekf_parameters;
 ekfBias=zeros(3,1);ekfP=diag([ones(1,3)*ep.initial_attitude_std_rad^2 ones(1,3)*ep.initial_bias_std_rad_s^2]);
 yp=encoder_yaw_parameters;yawState=zeros(48,1);yawState(1)=angles(3);yawState(3)=yp.bias_initial_std_rad_s^2;
 historyAcc=zeros(12,3,3);historyGyro=zeros(12,3,3);
end
if cfg(1)<.5,prevV=v;return;end
accel=(v-prevV)/dt;prevV=v;acc=R'*([0;0;9.80665]+accel)/9.80665;gyro=R'*raw(12:14);
a=dt/(1/(2*pi*cfg(8))+dt);accLP=accLP+a*(acc-accLP);gyroLP=gyroLP+a*(gyro-gyroLP);
z=zeros(6,1);
for k=1:3
 seed=mod(1664525*seed+1013904223,4294967296);u=(seed+1)/4294967297;
 seed=mod(1664525*seed+1013904223,4294967296);vnoise=(seed+1)/4294967297;r=sqrt(-2*log(u));z(2*k-1)=r*cos(2*pi*vnoise);z(2*k)=r*sin(2*pi*vnoise);
end
speed=.5*(abs(actuator(15))+abs(actuator(29)));u=min(speed/10,2);
bankA=zeros(3,3);bankG=zeros(3,3);
for j=1:3
 base=73+45*(j-1);A=reshape(cfg(base:base+35),12,3);L=reshape(cfg(base+36:base+44),3,3);
 bankA(:,j)=-sum(A.*historyAcc(:,:,j),1)'+L*z(1:3);historyAcc(2:12,:,j)=historyAcc(1:11,:,j);historyAcc(1,:,j)=bankA(:,j)';
 base=73+45*(j+2);A=reshape(cfg(base:base+35),12,3);L=reshape(cfg(base+36:base+44),3,3);
 bankG(:,j)=-sum(A.*historyGyro(:,:,j),1)'+L*z(4:6);historyGyro(2:12,:,j)=historyGyro(1:11,:,j);historyGyro(1,:,j)=bankG(:,j)';
end
weights=[max(1-u,0);1-max(1-u,0)-max(u-1,0);max(u-1,0)];accNoise=bankA*weights;gyroNoise=bankG*weights;
acc=round(max(-2,min(2,cfg(72)*accLP+accNoise))*16384)/16384;
gyro=gyroLP+[cfg(4);cfg(5);cfg(6)]+gyroNoise;
% Explicit simulation stress profile, never inserted into measured source data.
if numel(cfg)>343 && cfg(344)>.5,gyro(3)=gyro(3)+(.25+.015*raw(1))*pi/180;end
gyro=round(max(-500*pi/180,min(500*pi/180,gyro))*65.5*180/pi)/(65.5*180/pi);
ap=atan2(acc(2),acc(3));ar=atan2(-acc(1),hypot(acc(2),acc(3)));
angles(1)=.98*(angles(1)+gyro(1)*dt)+.02*ap;angles(2)=.98*(angles(2)+gyro(2)*dt)+.02*ar;angles(3)=angles(3)+gyro(3)*dt;
if numel(cfg)>342 && cfg(343)>2.5
 % Heading derivative is not simply body gyro Z when the body is tilted.
 gyroYaw=(sin(angles(1))*gyro(2)+cos(angles(1))*gyro(3))/max(.2,cos(angles(2)));
 [yawState,yawDiagnostic]=measured_encoder_yaw_step(yawState,gyroYaw,actuator([15 29 16 30]),dt,round(cfg(343))-2);
 angles(3)=yawState(1);
 % All A/B/C keep the original measured angular-rate feedback for PID D.
 % This comparison changes yaw integration/bias handling only.
end
cx=cos(angles(1)/2);sx=sin(angles(1)/2);cy=cos(angles(2)/2);sy=sin(angles(2)/2);cz=cos(angles(3)/2);sz=sin(angles(3)/2);
quat=[cx*cy*cz+sx*sy*sz;sx*cy*cz-cx*sy*sz;cx*sy*cz+sx*cy*sz;cx*cy*sz-sx*sy*cz];
% Optional six-axis quaternion complementary filter. Legacy remains default.
if numel(cfg)>342 && cfg(343)>.5 && cfg(343)<2.5
 if cfg(343)>1.5
  [qEstimate,ekfBias,ekfP]=measured_ekf_step(qEstimate,ekfBias,ekfP,gyro,acc,dt,cfg(72));
  gyro=gyro-ekfBias; % Bias-corrected rate is part of the EKF observation.
 else
  qEstimate=measured_quaternion_step(qEstimate,gyro,acc,dt,4,cfg(72));
 end
 quat=qEstimate;Rq=rotation(quat);angles(3)=atan2(Rq(2,1),Rq(1,1));
end
sensed(8:11)=quat;Rs=rotation(quat);sensed(12:14)=Rs*gyro;
forward=cfg(9)*(.5*(actuator(15)+actuator(29))-gyro(1));
sensed(3)=-sin(angles(3))*forward;sensed(5)=cos(angles(3))*forward;
end
function R=rotation(q)
w=q(1);x=q(2);y=q(3);z=q(4);R=[1-2*(y*y+z*z) 2*(x*y-w*z) 2*(x*z+w*y);2*(x*y+w*z) 1-2*(x*x+z*z) 2*(y*z-w*x);2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x*x+y*y)];
end
