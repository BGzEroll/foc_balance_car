function out=measured_pid_observation(imu,actuator,now,sensorCfg)
%#codegen
% Fast encoder observation + held 200Hz attitude/gyro. No new IMU/noise step.
out=imu;out(1)=now;
if sensorCfg(1)<.5,return;end % Preserve the existing ideal-observation mode.
q=imu(8:11);w=q(1);x=q(2);y=q(3);z=q(4);
R=[1-2*(y*y+z*z) 2*(x*y-w*z) 2*(x*z+w*y);2*(x*y+w*z) 1-2*(x*x+z*z) 2*(y*z-w*x);2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x*x+y*y)];
gyro=R'*imu(12:14);yaw=atan2(R(2,1),R(1,1));
forward=sensorCfg(9)*(.5*(actuator(15)+actuator(29))-gyro(1));
out(3)=-sin(yaw)*forward;out(5)=cos(yaw)*forward;
end
