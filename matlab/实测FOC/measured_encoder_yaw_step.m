function [state,diag]=measured_encoder_yaw_step(state,gyroYaw,encoder,dt,mode)
%#codegen
% Explicit fixed-size state for replay and native MATLAB Function use.
% state=[yaw bias P fastRate quietTime confirmed count index; historyL(20);
% historyR(20)] (48x1). encoder=[omegaL omegaR angleL angleR].
% Modes: 1 A always integrate + quiet bias LP; 2 B hold; 3 C hold + bias KF.
p=encoder_yaw_parameters;
assert(abs(dt-.005)<1e-9,'Encoder yaw observer requires 200Hz IMU');
n=20;slot=round(state(8))+1;if slot>n,slot=1;end
count=min(n,round(state(7))+1);
oldL=state(8+slot);oldR=state(28+slot);
state(8+slot)=encoder(3);state(28+slot)=encoder(4);
fast=p.radius_m/p.track_m*(encoder(2)-encoder(1));
state(4)=state(4)+dt/(p.fast_tau_s+dt)*(fast-state(4));
wheelRate=state(4);
valid=count==n && state(7)>=n;
if valid
 wheelRate=p.radius_m/p.track_m*((encoder(4)-oldR)-(encoder(3)-oldL))/p.window_s;
end
quiet=valid && abs(wheelRate)<p.enter_rad_s && abs(state(4))<p.fast_exit_rad_s;
moving=~valid || abs(wheelRate)>p.exit_rad_s || abs(state(4))>p.fast_exit_rad_s;
if moving,state(5)=0;state(6)=0;
elseif quiet,state(5)=state(5)+dt;if state(5)>=p.confirm_s,state(6)=1;end
else,state(5)=0;end % Hysteresis retains confirmed state until exit threshold.
noTurn=state(6)>.5;
gain=0;
if mode==1
 if noTurn,state(2)=state(2)+dt/(p.bias_tau_s+dt)*(gyroYaw-state(2));end
elseif mode==3
 state(3)=state(3)+p.bias_walk_rad_s_sqrt_s^2*dt;
 if noTurn
  % z_omega=0, predicted omega=gyroYaw-bias. H_bias=-1.
  % Equivalent direct bias measurement z_bias=gyroYaw, H=1.
  variance=p.zero_rate_std_rad_s^2;
  gain=state(3)/(state(3)+variance);innovation=gyroYaw-state(2);
  state(2)=state(2)+gain*innovation;
  state(3)=(1-gain)^2*state(3)+gain^2*variance; % Joseph scalar form.
 end
end
rate=gyroYaw-state(2);delta=rate*dt;
if mode>=2 && noTurn,delta=0;end
state(1)=state(1)+delta;state(7)=count;state(8)=slot;
diag=[state(1);gyroYaw;rate;state(2);wheelRate;state(4);double(noTurn);state(3);state(5);delta;gain;double(valid)];
end
