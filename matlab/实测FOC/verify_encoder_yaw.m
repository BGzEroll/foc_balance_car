function r=verify_encoder_yaw
dt=.005;p=encoder_yaw_parameters;
states=zeros(48,3);states(3,:)=p.bias_initial_std_rad_s^2;
for k=1:4000
 for m=1:3,[states(:,m),d]=measured_encoder_yaw_step(states(:,m),.005,zeros(4,1),dt,m);end
end
r.A_bias_error=abs(states(2,1)-.005);r.B_bias=states(2,2);r.C_bias_error=abs(states(2,3)-.005);
assert(r.A_bias_error<1e-6&&r.C_bias_error<1e-6&&r.B_bias==0,'Static bias behavior mismatch');
before=states(1,:);
for k=1:100
 for m=1:3,[states(:,m),d]=measured_encoder_yaw_step(states(:,m),.005,zeros(4,1),dt,m);end
end
assert(states(1,2)==before(2)&&states(1,3)==before(3),'B/C must hold exactly when quiet');
assert(states(1,1)~=before(1),'A must continue integrating');
turnRate=.1;w=turnRate*p.track_m/(2*p.radius_m);delay=zeros(1,3);releaseBias=zeros(1,3);
for k=1:100
 enc=[-w;w;-w*k*dt;w*k*dt];
 for m=1:3
  [states(:,m),d]=measured_encoder_yaw_step(states(:,m),turnRate+.005,enc,dt,m);
  if d(7)<.5&&delay(m)==0,delay(m)=k*dt;releaseBias(m)=states(2,m);end
 end
end
r.turn_release_delay_s=delay;assert(all(delay>0&delay<.15),'Turn release too slow');
assert(abs(states(2,3)-releaseBias(3))<1e-8,'C must not learn turning rate as bias once released');
assert(states(1,3)-before(3)>.035,'C failed to recover turn integration');
r.C_variance=states(3,3);assert(r.C_variance>0&&isfinite(r.C_variance));
% Equal wheel motion must qualify as straight even when translating.
s=zeros(48,1);s(3)=p.bias_initial_std_rad_s^2;
for k=1:200,[s,d]=measured_encoder_yaw_step(s,0,[3;3;3*k*dt;3*k*dt],dt,3);end
assert(d(7)==1,'Straight rolling should be eligible');
r.pass=true;disp(r);
end
