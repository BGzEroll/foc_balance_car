function r=verify_measured_physics
p=measured_foc_parameters;cfg=p.cfg;cfg(13)=0;cfg(21:28)=0;cfg(29:30)=0;x=measured_foc_initial([0 0]);q=[0 0];w=[10 10];Ein=0;Ecu=0;Emech=0;
for k=0:3999
 old=x(1:2,:);[x,y]=measured_foc_step(x,[0 0;.2 .2],q,w,k,cfg);mid=(old+x(1:2,:))/2;
 for m=1:2
  a=7*w(m)*cfg(6)/2;v=[cos(a)*y(6,m)+sin(a)*y(7,m);-sin(a)*y(6,m)+cos(a)*y(7,m)];
  Ein=Ein+1.5*sum(v.*mid(:,m))*cfg(6);Ecu=Ecu+1.5*p.Rs*sum(mid(:,m).^2)*cfg(6);Emech=Emech+p.Kt*mid(2,m)*w(m)*cfg(6);
 end
 q=q+w*cfg(6);
end
Emag=.75*p.Ld*sum(x(1:2,:).^2,'all');res=abs(Ein-Ecu-Emech-Emag)/max(abs(Ein),1e-12);assert(res<.01);
sp=measured_sensor_parameters;clear measured_vehicle_sensors
alpha=-3*pi/180;N=5000;out=zeros(N,14);raw=[0;0;0;0;0;.0325;0;cos(alpha/2);sin(alpha/2);0;0;0;0;0];act=zeros(30,1);
for k=1:N,raw(1)=(k-1)*.005;out(k,:)=measured_vehicle_sensors(raw,act,sp.cfg)';end
Q=out(:,8:11);angle=atan2(2*(Q(:,1).*Q(:,2)+Q(:,3).*Q(:,4)),1-2*(Q(:,2).^2+Q(:,3).^2));
bias=mean(angle(end-999:end))-alpha;assert(abs(bias)<.005);
clear measured_vehicle_sensors;raw=[0;zeros(6,1);1;zeros(6,1)];act(15)=10;act(29)=10;
for k=1:100,raw(1)=(k-1)*.005;s=measured_vehicle_sensors(raw,act,sp.cfg);end
assert(abs(s(5)-.325)<.005&&abs(s(3))<.005);
r=struct('electrical_energy_relative_residual',res,'stationary_3deg_attitude_bias_rad',bias,'positive_wheel_odometry_forward_mps',s(5),'pass',true);
fid=fopen(fullfile(fileparts(mfilename('fullpath')),'physics_validation.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);disp(r);
end
