function result=design_pid_current
% Current-output cascade. Same CAD mechanics as LQI, no Riccati design.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
j=jsondecode(fileread(fullfile(root,'模型/参数与图片/机械参数.json')));b=j.balance_parameters;
p=struct('M',j.mass_kg.body,'mw',j.mass_kg.left_drive,'r',b.wheel_radius_m,...
 'l',hypot(b.body_com_height_above_axle_m,b.body_com_forward_offset_m),'I',b.body_Ixx_at_com_kg_m2,...
 'Jw',b.left_drive_J_axis_kg_m2,'Iyaw',b.effective_inplace_yaw_kg_m2,'track',b.contact_track_m,'g',9.80665,'Kt',.013311959501823515);
D=[p.M+2*p.mw+2*p.Jw/p.r^2 p.M*p.l;p.M*p.l p.I+p.M*p.l^2];
ag=D\[0;p.M*p.g*p.l];bg=D\[1/p.r;-1]*p.Kt;
A=[0 ag(1) 0;0 0 1;0 ag(2) 0];B=[bg(1);0;bg(2)];
cases=struct('kt',{1,.7,1.3},'lag',{.003,.005,.001},'delay',{1,2,0},'friction',{.015,.08,.25});
for k=1:3
 E=expm([A cases(k).kt*B;zeros(1,4)]*.005);cases(k).Ad=E(1:3,1:3);cases(k).Bd=E(1:3,4);
end
profiles=struct([]);index=0;
for pos=[.7 1 1.3]
 for vp=[.5 .67 .85]
  for vi=[.1 .2 .3]
   for pp=[20 24 30]
    for pd=[1.3 1.8 2.4]
     g=[vp vi pd pp 0 3.1 .8 .7 pos -.001564208089];radii=zeros(1,3);
     for c=1:3
      n=11+cases(c).delay;F=zeros(n);
      for col=1:n,z=zeros(n,1);z(col)=1;F(:,col)=linear_step(z,g,p,cases(c));end
      radii(c)=max(abs(eig(F)));
     end
     index=index+1;profiles(index).gains=g;profiles(index).worst_radius=max(radii);
    end
   end
  end
 end
end
stable=find([profiles.worst_radius]<.9995);assert(~isempty(stable),'No stable PID profile');
[~,order]=sort([profiles(stable).worst_radius]);shortlist=stable(order(1:min(5,numel(order))));scores=inf(size(shortlist));runs=cell(size(shortlist));
for k=1:numel(shortlist)
 gain=profiles(shortlist(k)).gains;tests=cell(1,9);q=0;
 for c=1:3
  for scenario={'recover','push','drive'}
   z=nonlinear(gain,p,cases(c),scenario{1});q=q+1;tests{q}=z.metrics;
  end
 end
 metrics=[tests{:}];runs{k}=metrics;
 if ~any([metrics.fell])
  scores(k)=mean([metrics.position_rms_m])+.02*mean([metrics.pitch_rms_deg])+.03*mean([metrics.current_rms_A]);
 end
 fprintf('PID profile=%d radius=%.7f falls=%d score=%.6g\n',shortlist(k),profiles(shortlist(k)).worst_radius,sum([metrics.fell]),scores(k));
end
[score,best]=min(scores);assert(isfinite(score),'Nonlinear PID screen failed');selected=profiles(shortlist(best));g=selected.gains;
% Yaw loop is independent of the planar gain screen.
yawRadius=0;
for c=1:3
 n=6+cases(c).delay;F=zeros(n);
 for col=1:n,z=zeros(n,1);z(col)=1;F(:,col)=yaw_step(z,g,p,cases(c));end
 yawRadius=max(yawRadius,max(abs(eig(F))));
end
assert(yawRadius<1,'Yaw PID unstable in the stated linear cases');
names={'velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad'};
design=struct('schema','pid_current_v1','gain_names',{names},'gains',g,'Ts_s',.005,'actual_dt',true,...
 'current_limit_each_A',6,'phase_overcurrent_A',6,'voltage_bus_ratio',.48,'speed_limit_m_s',.35,'pitch_target_limit_rad',8*pi/180,...
 'velocity_integral_limit_m',.5,'pitch_integral_limit_rad_s',.25,'yaw_integral_limit_rad_s',.5,...
 'encoder_ab_us',4000,'worst_planar_radius',selected.worst_radius,'worst_yaw_radius',yawRadius,...
 'scope','CAD-based cascade PID screen with sensor/encoder lag, torque/current-delay/friction uncertainty. Initial candidate only; not a powered ground validation. Inner pitch I is zero to avoid redundant cascaded integrators.');
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(uint8(unicode2native(jsonencode(design),'UTF-8')));
sha=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));design.design_sha256=sha;
folder=fullfile(root,'esp32/lqi_firmware');fid=fopen(fullfile(folder,'pid_design.json'),'w');fprintf(fid,'%s',jsonencode(design,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(folder,'selected_pid.json'),'w');fprintf(fid,'%s',jsonencode(design,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(root,'esp32/balance_car/user_lib/pid_config.h'),'w');
fprintf(fid,'#pragma once\n#include <stdint.h>\nnamespace pid_config {\nconstexpr float DEFAULT_GAINS[10]={');
fprintf(fid,'%.9ef,',g(1:9));fprintf(fid,'%.9ef};\n',g(10));
fprintf(fid,'constexpr float SPEED_LIMIT=.35f;\nconstexpr float PITCH_TARGET_LIMIT=.1396263402f;\nconstexpr float VELOCITY_INTEGRAL_LIMIT=.5f;\nconstexpr float PITCH_INTEGRAL_LIMIT=.25f;\nconstexpr float YAW_INTEGRAL_LIMIT=.5f;\nconstexpr uint32_t REVISION=%uU;\nconstexpr const char *DESIGN_SHA256="%s";\n}\n',uint32(hex2dec(sha(1:8))),sha);fclose(fid);
out=fullfile(here,'results/pid_current');if ~isfolder(out),mkdir(out);end
result=struct('selected',design,'profiles',profiles,'shortlist',shortlist,'scores',scores,'nonlinear',{runs},'mechanics',p);
fid=fopen(fullfile(out,'screen.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
z=nonlinear(g,p,cases(1),'drive');writetable(z.trace,fullfile(out,'reference.csv'));
fig=figure('Visible','off','Position',[80 80 1200 850]);tiledlayout(3,1);
nexttile;plot(z.trace.time_s,100*z.trace.true_position_m,z.trace.time_s,100*z.trace.target_position_m,'--');grid on;ylabel('位移 (cm)');legend('模型真实位移','指令');
nexttile;plot(z.trace.time_s,z.trace.pitch_rad*180/pi,z.trace.time_s,z.trace.target_pitch_rad*180/pi,'--');grid on;ylabel('俯仰 (°)');
nexttile;plot(z.trace.time_s,z.trace.command_A);grid on;ylabel('每轮电流 (A)');xlabel('时间 (s)');
sgtitle('PID初始候选：位置P→速度PI→姿态PD；仿真筛选，尚未落地验证');exportgraphics(fig,fullfile(out,'candidate.png'),'Resolution',140);close(fig);
disp(design);
end

function next=linear_step(z,g,p,c)
Ts=.005;x=z(1:3);m=z(4:5);e=z(6:7);iq=z(8);distance=z(9);previous=z(10);vi=z(11);
f=exp(-Ts/.0057);m=f*m+(1-f)*x(2:3);
w=x(1)/p.r-x(3);f=exp(-Ts/.002);e(1)=w+(e(1)-w)*f;e(2)=e(1)+(e(2)-e(1))*f;
v=p.r*(e(2)+m(2));distance=distance+Ts*(previous+v)/2;ve=-g(9)*distance-v;vi=vi+Ts*ve;
u=g(4)*(m(1)-g(1)*ve-g(2)*vi)+g(3)*m(2);
if c.delay>0,target=z(12);queue=[z(13:end);u];else,target=u;queue=[];end
iq=target+(iq-target)*exp(-Ts/c.lag);x=c.Ad*x+c.Bd*iq;
next=[x;m;e;iq;distance;v;vi;queue];
end

function next=yaw_step(z,g,p,c)
Ts=.005;x=z(1:2);m=z(3:4);iq=z(5);yi=z(6);f=exp(-Ts/.0057);m=f*m+(1-f)*x;yi=yi-Ts*m(1);
u=-g(6)*m(1)+g(7)*yi-g(8)*m(2);
if c.delay>0,target=z(7);queue=[z(8:end);u];else,target=u;queue=[];end
iq=target+(iq-target)*exp(-Ts/c.lag);b=p.track/(2*p.r*p.Iyaw)*p.Kt*c.kt;
x=[x(1)+Ts*x(2)+.5*Ts^2*b*iq;x(2)+Ts*b*iq];next=[x;m;iq;yi;queue];
end

function z=nonlinear(g,p,c,scenario)
duration=12;if strcmp(scenario,'drive'),duration=24;end
x=zeros(4,1);if strcmp(scenario,'recover'),x(3)=5*pi/180;end
sensor=x(3:4);encoder=zeros(2,1);iq=0;target=0;queue=zeros(c.delay,1);
distance=0;previous=0;vi=0;pii=0;now=0;k=0;fell=false;record=zeros(6000,15);
while now<duration
 k=k+1;dt=.005*(1+.03*sin(k*.7));now=now+dt;f=exp(-dt/.0057);sensor=f*sensor+(1-f)*x(3:4);
 wheel=x(2)/p.r-x(4);f=exp(-dt/.002);encoder(1)=wheel+(encoder(1)-wheel)*f;encoder(2)=encoder(1)+(encoder(2)-encoder(1))*f;
 speed=p.r*(encoder(2)+sensor(2));distance=distance+dt*(previous+speed)/2;previous=speed;
 ref=0;rv=0;if strcmp(scenario,'drive'),[a,b]=move(now,2,6,.12);[d,e]=move(now,14,6,-.12);ref=a+d;rv=b+e;end
 measured_pitch=g(10)+sensor(1)+.0003*sin(2*pi*3*now);rate=sensor(2)+.003*sin(2*pi*17*now);
 vref=clamp(rv+g(9)*(ref-distance),.35);ve=vref-speed;candidate=clamp(vi+dt*ve,.5);
 lean=g(1)*ve+g(2)*candidate;pitch_target=g(10)+clamp(lean,8*pi/180);pe=measured_pitch-pitch_target;
 pc=0;if g(5)>0,pc=clamp(pii+dt*pe,.25);end
 common=g(4)*pe+g(5)*pc+g(3)*rate;command=clamp(common/2,6);sat=abs(common/2)>6;
 if ~sat,pii=pc;if abs(lean)<=8*pi/180,vi=candidate;end,end
 if c.delay>0,target=queue(1);queue=[queue(2:end);2*command];else,target=2*command;end
 record(k,:)=[now dt speed measured_pitch rate ref rv command vi pii distance x(1) x(3) pitch_target sat];
 force=0;if strcmp(scenario,'push') && now>=2 && now<2.15,force=.15;end
 for sub=1:5
  h=dt/5;iq=target+(iq-target)*exp(-h/c.lag);f0=@(y)plant(y,iq,p,c,force);
  a=f0(x);b=f0(x+h*a/2);d=f0(x+h*b/2);e=f0(x+h*d);x=x+h*(a+2*b+2*d+e)/6;
 end
 if abs(x(3))>pi/4 || any(~isfinite(x)),fell=true;break;end
end
record=record(1:k,:);z.trace=array2table(record,'VariableNames',{'time_s','dt_s','speed_m_s','pitch_rad','pitch_rate_rad_s','target_position_m','target_speed_m_s','command_A','velocity_integral','pitch_integral','odometry_m','true_position_m','true_pitch_rad','target_pitch_rad','saturated'});
z.metrics=struct('scenario',scenario,'kt_scale',c.kt,'delay_cycles',c.delay,'friction_N',c.friction,'fell',fell,...
 'position_rms_m',rms(record(:,12)-record(:,6)),'pitch_rms_deg',rms(record(:,13))*180/pi,'peak_pitch_deg',max(abs(record(:,13)))*180/pi,...
 'current_rms_A',rms(record(:,8)),'current_peak_A',max(abs(record(:,8))),'saturation_fraction',mean(record(:,15)));
end

function dx=plant(x,current,p,c,force)
theta=x(3);q=x(4);D=[p.M+2*p.mw+2*p.Jw/p.r^2 p.M*p.l*cos(theta);p.M*p.l*cos(theta) p.I+p.M*p.l^2];
tau=p.Kt*c.kt*current;cart=tau/p.r+p.M*p.l*sin(theta)*q^2+force;body=p.M*p.g*p.l*sin(theta)-tau-.0001*q;
required=cart-D(1,2)*body/D(2,2);
if abs(x(2))<.001 && abs(required)<=c.friction,a=[0;body/D(2,2)];else,dir=sign(x(2));if abs(x(2))<.001,dir=sign(required);end,a=D\[cart-.02*x(2)-c.friction*dir;body];end
dx=[x(2);a(1);q;a(2)];
end
function y=clamp(x,limit),y=max(-limit,min(limit,x));end
function [s,v]=move(t,start,T,amp)
u=max(0,min(1,(t-start)/T));s=amp*(10*u^3-15*u^4+6*u^5);v=amp/T*(30*u^2-60*u^3+30*u^4);
end
