function results=screen_measured_pid(mode)
% Fast nonlinear longitudinal surrogate for screening, NOT 3D/FOC validation.
% Same mass/inertia/current/voltage/friction, approximate MPU observer. No tire
% impact, roll, yaw coupling, switching or full electrical integration here.
if nargin<1,mode='tune';end
here=fileparts(mfilename('fullpath'));course=measured_course_definition;[cfg,base,p]=measured_pid_config('baseline',10,'explore6');dt=.005;t=(0:dt:course.duration)';N=numel(t);
refs=zeros(N,2);for k=1:N,[r,~,~]=measured_car_reference(t(k),10,course.moves);refs(k,:)=r(1:2)';end
f=jsondecode(fileread(fullfile(here,'sensor_noise_parameters.json')));rng(20261019);
ga=zeros(N,3);gg=ga;weights=[max(1-abs(refs(:,2))/.0325/10,0) min(abs(refs(:,2))/.0325/10,1) zeros(N,1)];
u=min(abs(refs(:,2))/.0325/10,2);weights=[max(1-u,0) 1-max(1-u,0)-max(u-1,0) max(u-1,0)];
zA=randn(N,3);zG=randn(N,3);
for j=1:3
 keys={'static','10','20'};A=f.(['acc_ar_' keys{j}]);G=f.(['gyro_ar_' keys{j}]);ia=zA*A.innovation_chol';ig=zG*G.innovation_chol';
 for axis=1:3,ga(:,axis)=ga(:,axis)+weights(:,j).*filter(1,[1;A.coefficients(:,axis)],ia(:,axis));gg(:,axis)=gg(:,axis)+weights(:,j).*filter(1,[1;G.coefficients(:,axis)],ig(:,axis));end
end
noise=[-ga(:,2)/f.acc_common_scale_single_pose_prior -gg(:,1)];
if strcmp(mode,'boundary')
 [~,g]=measured_pid_config('selected',10,'explore6');rows=struct([]);
 for amps=[.7 1.5 3 6 10]
  for speed=[.6 1.5 2.5 4]
   pp=p;pp.Imax=amps;trial=course;trial.moves(1,3)=speed*3.8/1.875;shift=trial.moves(1,3)-3;trial.ramp_start=trial.ramp_start+shift;trial.hold_y=trial.hold_y+shift;
   rr=zeros(N,2);for k=1:N,[ref,~,~]=measured_car_reference(t(k),10,trial.moves);rr(k,:)=ref(1:2)';end
   gains=g;gains.speed_limit=max(1.8,1.15*speed);gains.pitch_target_limit=deg2rad(35);
   row=simulate(gains,cfg,pp,t,rr,noise,trial);row.current_limit_A=amps;row.reference_fast_peak_mps=speed;
   row.pass_screen_only=row.completed_fraction==1&&row.fast_velocity_rms<.15*speed+.08&&abs(row.hill_hold_position_error)<.05&&row.max_pitch_deg<40&&~row.no_slip_assumption_exceeded;
   row.full_3d_validated=false;row.voltage_limit_policy='0.45Vbus at constant 8.03V; simulation only';
   if isempty(rows),rows=row;else,rows(end+1)=row;end
  end
 end
 results=rows;out=fullfile(here,'pid_validation');fid=fopen(fullfile(out,'boundary_screening.json'),'w');fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true));fclose(fid);
 disp(struct2table(rows));return;
end
sets=base;names={'baseline'};
for vk=[.65 .85 1 1.2 1.4]
 for ki=[.15 .3 .5 .8 1.2]
  for kd=[.02 .025 .035 .045]
   q=base;q.velocity_kp=vk;q.velocity_ki=ki;q.pitch_kd=kd;sets(end+1)=q;names{end+1}=sprintf('v%.2f_i%.2f_d%.3f',vk,ki,kd);
  end
 end
end
rows=struct([]);best=Inf;bestData=[];
for j=1:numel(sets)
 [row,data]=simulate(sets(j),cfg,p,t,refs,noise,course);row.name=names{j};
 if isempty(rows),rows=row;else,rows(end+1)=row;end
 if j>1&&row.score<best,best=row.score;bestIdx=j;bestData=data;end
 if j==1,baseData=data;end
end
selected=sets(bestIdx);gainFields={'position_kp','velocity_kp','velocity_ki','pitch_kp','pitch_ki','pitch_kd','yaw_kp','yaw_ki','yaw_kd'};gains=struct;for k=1:numel(gainFields),gains.(gainFields{k})=selected.(gainFields{k});end
result=struct('status','surrogate_candidate_pending_full_3D_validation','gains',gains,'selected_screening',rows(bestIdx),'baseline_screening',rows(1),'scope','Nonlinear longitudinal screening only; no tire impacts or full FOC. Must validate native model.');
fid=fopen(fullfile(here,'pid_selected_parameters.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
out=fullfile(here,'pid_validation');if ~isfolder(out),mkdir(out);end
results=rows;fid=fopen(fullfile(out,'screening.json'),'w');fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true));fclose(fid);
f1=figure('Visible','off','Color','w','Position',[20 20 1200 800]);tiledlayout(3,1);nexttile;plot(t,refs(:,2),'k:',t,baseData(:,2),t,bestData(:,2));grid on;ylabel('速度 / m/s');legend('指令','原PID','候选PID');nexttile;plot(t,rad2deg(baseData(:,3)),t,rad2deg(bestData(:,3)));grid on;ylabel('真实俯仰 / deg');nexttile;plot(t,baseData(:,1)-refs(:,1),t,bestData(:,1)-refs(:,1));grid on;ylabel('路程误差 / m');xlabel('时间 / s');sgtitle('简化模型预筛选，尚非完整3D验证');exportgraphics(f1,fullfile(out,'screening.png'),'Resolution',140);savefig(f1,fullfile(out,'screening.fig'));close(f1);
disp(result);disp(rows(bestIdx));
end
function [r,data]=simulate(g,cfg,p,t,refs,noise,course)
dt=.005;N=numel(t);x=zeros(4,1);integrals=zeros(3,1);obs=0;gyro=0;accAngle=0;acc=0;data=NaN(N,5);satCount=0;
for k=1:N
 gain=dt/(1/(2*pi*44)+dt);gyro=gyro+gain*(x(4)-gyro);accAngle=accAngle+gain*(x(3)-atan2(acc,9.80665)-accAngle);
 obs=.98*(obs+(gyro+noise(k,2))*dt)+.02*(accAngle+noise(k,1));
 targetV=clip(refs(k,2)+g.position_kp*(refs(k,1)-x(1)),g.speed_limit);ve=targetV-x(2);vi=clip(integrals(1)+dt*ve,g.velocity_integral_limit);
 un=g.velocity_kp*ve+g.velocity_ki*vi;target=clip(un,g.pitch_target_limit);pe=obs-target;pii=clip(integrals(2)+dt*pe,g.pitch_integral_limit);
 iq=(g.pitch_kp*pe+g.pitch_ki*pii+g.pitch_kd*(gyro+noise(k,2)))/(2*p.Kt);
 if abs(iq)<=p.Imax,integrals(2)=pii;if abs(un)<=g.pitch_target_limit,integrals(1)=vi;end,end
 satCount=satCount+(abs(iq)>p.Imax);iq=clip(iq,p.Imax);w=x(2)/.0325-x(4);vm=min(p.voltage_cap,p.ratio*p.Vbus);
 iq=max((-vm-p.ke*w)/p.Rs,min((vm-p.ke*w)/p.Rs,iq));tau=2*(p.Kt*iq-p.B(2)*w-p.coulomb(2)*tanh(w/.5));
 slope=0;if x(1)>=course.ramp_start&&x(1)<course.ramp_start+course.ramp_run,slope=deg2rad(course.slope_deg);elseif x(1)>=course.ramp_start+course.ramp_run+course.platform_length&&x(1)<course.ramp_start+2*course.ramp_run+course.platform_length,slope=-deg2rad(course.slope_deg);end
 a=rhs(x,tau,slope);b=rhs(x+dt*a/2,tau,slope);c=rhs(x+dt*b/2,tau,slope);d=rhs(x+dt*c,tau,slope);x=x+dt*(a+2*b+2*c+d)/6;acc=a(2);data(k,:)=[x' iq];
 if abs(x(3))>pi/4||any(~isfinite(x)),break;end
end
valid=all(isfinite(data),2);fast=t>=1&t<=4.8&valid;hold=t>=12&t<=14.3&valid;
r=struct('score',1e6,'completed_fraction',sum(valid)/N,'fast_velocity_rms',Inf,'hill_hold_position_error',Inf,'max_pitch_deg',max(abs(rad2deg(data(valid,3)))),'saturation_fraction',satCount/N);
r.maximum_speed_mps=max(abs(data(valid,2)));r.peak_each_iq_A=max(abs(data(valid,5)));r.copper_energy_both_motors_J=3*p.Rs*sum(data(valid,5).^2)*dt;
ww=data(valid,2)/.0325-data(valid,4);force=2*(p.Kt*data(valid,5)-p.B(2)*ww-p.coulomb(2)*tanh(ww/.5))/.0325;
r.maximum_requested_ground_force_N=max(abs(force));r.no_slip_assumption_exceeded=r.maximum_requested_ground_force_N>.8*.4025*9.80665;
if all(valid)
 e=data(:,1)-refs(:,1);ev=data(:,2)-refs(:,2);r.fast_velocity_rms=sqrt(mean(ev(fast).^2));r.hill_hold_position_error=mean(e(hold));r.score=4*r.fast_velocity_rms+12*abs(r.hill_hold_position_error)+2*sqrt(mean(e.^2))+.01*r.max_pitch_deg+.3*r.saturation_fraction+3*abs(e(end));
else,r.score=1e5+1e5*(1-r.completed_fraction);end
end
function d=rhs(x,tau,slope)
M=.2771;l=.03652922742030107;I=.000311847668361045;mt=.4025;r=.0325;J=2.996788998409844e-5;g=9.80665;
a=mt+2*J/r^2;b=M*l*cos(x(3));c=I+M*l^2;f1=tau/r+M*l*sin(x(3))*x(4)^2-mt*g*sin(slope);f2=M*g*l*sin(x(3))-tau;det=a*c-b*b;
d=[x(2);(c*f1-b*f2)/det;x(4);(a*f2-b*f1)/det];
end
function y=clip(x,c)
y=max(-c,min(c,x));
end
