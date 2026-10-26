function result=identify_measured_plant
% Fit measured current to CAD mechanics; entire separate runs are held out.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
folder=fullfile(root,'esp32/lqi_firmware/results/identification');out=fullfile(here,'results/measured_identification');if ~isfolder(out),mkdir(out);end
j=jsondecode(fileread(fullfile(root,'模型/参数与图片/机械参数.json')));b=j.balance_parameters;
p=struct('M',j.mass_kg.body,'mw',j.mass_kg.left_drive,'r',b.wheel_radius_m,'l',hypot(b.body_com_height_above_axle_m,b.body_com_forward_offset_m),'I',b.body_Ixx_at_com_kg_m2,'Jw',b.left_drive_J_axis_kg_m2,'Iyaw',b.effective_inplace_yaw_kg_m2,'track',b.contact_track_m,'Kt',.013311959501823515,'g',9.80665);
source=fileread(fullfile(root,'esp32/balance_car/user_lib/hw/motor.cpp'));
block=regexp(source,'const float cogging_table\[2\]\[COGGING_TABLE_SIZE\]\s*=\s*\{([\s\S]*?)\n        \};','tokens','once');
values=regexp(block{1},'[-+]?\d+\.\d+(?:[eE][-+]?\d+)?f','match');
p.cogging_table_A=reshape(str2double(erase(values,'f')),4096,2);
files=dir(fullfile(folder,'*/report.json'));train={};valid={};yawtrain={};yawvalid={};
for n=1:numel(files)
 r=jsondecode(fileread(fullfile(files(n).folder,files(n).name)));t=readtable(fullfile(files(n).folder,'samples.csv'));
 t=t(t.sample_us>=r.start_us & t.sample_us<=r.start_us+20e6,:);
 if strcmp(r.kind,'linear')
  if contains(files(n).folder,'validate'),valid{end+1}=t;else,train{end+1}=t;end %#ok<AGROW>
 else
  if contains(files(n).folder,'validate'),yawvalid{end+1}=t;else,yawtrain{end+1}=t;end %#ok<AGROW>
 end
end
assert(numel(train)>=3 && numel(valid)>=1,'Need at least three train runs and one independent validation run');
options=optimset('Display','off','MaxIter',80,'TolX',1e-6);
z=fminsearch(@(z)cost(train,p,deg2rad(3)*tanh(z)),0,options);offset=deg2rad(3)*tanh(z);
[X,Z,y]=equations(train,p,offset);[coef,constant]=solve(X,Z,y);
[Xv,Zv,yv,truth]=equations(valid,p,offset);prediction=Xv*coef+Zv*constant;
[X0,Z0,y0]=equations(valid,p,0);nominal=[1;.015;.02;.0001];
result=struct('parameters',p,'torque_scale',coef(1),'friction_N',coef(2),'viscous_N_s_m',coef(3),'pitch_damping_N_m_s',coef(4),'pitch_sensor_offset_rad',offset,'constant_force_N',constant(1),'train_runs',numel(train),'validation_runs',numel(valid),'regressor_condition',cond(X-Z*(Z\X)),'validation_nominal',metrics(X0*nominal-y0,p),'validation_identified',metrics(prediction-yv,p),'scope','Effective parameters conditional on CAD inertia and encoder/ESKF observations. Cogging torque approximated using deployed compensation table; whole-run holdout; no independent ground-position truth. Offsets are fitted nuisance terms, not sensor calibration.');
if numel(yawtrain)>=2 && ~isempty(yawvalid)
 [A,c]=yaweq(yawtrain,p);[Av,cv]=yaweq(yawvalid,p);coeff=lsqnonneg(A,c);
 result.yaw_candidate=struct('accepted_for_simulation',false,'scope','Slow-turn data give implausibly small torque gain. Keep as diagnostic; do not replace yaw mechanics with this fit.', 'validation_zero_acceleration_rms_rad_s',rms(cv),'torque_scale',coeff(1),'viscous_Nm_s',coeff(2),'coulomb_Nm',coeff(3),'validation_rate_increment_rms_rad_s',rms(Av*coeff-cv),'validation_nominal_rms_rad_s',rms(Av*[1;0;0]-cv),'regressor_condition',cond(A));
 f=figure('Visible','off','Color','w','Position',[80 80 1100 650]);tiledlayout(2,1);
 nexttile;plot([cv,Av*coeff]);legend('编码器实测角速度增量','拟合候选');ylabel('100 ms偏航角速度增量 (rad/s)');grid on;
 nexttile;plot([Av*coeff-cv,-cv]);legend('候选预测误差','零加速度基线误差');ylabel('误差 (rad/s)');xlabel('独立验证窗口');grid on;
 sgtitle('偏航候选仅用于诊断：力矩系数不合理，尚不用于替换物理模型');exportgraphics(f,fullfile(out,'yaw_diagnostic.png'),'Resolution',140);savefig(f,fullfile(out,'yaw_diagnostic.fig'));close(f);
end
noise=jsondecode(fileread(fullfile(folder,'static_noise.json')));result.sensor_noise=noise;
result.encoder=struct('counts_per_turn',4096,'observer_tau_s',.004,'scope','AS5600 quantization and deployed alpha-beta time constant; not independent wheel-radius calibration.');
fid=fopen(fullfile(out,'identified_plant.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
D=[p.M+2*p.mw+2*p.Jw/p.r^2,p.M*p.l;p.M*p.l,p.I+p.M*p.l^2];w=[1/D(1,1);.1/D(2,2)];
e0=D\(reshape(X0*nominal-y0,2,[])./w);e1=D\(reshape(prediction-yv,2,[])./w);
f=figure('Visible','off','Color','w','Position',[80 80 1150 760]);tiledlayout(3,1);
nexttile;plot([e0(1,:);e1(1,:)]');ylabel('100 ms速度增量误差 (m/s)');legend('CAD原模型','实测辨识模型');grid on;
nexttile;plot([e0(2,:);e1(2,:)]');ylabel('100 ms俯仰角速度增量误差 (rad/s)');grid on;
nexttile;plot(truth');ylabel('独立验证的速度、角速度');xlabel('验证窗口');legend('速度 m/s','俯仰角速度 rad/s');grid on;
sgtitle('独立动作验证：实际电流输入、CAD非线性动量方程；未使用验证段拟合');
exportgraphics(f,fullfile(out,'heldout_validation.png'),'Resolution',140);savefig(f,fullfile(out,'heldout_validation.fig'));close(f);disp(result);
end
function f=cost(runs,p,offset)
[X,Z,y]=equations(runs,p,offset);[a,b]=solve(X,Z,y);f=mean((X*a+Z*b-y).^2);
end
function [a,b]=solve(X,Z,y)
Xp=X-Z*(Z\X);yp=y-Z*(Z\y);a=lsqnonneg(Xp,yp);b=Z\(y-X*a);
end
function [X,Z,y,truth]=equations(runs,p,offset)
X=[];Z=[];y=[];truth=[];m=p.M+2*p.mw+2*p.Jw/p.r^2;J=p.I+p.M*p.l^2;w=[1/m;.1/J];
for n=1:numel(runs)
 t=runs{n};v=movmean(t.speed,3);q=movmean(t.pitch_rate,3);theta=t.pitch-offset;net=net_current(t,p);u=-net(:,1)+net(:,2);
 for a=1:10:height(t)-21
  k=a:a+20;s=t.sample_us(k)*1e-6;duration=s(end)-s(1);
  if duration<.095 || duration>.11 || any(t.fault(k)) || any(t.mode(k)~=2) || mean(abs(v(k)))<.008,continue;end
  v0=v(k(1));v1=v(k(end));q0=q(k(1));q1=q(k(end));th0=theta(k(1));th1=theta(k(end));
  momentum=[m*(v1-v0)+p.M*p.l*(cos(th1)*q1-cos(th0)*q0);p.M*p.l*(cos(th1)*v1-cos(th0)*v0)+J*(q1-q0)];
  input=trapz(s,u(k));localX=[p.Kt/p.r*input,-trapz(s,tanh(v(k)/.008)),-trapz(s,v(k)),0;-p.Kt*input,0,0,-trapz(s,q(k))];
  localZ=[duration;0];gravity=[0;trapz(s,p.M*p.g*p.l*sin(theta(k))-p.M*p.l*sin(theta(k)).*q(k).*v(k))];
  X=[X;localX.*w];Z=[Z;localZ.*w];y=[y;(momentum-gravity).*w];truth=[truth,[v1-v0;q1-q0]]; %#ok<AGROW>
 end
end
end
function m=metrics(e,p)
D=[p.M+2*p.mw+2*p.Jw/p.r^2,p.M*p.l;p.M*p.l,p.I+p.M*p.l^2];w=[1/D(1,1);.1/D(2,2)];delta=D\(reshape(e,2,[])./w);
m=struct('windows',size(delta,2),'delta_speed_rms_m_s',rms(delta(1,:)),'delta_pitch_rate_rms_rad_s',rms(delta(2,:)));
end
function [X,y]=yaweq(runs,p)
X=[];y=[];
for n=1:numel(runs)
 t=runs{n};s=t.sample_us*1e-6;angle=(t.angle_a+t.angle_b)*p.r/p.track;rate=movmean(gradient(angle,s),11);net=net_current(t,p);u=net(:,1)+net(:,2);
 for a=1:10:height(t)-21
  k=a:a+20;dt=s(k(end))-s(k(1));if dt<.095 || dt>.11 || any(t.fault(k)) || mean(abs(rate(k)))<.03,continue;end
  X=[X;[p.track/(2*p.r*p.Iyaw)*p.Kt*trapz(s(k),u(k)),-trapz(s(k),rate(k))/p.Iyaw,-trapz(s(k),tanh(rate(k)/.03))/p.Iyaw]];y=[y;rate(k(end))-rate(k(1))]; %#ok<AGROW>
 end
end
end

function iq=net_current(t,p)
idx=mod(floor(mod([t.angle_a,t.angle_b],2*pi)*4096/(2*pi)+.5),4096)+1;
iq=[t.iq_a-p.cogging_table_A(idx(:,1),1),t.iq_b-p.cogging_table_A(idx(:,2),2)];
end
