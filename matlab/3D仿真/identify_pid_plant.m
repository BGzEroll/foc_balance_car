function result=identify_pid_plant
% Fit integrated mechanical equations; reserve the third run for validation.
here=fileparts(mfilename('fullpath'));folder=fullfile(here,'results/pid_refine/identification');
j=jsondecode(fileread(fullfile(fileparts(fileparts(here)),'模型/参数与图片/机械参数.json')));b=j.balance_parameters;
p=struct('M',j.mass_kg.body,'mw',j.mass_kg.left_drive,'r',b.wheel_radius_m,...
 'l',hypot(b.body_com_height_above_axle_m,b.body_com_forward_offset_m),'I',b.body_Ixx_at_com_kg_m2,...
 'Jw',b.left_drive_J_axis_kg_m2,'Kt',.013311959501823515,'g',9.80665);
D=[p.M+2*p.mw+2*p.Jw/p.r^2,p.M*p.l;p.M*p.l,p.I+p.M*p.l^2];
[X1,Z1,y1,m1]=equations(readtable(fullfile(folder,'train_samples.csv')),p,D);
[X2,Z2,y2,m2]=equations(readtable(fullfile(folder,'validate_samples.csv')),p,D);
X=[X1;X2];Z=[Z1;Z2];y=[y1;y2];
Xp=X-Z*(Z\X);yp=y-Z*(Z\y);
positive=lsqnonneg(Xp,yp);offset=Z\(y-X*positive);
result=struct('torque_scale',positive(1),'friction_N',positive(2),...
 'viscous_N_s_m',positive(3),'pitch_damping_N_m_s',positive(4),...
 'pitch_sensor_offset_rad',offset(1),'constant_force_N',offset(2),...
 'regressor_condition',cond(Xp),'train_windows',m1.count+m2.count,...
 'method','100 ms integrated CAD mechanical equations, positive torque/friction/damping; first two runs fitted, third run held out. Offset includes IMU and COM alignment; not an accelerometer calibration.');
[Xv,Zv,yv,mv]=equations(readtable(fullfile(folder,'repeat_samples.csv')),p,D);
nominal=[1;.015;.02;.0001];
result.train=metrics(X*positive+Z*offset-y,[m1;m2],D);
result.validation_before=metrics(Xv*nominal-yv,mv,D);
result.validation_offset_only=metrics(Xv*nominal+Zv*offset-yv,mv,D);
result.validation_after=metrics(Xv*positive+Zv*offset-yv,mv,D);
result.parameters=p;
fid=fopen(fullfile(folder,'identified_plant.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
before=D\(reshape(Xv*nominal-yv,2,[])./mv.weights);
after=D\(reshape(Xv*positive+Zv*offset-yv,2,[])./mv.weights);
fig=figure('Visible','off','Position',[80 80 1100 650]);tiledlayout(2,1);
nexttile;plot(before(1,:));hold on;plot(after(1,:));grid on;ylabel('100 ms 速度增量误差 (m/s)');legend('原模型','辨识模型');
nexttile;plot(before(2,:));hold on;plot(after(2,:));grid on;ylabel('100 ms 角速度增量误差 (rad/s)');xlabel('独立验证窗口');
sgtitle('第三次实机往返：模型短时预测误差，未参与参数拟合');
exportgraphics(fig,fullfile(folder,'heldout_validation.png'),'Resolution',140);close(fig);
disp(result);
end

function [X,Z,y,meta]=equations(t,p,D)
% Keep overlapping windows, excluding stationary friction ambiguity.
n=height(t);X=[];Z=[];y=[];count=0;
v=movmean(t.speed,5);q=movmean(t.pitch_rate,5);theta=t.pitch;
u=t.command_a+t.command_b;weights=[1/D(1,1);.1/D(2,2)];
for a=1:5:n-21
 k=a:a+20;s=t.sample_us(k)*1e-6;duration=s(end)-s(1);
 if duration<.095 || duration>.11 || max(t.fault(k))>0 || mean(abs(v(k)))<.015,continue;end
 count=count+1;input=trapz(s,u(k));sv=trapz(s,tanh(v(k)/.008));iv=trapz(s,v(k));iq=trapz(s,q(k));it=trapz(s,theta(k));
 delta=D*[v(k(end))-v(k(1));q(k(end))-q(k(1))];
 localX=[p.Kt/p.r*input,-sv,-iv,0;-p.Kt*input,0,0,-iq];
 localZ=[0,duration;-p.M*p.g*p.l*duration,0];
 localY=delta-[0;p.M*p.g*p.l*it];
 X=[X;localX.*weights];Z=[Z;localZ.*weights];y=[y;localY.*weights]; %#ok<AGROW>
end
meta=struct('count',count,'weights',weights);
end

function m=metrics(residual,meta,D)
% Weighted residuals back to velocity and pitch-rate increments.
count=sum([meta.count]);e=reshape(residual,2,[])./meta(1).weights;delta=D\e;
m=struct('windows',count,'delta_speed_rms_m_s',rms(delta(1,:)),...
 'delta_pitch_rate_rms_rad_s',rms(delta(2,:)),'weighted_rms',rms(residual));
end
