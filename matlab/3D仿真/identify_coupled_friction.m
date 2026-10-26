function result=identify_coupled_friction
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification');out=fullfile(here,'results/attitude_motion/coupled_friction');if ~isfolder(out),mkdir(out);end
base=jsondecode(fileread(fullfile(here,'results/attitude_motion/mean_current_model/identified_plant.json')));p=base.parameters;runs={};files=dir(fullfile(folder,'linear_mean*train/report.json'));
for n=1:numel(files),r=jsondecode(fileread(fullfile(files(n).folder,'report.json')));t=load_torque_inputs(readtable(fullfile(files(n).folder,'samples.csv')),files(n).folder);runs{end+1}=t(t.sample_us>=r.start_us & t.sample_us<r.start_us+16e6,:);end %#ok<AGROW>
opts=optimset('Display','off','MaxIter',70,'TolX',1e-6);z=fminsearch(@(z)fit(runs,p,deg2rad(3)*tanh(z)),atanh(base.pitch_sensor_offset_rad/deg2rad(3)),opts);offset=deg2rad(3)*tanh(z);[loss,c,f,condition]=fit(runs,p,offset);
result=rmfield(base,{'validation_nominal','validation_identified'});result.parameters.input_source='coupled_net_iq';result.torque_scale=c(1);result.friction_N=c(2);result.viscous_N_s_m=c(3);result.pitch_damping_N_m_s=c(4);result.motor_friction_N=c(5);result.motor_viscous_N_s_m=c(6);result.constant_force_N=f;result.pitch_sensor_offset_rad=offset;result.regressor_condition=condition;result.training_residual=loss;
result.scope='CAD-fixed model with separate ground drag and internal motor-relative drag/reaction. Effective fit, not a direct friction measurement. Mean FOC net-current input; independent rollout validation required.';
fid=fopen(fullfile(out,'identified_plant.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);disp(rmfield(result,{'parameters','sensor_noise','encoder'}));
end
function [loss,c,f,condition]=fit(runs,p,offset)
X=[];Z=[];y=[];mass=p.M+2*p.mw+2*p.Jw/p.r^2;J=p.I+p.M*p.l^2;w=[1/mass;.1/J];
for n=1:numel(runs)
 t=runs{n};v=movmean(t.speed,3);q=movmean(t.pitch_rate,3);th=t.pitch-offset;u=-t.mean_net_iq_a+t.mean_net_iq_b;relative=v-p.r*q;
 for a=1:10:height(t)-21
  k=a:a+20;s=t.sample_us(k)*1e-6;dt=s(end)-s(1);if dt<.095 || dt>.11 || max(diff(s))>.008 || any(t.fault(k)) || mean(abs(v(k)))<.008,continue;end
  mom=[mass*(v(k(end))-v(k(1)))+p.M*p.l*(cos(th(k(end)))*q(k(end))-cos(th(k(1)))*q(k(1)));p.M*p.l*(cos(th(k(end)))*v(k(end))-cos(th(k(1)))*v(k(1)))+J*(q(k(end))-q(k(1)))];
  input=trapz(s,u(k));sr=trapz(s,tanh(relative(k)/.008));ir=trapz(s,relative(k));
  A=[p.Kt/p.r*input,-trapz(s,tanh(v(k)/.008)),-trapz(s,v(k)),0,-sr,-ir;-p.Kt*input,0,0,-trapz(s,q(k)),p.r*sr,p.r*ir];
  gravity=[0;trapz(s,p.M*p.g*p.l*sin(th(k))-p.M*p.l*sin(th(k)).*q(k).*v(k))];X=[X;A.*w];Z=[Z;[dt;0].*w];y=[y;(mom-gravity).*w]; %#ok<AGROW>
 end
end
Xp=X-Z*(Z\X);yp=y-Z*(Z\y);c=lsqnonneg(Xp,yp);f=Z\(y-X*c);loss=mean((X*c+Z*f-y).^2);condition=cond(Xp);
end
