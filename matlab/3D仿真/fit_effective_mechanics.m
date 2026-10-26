function result=fit_effective_mechanics
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification');out=fullfile(here,'results/attitude_motion');
baseline=jsondecode(fileread(fullfile(here,'results/measured_identification/identified_plant.json')));p=baseline.parameters;
files=dir(fullfile(folder,'linear_*train*/report.json'));runs={};
for n=1:numel(files),r=jsondecode(fileread(fullfile(files(n).folder,'report.json')));t=readtable(fullfile(files(n).folder,'samples.csv'));runs{end+1}=t(t.sample_us>=r.start_us & t.sample_us<r.start_us+16e6,:);end %#ok<AGROW>
options=optimset('Display','off','MaxIter',60,'TolX',1e-5);z=fminsearch(@(z)objective(runs,p,deg2rad(3)*tanh(z)),atanh(baseline.pitch_sensor_offset_rad/deg2rad(3)),options);offset=deg2rad(3)*tanh(z);
[c,loss,condition]=solve(runs,p,offset);result=baseline;result.parameters.l=p.l*c(5);result.parameters.I=(p.I+p.M*p.l^2)*c(6)-p.M*result.parameters.l^2;
result.torque_scale=c(1);result.friction_N=c(2);result.viscous_N_s_m=c(3);result.pitch_damping_N_m_s=c(4);result.constant_force_N=c(7);result.pitch_sensor_offset_rad=offset;
result.effective_mechanics=struct('com_lever_scale',c(5),'pitch_inertia_about_axle_scale',c(6),'prior_com_lever_m',p.l,'prior_pitch_inertia_about_axle',p.I+p.M*p.l^2,'regularization_residual_scale',.002,'train_equation_rms',loss,'regressor_condition',condition,'accepted_for_simulation',false,'scope','Effective CAD-constrained coefficients, not measured physical mass/COM/inertia. COM lever bounded 0.5..1.5 of CAD; axle inertia bounded 0.5..3.0 of CAD; parameters require held-out rollout validation.');
assert(result.parameters.I>0,'Nonphysical fitted COM inertia');
fid=fopen(fullfile(out,'effective_mechanics.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);disp(result.effective_mechanics);
end
function loss=objective(runs,p,offset)
[~,loss]=solve(runs,p,offset);
end
function [c,loss,condition]=solve(runs,p,offset)
X=[];y=[];mass=p.M+2*p.mw+2*p.Jw/p.r^2;ml=p.M*p.l;J=p.I+p.M*p.l^2;weights=[1/mass;.15/J];
for n=1:numel(runs)
 t=runs{n};v=movmean(t.speed,3);q=movmean(t.pitch_rate,3);th=t.pitch-offset;
 point=mod(floor(mod([t.angle_a,t.angle_b],2*pi)*4096/(2*pi)+.5),4096)+1;u=-(t.iq_a-p.cogging_table_A(point(:,1),1))+(t.iq_b-p.cogging_table_A(point(:,2),2));
 for a=1:10:height(t)-21
  k=a:a+20;s=t.sample_us(k)*1e-6;dt=s(end)-s(1);if dt<.095 || dt>.11 || max(diff(s))>.008 || any(t.fault(k)) || mean(abs(v(k)))<.008,continue;end
  iq=trapz(s,q(k));input=trapz(s,u(k));coupling=cos(th(k(end)))*q(k(end))-cos(th(k(1)))*q(k(1));
  gravity=p.g*trapz(s,sin(th(k)))-trapz(s,sin(th(k)).*q(k).*v(k))-(cos(th(k(end)))*v(k(end))-cos(th(k(1)))*v(k(1)));
  A=[p.Kt/p.r*input,-trapz(s,tanh(v(k)/.008)),-trapz(s,v(k)),0,-ml*coupling,0,dt;-p.Kt*input,0,0,-iq,ml*gravity,-J*(q(k(end))-q(k(1))),0];
  X=[X;A.*weights];y=[y;[mass*(v(k(end))-v(k(1)));0].*weights]; %#ok<AGROW>
 end
end
prior=zeros(2,7);prior(1,5)=.002;prior(2,6)=.002;
opts=optimoptions('lsqlin','Display','off');c=lsqlin([X;prior],[y;.002;.002],[],[],[],[],[.2;0;0;0;.5;.5;-.2],[1.5;.5;2;.03;1.5;3;.2],[],opts);
loss=mean(([X;prior]*c-[y;.002;.002]).^2);condition=cond(X);
end
