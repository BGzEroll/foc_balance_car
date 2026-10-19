function [cfg,d,p]=measured_pid_config(name,scenario,envelope)
% Gains stay in equivalent total Nm/rad etc, then convert explicitly to A.
if nargin<1,name='selected';end
if nargin<2,scenario=10;end
if nargin<3,envelope='explore6';end
p=measured_foc_parameters;d=pid_balance_parameters('normal');
d.velocity_integral_limit=1.2;d.speed_limit=1.8;d.pitch_target_limit=deg2rad(25);
if ~strcmp(name,'baseline')
 f=fullfile(fileparts(mfilename('fullpath')),'pid_selected_parameters.json');
 if isfile(f),s=jsondecode(fileread(f));keys=fieldnames(s.gains);for k=1:numel(keys),d.(keys{k})=s.gains.(keys{k});end;end
end
if strcmp(envelope,'bench')
 p.Imax=.7;d.speed_limit=.65;d.pitch_target_limit=deg2rad(8);
else
 levels={'explore1p5','explore3','explore6','explore10'};amps=[1.5 3 6 10];idx=find(strcmp(envelope,levels),1);assert(~isempty(idx),'Unknown simulation envelope');p.Imax=amps(idx);
 p.ratio=.45;p.voltage_cap=p.Vbus;
end
p.cfg(9)=p.Vbus;p.cfg(10)=p.ratio;p.cfg(11)=p.voltage_cap;p.cfg(12)=p.Imax;
cfg=[scenario p.Imax d.pitch_trim zeros(1,12) 2 d.pitch_kp/p.Kt d.pitch_ki/p.Kt d.pitch_kd/p.Kt ...
 d.velocity_kp d.velocity_ki d.position_kp d.speed_limit d.pitch_target_limit d.velocity_integral_limit d.pitch_integral_limit ...
 d.yaw_kp/p.Kt d.yaw_ki/p.Kt d.yaw_kd/p.Kt d.yaw_integral_limit d.Ts 1 1 0 0 0 p.Kt];
d.controller='PID_FOC';d.envelope=envelope;d.tuning=name;d.config=cfg;d.current_parameters=p;
d.motion_profile=struct('name','mixed','durations',[6 11 9 10 9 13 18 26 34 39 7]);
end
