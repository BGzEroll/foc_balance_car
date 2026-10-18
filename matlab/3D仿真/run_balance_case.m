function [simout,report]=run_balance_case(scenario,showWindows,torqueLimit,controller,actuatorGain,profile)
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
% 例：run_balance_case('turn',true,[],'pid_current',[.8 1.2])。
% 第三个参数：pid_current模式为单轮iq上限/A，其他模式为Nm。
if nargin<1,scenario='recover';end
if nargin<2,showWindows=true;end
if nargin<3,torqueLimit=[];end
if nargin<4,controller='lqr';end
if nargin<5,actuatorGain=[1 1];end
if nargin<6,profile='normal';end
[cfg,d]=balance_control_config(controller,1,torqueLimit,actuatorGain,profile);torqueLimit=cfg(2);
controller=lower(d.controller);actuatorGain=cfg(32:33);
assert(isscalar(torqueLimit)&&torqueLimit>0,'输出上限须为正数或Inf；电流模式单位A，其他模式Nm');
names={'recover','drive','turn','route','push','terrain','bias','slope','slope_hold'};
durations=d.motion_profile.durations;profile=d.motion_profile.name;idx=find(strcmpi(scenario,names),1);
assert(~isempty(idx),'Unknown scenario');
here=fileparts(mfilename('fullpath'));addpath(here);configure_balance_cache;addpath(fullfile(paths.root,'模型','仿真模型','通用URDF','meshes'));
model='balance_car_3d_suite';file=fullfile(here,[model '.slx']);if ~isfile(file),build_balance_suite;end
load_system(file);mw=get_param(model,'ModelWorkspace');
assert(getSimulinkBlockHandle([model '/Balance_Controller'])>0,'请先保存并关闭旧模型，运行build_balance_suite更新LQR/PID模型');
cfg(1)=idx;cfg(2)=torqueLimit;
d.config=cfg;
assignin(mw,'suite_config',cfg);assignin(mw,'terrain_on',double(idx==6||idx>=8));
kind='mild';if idx>=8,kind='steep';end
terrain=configure_balance_terrain(model,kind);d.terrain=terrain;
set_param([model '/Sample_200Hz'],'SampleTime',num2str(d.Ts,16));
initial=rad2deg(d.pitch_trim);if idx==1,initial=initial+8;end
set_param([model '/FloatingBase'],'SphPositionTargetRotationAngle',num2str(initial,16));
set_param(model,'StopTime',num2str(durations(idx)),'SimMechanicsOpenEditorOnUpdate',onoff(showWindows));
if showWindows,open_system(model);end
fprintf('3D %s / %s, duration=%gs, command limit=%g %s, gains=[%g %g]\n',scenario,upper(controller),durations(idx),torqueLimit,d.output_unit,actuatorGain);
simout=sim(model);
out=fullfile(balance_result_folder(paths.results,controller,actuatorGain,profile),[lower(scenario) '_' controller]);
try
 drive=[];if cfg(16)>1.5,drive=d.current_parameters;end
 report=report_balance_case(simout,scenario,durations(idx),torqueLimit,out,upper(controller),actuatorGain,profile,terrain,drive);
catch err
 if ~isfolder(out),mkdir(out);end
 save(fullfile(out,'unreported_simulation.mat'),'simout','cfg','d','-v7.3');
 rethrow(err);
end
save(fullfile(out,'controller_config.mat'),'cfg','d');
fid=fopen(fullfile(out,'controller_config.json'),'w');assert(fid>=0);
fprintf(fid,'%s',jsonencode(d,PrettyPrint=true));fclose(fid);
end

function s=onoff(v)
if v,s='on';else,s='off';end
end

