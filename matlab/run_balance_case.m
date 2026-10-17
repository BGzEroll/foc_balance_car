function [simout,report]=run_balance_case(scenario,controller,showWindows,torqueLimit)
% RUN_BALANCE_CASE('turn','LQI')；torqueLimit=Inf表示取消单轮转矩限幅。
if nargin<1,scenario='recover';end
if nargin<2,controller='LQR';end
if nargin<3,showWindows=true;end
d=design_balance_suite;if nargin<4,torqueLimit=d.torque_limit_each;end
assert(isscalar(torqueLimit)&&torqueLimit>0,'Torque limit must be positive or Inf');
names={'recover','drive','turn','route','push','terrain','bias'};
durations=[6 11 9 10 9 13 18];idx=find(strcmpi(scenario,names),1);
assert(~isempty(idx),'Unknown scenario');assert(any(strcmpi(controller,{'LQR','LQI'})),'Choose LQR or LQI');
here=fileparts(mfilename('fullpath'));addpath(here);addpath(fullfile(here,'..','模型','仿真模型','通用URDF','meshes'));
model='balance_car_3d_suite';file=fullfile(here,[model '.slx']);if ~isfile(file),build_balance_suite;end
load_system(file);mw=get_param(model,'ModelWorkspace');cfg=d.config;
cfg(1)=strcmpi(controller,'LQI');cfg(2)=idx;cfg(3)=torqueLimit;
assignin(mw,'suite_config',cfg);assignin(mw,'terrain_on',double(idx==6));
initial=rad2deg(d.pitch_trim);if idx==1,initial=initial+8;end
set_param([model '/FloatingBase'],'SphPositionTargetRotationAngle',num2str(initial,16));
set_param(model,'StopTime',num2str(durations(idx)),'SimMechanicsOpenEditorOnUpdate',onoff(showWindows));
if showWindows,open_system(model);end
fprintf('3D %s / %s, duration=%gs, each-wheel torque limit=%g Nm\n',scenario,upper(controller),durations(idx),torqueLimit);
simout=sim(model);
out=fullfile(here,'results','suite',[lower(scenario) '_' lower(controller)]);
report=report_balance_case(simout,scenario,upper(controller),durations(idx),torqueLimit,out);
end

function s=onoff(v)
if v,s='on';else,s='off';end
end
