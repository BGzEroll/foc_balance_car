function [simout,report]=run_foc_demo(scenario,showWindows)
% RUN_FOC_DEMO('current')；current / reverse / load / voltage。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
if nargin<1,scenario='current';end
if nargin<2,showWindows=true;end
names={'current','reverse','load','voltage'};durations=[.42 .75 .65 .8];
index=find(strcmpi(scenario,names),1);assert(~isempty(index),'Unknown FOC scenario');
p=foc_parameters;p.config(15)=index;
model='zd2808_foc_demo';file=fullfile(paths.foc,[model '.slx']);
if ~isfile(file),build_foc_demo;end
load_system(file);mw=get_param(model,'ModelWorkspace');
assignin(mw,'foc_config',p.config);assignin(mw,'foc_ts',p.Ts);
set_param(model,'StopTime',num2str(durations(index)));
if showWindows,open_system(model);end
fprintf('FOC %s: Rs=%.3f ohm, Ld=Lq=%.1f uH, Vdc=%.2f V, p=%d\n', ...
 names{index},p.Rs,p.Ld*1e6,p.Vdc,p.pole_pairs);
simout=sim(model);
report=report_foc_demo(simout,p,names{index},paths.results,showWindows);
end
