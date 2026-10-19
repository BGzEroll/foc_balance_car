function [simout,report]=run_balance_case(scenario,showWindows,limit,controller,varargin)
% Current public car interface: target-current PID + measured electrical FOC.
if nargin<1,scenario='mixed';end
if nargin<2,showWindows=true;end
if nargin<4,controller='pid_current';end
assert(ismember(lower(controller),{'pid','pid_current'}),'当前仿真仅保留PID；LQR运行接口已停用');
assert(isempty(varargin),'请用run_measured_car选择当前参数档，历史理想执行器增益/运动档接口已停用');
if nargin<3||isempty(limit),if strcmpi(scenario,'mixed'),envelope='explore6';else,envelope='bench';end
else
 caps=[.7 1.5 3 6 10];keys={'bench','explore1p5','explore3','explore6','explore10'};idx=find(abs(caps-limit)<1e-9,1);assert(~isempty(idx),'电流档位可选0.7/1.5/3/6/10 A');envelope=keys{idx};
end
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;addpath(fullfile(paths.matlab,'实测FOC'),'-begin');
[simout,report]=run_measured_car(scenario,showWindows,3,true,.0005,[],'selected',envelope);
end
