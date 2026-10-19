function [simout,report]=start_balance_3d(showWindows,exportVideo,scenario,varargin)
% Active entry is PID+FOC. Historical positional controller args are retired.
if nargin<1,showWindows=true;end
if nargin<2,exportVideo=false;end
if nargin<3,scenario='mixed';end
assert(isempty(varargin),'请用run_measured_car选择PID参数和仿真电流档位，旧LQR/理想转矩接口已停用');
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;addpath(fullfile(paths.matlab,'实测FOC'),'-begin');
[simout,report]=run_measured_car(scenario,showWindows);
if exportVideo
 folder=fullfile(paths.matlab,'实测FOC','results');smwritevideo('balance_car_pid_foc',fullfile(folder,'pid_foc_demo.mp4'),'VideoFormat','mpeg-4','FrameRate',20,'FrameSize',[960 720]);
end
end
