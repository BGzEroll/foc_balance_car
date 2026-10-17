function [simout,report]=start_balance_3d(showWindows,exportVideo,scenario,torqueLimit,controller,profile)
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
% START_BALANCE_3D 扩展版3D入口，原始balance_car_3d.slx作为基线保留。
if nargin<1,showWindows=true;end
if nargin<2,exportVideo=false;end
if nargin<3,scenario='recover';end
if nargin<4,torqueLimit=[];end
if nargin<5,controller='lqr';end
if nargin<6,profile='normal';end
[simout,report]=run_balance_case(scenario,showWindows,torqueLimit,controller,1,profile);
if exportVideo
 here=fileparts(mfilename('fullpath'));
 smwritevideo('balance_car_3d_suite',fullfile(balance_result_folder(paths.results,controller,1,profile),[lower(scenario) '_' lower(controller)],'balance_3d.mp4'), ...
  'VideoFormat','mpeg-4','FrameRate',20,'FrameSize',[960 720]);
end
fprintf('完成。原生3D查看器可以播放及旋转观察；数据在matlab/results/suite。\n');
end

