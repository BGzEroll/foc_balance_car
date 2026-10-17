function [simout,report]=start_balance_3d(showWindows,exportVideo,scenario,controller,torqueLimit)
% START_BALANCE_3D 扩展版3D入口，原始balance_car_3d.slx作为基线保留。
if nargin<1,showWindows=true;end
if nargin<2,exportVideo=false;end
if nargin<3,scenario='recover';end
if nargin<4,controller='LQR';end
if nargin<5,torqueLimit=.10;end
[simout,report]=run_balance_case(scenario,controller,showWindows,torqueLimit);
if exportVideo
 here=fileparts(mfilename('fullpath'));
 smwritevideo('balance_car_3d_suite',fullfile(here,'results','suite',[lower(scenario) '_' lower(controller)],'balance_3d.mp4'), ...
  'VideoFormat','mpeg-4','FrameRate',20,'FrameSize',[960 720]);
end
fprintf('完成。原生3D查看器可以播放及旋转观察；数据在matlab/results/suite。\n');
end
