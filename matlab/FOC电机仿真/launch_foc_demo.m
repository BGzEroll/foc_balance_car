% 打开本文件，修改工况后点击“运行”。复用当前MATLAB，不启动新桌面。
% current：锁轴电流阶跃；reverse：转矩反向；load：带载；voltage：电压饱和。
addpath(fileparts(fileparts(mfilename('fullpath'))));balance_setup;
scenario='current';
[simout,report]=run_foc_demo(scenario,true);
