function paths=balance_setup
% 工程路径初始化；可从任意工作目录调用，不切换当前目录。
paths.matlab=fileparts(mfilename('fullpath'));
paths.root=fileparts(paths.matlab);
paths.calculation=fullfile(paths.matlab,'LQR参数计算');
paths.simulation=fullfile(paths.matlab,'3D仿真');
paths.results=fullfile(paths.matlab,'results');
paths.foc=fullfile(paths.matlab,'FOC电机仿真');
addpath(paths.matlab,paths.calculation,paths.simulation,paths.foc);
configure_balance_cache;
end
