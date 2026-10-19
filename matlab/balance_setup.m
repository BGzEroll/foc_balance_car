function paths=balance_setup
% Current active paths are PID car simulation and electrical FOC.
paths.matlab=fileparts(mfilename('fullpath'));paths.root=fileparts(paths.matlab);
paths.calculation=fullfile(paths.matlab,'LQR参数计算'); % historical data, not added to active path
paths.simulation=fullfile(paths.matlab,'3D仿真');paths.results=fullfile(paths.matlab,'results');
paths.suite_results=fullfile(paths.results,'suite','y_forward');paths.foc=fullfile(paths.matlab,'FOC电机仿真');paths.measured=fullfile(paths.matlab,'实测FOC');
addpath(paths.matlab,paths.simulation,paths.foc,paths.measured);configure_balance_cache;
end
