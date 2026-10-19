% 小车当前仿真入口：PID + 实测FOC综合性能面板。
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));paths=balance_setup;
addpath(fullfile(paths.matlab,'实测FOC'),'-begin');measured_pid_panel;
