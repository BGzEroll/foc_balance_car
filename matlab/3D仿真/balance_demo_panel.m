function fig=balance_demo_panel
% Compatibility entry: active car simulation is PID + measured FOC only.
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
addpath(fullfile(paths.matlab,'实测FOC'),'-begin');fig=measured_pid_panel;
end
